use std::collections::{HashMap, hash_map::{Values, ValuesMut}};

use rand::prelude::*;

use petgraph::graphmap::UnGraphMap;
use petgraph::visit::EdgeRef;
use rustworkx_core::Result;
use rustworkx_core::shortest_path::astar;

use crate::device::*;


const CHANGE_SIGNAL_LEVEL_FROM_GREEN_PROBABILITY: f32  = 0.95;
const CHANGE_SIGNAL_LEVEL_FROM_YELLOW_PROBABILITY: f32 = 0.70;
const CHANGE_SIGNAL_LEVEL_FROM_RED_PROBABILITY: f32    = 0.50;
const CHANGE_SIGNAL_LEVEL_FROM_BLACK_PROBABILITY: f32  = 0.0;


fn effective_propagated_signal<T, U>(
    transmitter: &T,
    receiver: &U
) -> SignalLevel 
where
    T: Transmitter,
    U: Receiver
{
    let coefficient: f32 = thread_rng().gen();
    
    let tx_signal_level = transmitter
        .tx_signal_level(&SignalType::Control)
        .unwrap_or_else(|| &SignalLevel::Black);

    let probability = signal_level_change_probability(&tx_signal_level);

    if coefficient < probability {
        let (_, propagated_signal_level) = transmitter
            .propagated_signal_to(receiver, &SignalType::Control);

        return propagated_signal_level;
    }
    else {
        let current_signal_level = receiver
            .rx_signal_level(&SignalType::Control)
            .unwrap_or_else(|| &SignalLevel::Black);

        return *current_signal_level;
    }
}

fn signal_level_change_probability(signal_level: &SignalLevel) -> f32 {
    match signal_level {
        SignalLevel::Black => CHANGE_SIGNAL_LEVEL_FROM_BLACK_PROBABILITY,
        SignalLevel::Red => CHANGE_SIGNAL_LEVEL_FROM_RED_PROBABILITY,
        SignalLevel::Yellow => CHANGE_SIGNAL_LEVEL_FROM_YELLOW_PROBABILITY,
        SignalLevel::Green => CHANGE_SIGNAL_LEVEL_FROM_GREEN_PROBABILITY,
    }
}


#[derive(Clone)]
pub struct CellularAutomaton {
    current_time_in_millis: u64,
    command_center: CommandCenter,
    radar_warfare_devices: Vec<RadarWarfareDevice>,
    destination_in_metres: Coordinates3D,
    drones: HashMap<u32, Drone>,
    topology: Topology,
    connections: UnGraphMap<u32, f32>,
    message_queue: Vec<Message>,
}

impl CellularAutomaton {
    pub fn new(
        command_center: CommandCenter,
        drones: &Vec<Drone>,
        radar_warfare_devices: Vec<RadarWarfareDevice>,
        // Scenario must be sorted in ascending order by message execution time.
        scenario: &Vec<Message>, // TODO create special struct for message queue
        topology: Topology,
    ) -> Self {
        let command_center_id = command_center.id();

        let drone_map = drones
            .iter()
            .map(|drone| (drone.id, drone.clone()))
            .collect();

        let mut cellular_automaton = Self {
            current_time_in_millis: 0,
            command_center,
            radar_warfare_devices,
            destination_in_metres: Coordinates3D::new(0.0, 0.0, 0.0),
            drones: drone_map,
            topology,
            connections: UnGraphMap::new(),
            message_queue: Vec::new(),
        };

        cellular_automaton.drones
            .values_mut()
            .for_each(|drone| drone.command_center_id = command_center_id);

        // Set initial state.
        cellular_automaton.update_connections_graph();
        cellular_automaton.update_signal_levels();
        cellular_automaton.remove_uncontrolled_drones();

        scenario
            .iter()
            .for_each(|message| cellular_automaton.add_message(*message));

        cellular_automaton
    }
    
    pub fn destination(&self) -> &Coordinates3D {
        &self.destination_in_metres
    }

    pub fn command_center(&self) -> &CommandCenter {
        &self.command_center
    }
    
    pub fn command_center_mut(&mut self) -> &mut CommandCenter {
        &mut self.command_center
    }

    pub fn drone_iter(&self) -> Values<'_, u32, Drone> { 
        self.drones.values()
    }

    pub fn drone_iter_mut(&mut self) -> ValuesMut<'_, u32, Drone> { 
        self.drones.values_mut()
    }

    pub fn rwd_iter(&self) -> std::slice::Iter<'_, RadarWarfareDevice> {
        self.radar_warfare_devices.iter()
    }

    pub fn rwd_iter_mut(
        &mut self
    ) -> std::slice::IterMut<'_, RadarWarfareDevice> {
        self.radar_warfare_devices.iter_mut()
    }

    fn remove_uncontrolled_drones(&mut self) { 
        self.drones.retain(|_, drone| drone.connected(&SignalType::Control));
    }

    fn closest_drone_to<T>(&self, object: &T) -> Option<&Drone>
    where
        T: Position
    {
        self.drones
            .values()
            .min_by_key(|drone| { drone.distance_to(object); })
    }

    fn closest_drone_to_mut<T>(&mut self, object: &T) -> Option<&mut Drone>
    where
        T: Position
    {
        self.drones
            .values_mut()
            .min_by_key(|drone| { drone.distance_to(object); })
    }
    
    fn set_command_center(&mut self, command_center: CommandCenter) {
        let cc_id = command_center.id();

        self.command_center = command_center;

        self.drones
            .values_mut()
            .for_each(|drone| drone.command_center_id = cc_id);
    }

    fn process_message(&mut self, message: &mut Message) {
        *message.message_state_mut() = MessageState::InProgress;
        
        match message.message_type() {
            MessageType::SetDestination(destination, _) =>
                if let Some(coordinates) = *destination { 
                    self.destination_in_metres = coordinates;
                },
        }
    }

    fn add_message(&mut self, message: Message) {
        self.message_queue.push(message);
    }
    
    fn process_message_queue(&mut self) {
        if self.message_queue.is_empty() {
            return;
        }
        
        let current_time = self.current_time_in_millis;

        // TODO find a way to avoid cloning without performance loss 
        // (not RefCell)
        let mut message_queue = self.message_queue.clone();

        for message in &mut message_queue {
            let message_time = message.time();
            
            if current_time >= message_time {
                self.process_message(message);
            }

            for drone in self.drones.values_mut() {
                if current_time >= message_time {
                    drone.process_message(message);
                }
            }
        }

        self.message_queue = message_queue;

        self.remove_processed_messages();
    }
    
    fn remove_processed_messages(&mut self) {
        self.message_queue.retain(|message|
            !matches!(message.message_state(), MessageState::Finished)
        );
    }

    fn update_connections_graph(&mut self) {
        self.connections.clear();
        
        let cc_node = self.connections.add_node(self.command_center.id);
        
        // Connect drones in command center's area.
        // In this area the network topology is always a Star.
        let mut connected = false;

        for drone in self.drones.values() {
            if let Some(distance) = self.command_center.connection_distance(
                drone
            ) {
                let node = self.connections.add_node(drone.id);
                
                self.connections.add_edge(cc_node, node, distance);
                
                connected = true;
            };
        } 

        // The drones in the command center area repeat the signal to each
        // other and the drones outside the area.
        // If there are no drones inside the area then the drones 
        // outside can not get any signals.
        // So, it is pointless to continue computation.
        if !connected {
            return;
        }

        if let Topology::Mesh = self.topology {
            for drone1 in self.drones.values() {
                let node1 = self.connections.add_node(drone1.id);
               
                for drone2 in self.drones.values() {
                    let node2 = self.connections.add_node(drone2.id);
            
                    if let Some(distance) = drone1.connection_distance(drone2) {
                        self.connections.add_edge(node1, node2, distance);
                    }
                }
            }
        }
    }

    fn update_signal_levels(&mut self) {
        let mut best_signal_levels: HashMap<u32, SignalLevel> = self.drones
            .keys()
            .map(|id| (*id, SignalLevel::Black))
            .collect();

        for drone in self.drones.values() {
            let shortest_path: Option<(f32, Vec<u32>)> = astar(
                &self.connections,
                self.command_center.id,
                |finish| -> Result<bool> {
                    Ok(finish == drone.id)
                },
                |edge| Ok(*edge.weight()),
                |_| Ok(0.0)
            ).expect("Failed to find the shortest path");

            let path: Vec<u32> = if let Some((_, p)) = shortest_path {
                p.clone()  
            }
            else {
                Vec::new()
            };

            for i in 0..(path.len() - 1) {
                let rx = self.drones.get(&path[i + 1])
                    .expect("Failed to get the receiver");

                let signal_level = if let Some(tx) = self.drones.get(
                    &path[i]
                ) {
                    effective_propagated_signal(tx, rx) 
                }
                else {
                    effective_propagated_signal(&self.command_center, rx) 
                };

                let best_signal_level = best_signal_levels.get_mut(&rx.id)
                    .expect("Failed to get the best signal level");

                if signal_level > *best_signal_level {
                    best_signal_levels.insert(rx.id, signal_level);
                }
            }
        }

        for (id, drone) in &mut self.drones {
            let best_signal_level = best_signal_levels.get(&id)
                .expect("Failed to get the best signal level");

            if let Some(signal_level) = drone.rx_signal_level_mut(
                &SignalType::Control
            ) {
                *signal_level = *best_signal_level;
            }
        }
    }

    fn suppress_network(&mut self) {
        // TODO describe influence of rwds on each other 
        for rwd in &self.radar_warfare_devices {
            for drone in self.drones.values_mut() {
                rwd.suppress(drone);
            }
        }
    }

    fn update_states(&mut self) {
        self.drones
            .values_mut()
            .for_each(Drone::update_state);
    }

    pub fn update_network(&mut self) {
        self.update_connections_graph();

        self.update_signal_levels();

        self.process_message_queue();

        self.suppress_network();

        self.remove_uncontrolled_drones();

        self.update_states();
       
        self.current_time_in_millis += STEP_DURATION_IN_MILLIS;
    }
}


#[derive(Clone)]
pub struct RadarWarfareDevice {
    id: u32,
    position_in_metres: Coordinates3D,
    signal_levels: HashMap<SignalType, SignalLevel>,
    area: SignalAreaType,
}

impl RadarWarfareDevice {
    pub fn new(
        position_in_metres: Coordinates3D,
        signal_levels: HashMap<SignalType, SignalLevel>,
        area: SignalAreaType
    ) -> Self {
        Self { 
            id: generate_drone_id(),
            position_in_metres, 
            signal_levels,
            area,
        }
    }
    
    pub fn id(&self) -> u32 {
        self.id
    }

    fn suppress<T>(&self, receiver: &mut T)
    where
        T: Receiver
    {
        for (signal_type, signal_level) in self.signal_levels.iter() {
            let coefficient: f32 = thread_rng().gen();
        
            let (_, tx_signal_level_at_rx) = self
                .propagated_signal_to(receiver, signal_type);

            if let Some(rx_signal_level) = receiver
                .rx_signal_level_mut(signal_type)
            {
                let suppressed_signal_level = SignalLevel::suppression(
                    &tx_signal_level_at_rx,
                    &rx_signal_level
                );

                let probability = signal_level_change_probability(
                    &signal_level
                );

                if coefficient < probability {
                    *rx_signal_level = suppressed_signal_level;
                }
            }
        }
    }
}

impl Hash for RadarWarfareDevice {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Position for RadarWarfareDevice {
    fn position(&self) -> Coordinates3D {
        self.position_in_metres
    }
}

impl Transmitter for RadarWarfareDevice {
    fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.signal_levels
    }
    
    fn tx_signal_levels_mut(
        &mut self
    ) -> &mut HashMap<SignalType, SignalLevel> {
        &mut self.signal_levels
    }

    fn area(&self) -> &SignalAreaType {
        &self.area
    }
}

impl Receiver for RadarWarfareDevice {
    fn rx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.signal_levels
    }
    
    fn rx_signal_levels_mut(
        &mut self
    ) -> &mut HashMap<SignalType, SignalLevel> {
        &mut self.signal_levels
    }
}

impl Transceiver for RadarWarfareDevice {}
