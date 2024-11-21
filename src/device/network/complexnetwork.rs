use std::collections::{HashMap, hash_map::{Values, ValuesMut}};

use petgraph::graphmap::UnGraphMap;
use petgraph::visit::EdgeRef;
use rustworkx_core::centrality::betweenness_centrality;
use rustworkx_core::dictmap::DictMap;
use rustworkx_core::Result;
use rustworkx_core::shortest_path::{dijkstra, astar};

use crate::device::*;


const SIGNAL_SPEED_IN_METRES_PER_S: f32 = 0.0002;


fn calculate_delay(distance_in_metres: f32, multiplier: f32) -> u64 {    
    if multiplier == 0.0 {
        return 0;
    }

    // Delay is in millis.
    let delay = ((distance_in_metres / (SIGNAL_SPEED_IN_METRES_PER_S * 1000.0))
        .round() * multiplier) as u64;

    let reminder = delay % STEP_DURATION_IN_MILLIS;
    
    delay - reminder
}


#[derive(Clone)]
pub struct ComplexNetwork {
    current_time_in_millis: u64,
    command_center: CommandCenter,
    radar_warfare_devices: Vec<RadarWarfareDevice>,
    destination_in_metres: Coordinates3D,
    drones: HashMap<u32, Drone>,
    topology: Topology,
    connections: UnGraphMap<u32, f32>,
    delay_multiplier: f32,
    message_queue: Vec<(Message, HashMap<u32, u64>)>,
}

impl ComplexNetwork {
    pub fn new(
        command_center: CommandCenter,
        drones: &Vec<Drone>,
        radar_warfare_devices: Vec<RadarWarfareDevice>,
        // Scenario must be sorted in ascending order by message execution time.
        scenario: &Vec<Message>, // TODO create special struct for message queue
        topology: Topology,
        delay_multiplier: f32
    ) -> Self {
        let command_center_id = command_center.id;

        let drone_map = drones
            .iter()
            .map(|drone| (drone.id, drone.clone()))
            .collect();

        let mut complex_network = Self {
            current_time_in_millis: 0,
            command_center,
            radar_warfare_devices,
            destination_in_metres: Coordinates3D::new(0.0, 0.0, 0.0),
            drones: drone_map,
            topology,
            connections: UnGraphMap::new(),
            delay_multiplier,
            message_queue: Vec::new(),
        };

        complex_network.drones
            .values_mut()
            .for_each(|drone| drone.command_center_id = command_center_id);

        // Set initial state.
        complex_network.update_connections_graph();
        complex_network.update_signal_levels();
        complex_network.remove_uncontrolled_drones();

        scenario
            .iter()
            .for_each(|message| complex_network.add_message(*message));

        complex_network
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
        let cc_id = command_center.id;

        self.command_center = command_center;

        self.drones
            .values_mut()
            .for_each(|drone| drone.command_center_id = cc_id);
    }

    fn add_message(&mut self, message: Message) {
        self.message_queue.push((message, self.delays()));
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

    fn process_message_queue(&mut self) {
        if self.message_queue.is_empty() {
            return;
        }

        let current_time = self.current_time_in_millis;
        // Preprocess delays to avoid unnecessary function calls in the loop.
        let delays = self.delays();
        
        // TODO find a way to avoid cloning without performance loss 
        // (not RefCell)
        let mut message_queue = self.message_queue.clone();

        for (message, delays_snapshot) in &mut message_queue {
            let message_time = message.time();
           
            if message_time > current_time {
                continue;
            }
            
            if !matches!(message.message_state(), MessageState::InProgress) {
                self.process_message(message);
                *delays_snapshot = delays.clone();
            }

            for drone in self.drones.values_mut() {
                // Every drone should have a delay connection even if it is 0.
                // Drones without delays are considered disconnected.
                if let Some(delay) = delays_snapshot.get(&drone.id) {
                    if current_time >= message_time + delay {
                        drone.process_message(message);
                    }
                }
            }

            // We assume that the message processing is finished if the drone
            // with the longest delay processed the message.
            if let Some(longest_delay) = delays_snapshot.values().max() {
                if current_time >= message_time + longest_delay {
                    *message.message_state_mut() = MessageState::Finished; 
                }
            }
        }

        self.message_queue = message_queue;

        self.remove_processed_messages();
    }
    
    fn remove_processed_messages(&mut self) {
        self.message_queue.retain(|(message, _)|
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
                    let (_, signal_level) = tx.propagated_signal_to(
                        rx,
                        &SignalType::Control
                    );

                    signal_level
                }
                else {
                    let tx = self.command_center.clone();

                    let (_, signal_level) = tx.propagated_signal_to(
                        rx,
                        &SignalType::Control
                    );

                    signal_level
                };

                let curr_signal_level = best_signal_levels.get_mut(&rx.id)
                    .expect("Failed to get the best signal level");

                if signal_level > *curr_signal_level {
                    best_signal_levels.insert(rx.id, signal_level);
                }
            }
        }

        for (id, drone) in &mut self.drones {
            let best_signal_level = best_signal_levels.get(&id)
                .expect("Failed to get the best signal level");

            if let Some(drone_signal_level) = drone.rx_signal_level_mut(
                &SignalType::Control
            ) {
                *drone_signal_level = *best_signal_level;
            }
        }
    }

    fn suppress_network(&mut self) {
        // TODO implement suppression/amplification of other rwds in area.
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

    // Connections graph should already be created and populated before delay
    // calculation.
    fn delays(&self) -> HashMap<u32, u64> {
        let mut delays: HashMap<u32, u64> = HashMap::new();

        let distances = self.single_source_dijkstra(self.command_center.id)
            .expect("Failed to find the shortest paths");

        for (device_id, distance) in distances.iter() {
            let delay = calculate_delay(*distance, self.delay_multiplier);
            
            delays.insert(*device_id, delay);
        }

        delays
    }

    fn single_source_dijkstra(
        &self,
        source_drone_id: u32
    ) -> Result<DictMap<u32, f32>> {
        dijkstra(
            &self.connections,
            source_drone_id,
            None,
            |edge| Ok(*edge.weight()),
            None
        )
    }

    pub fn diameter(&self) -> Option<f32> {
        let shortest_paths: Vec<DictMap<u32, f32>> = self.connections
            .nodes()
            .map(|drone_id| self.single_source_dijkstra(drone_id).unwrap())
            .collect();

        let diameter = shortest_paths
            .iter()
            .flat_map(|dictmap| dictmap.values())
            .fold(0f32, |a, &b| a.max(b));
        
        Some(diameter)
    }

    pub fn node_load(&self) -> Vec<Option<f64>> {
        betweenness_centrality(&self.connections, true, true, 50)
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
        for signal_type in self.signal_levels.keys() {
            let (_, suppression_signal_level) = self.propagated_signal_to(
                receiver,
                signal_type
            );

            if let Some(rx_signal_level) = receiver
                .rx_signal_level_mut(signal_type)
            {
                *rx_signal_level = SignalLevel::suppression(
                    &suppression_signal_level,
                    rx_signal_level
                );
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


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn suppress_tranceivers() {
        let rwd = RadarWarfareDevice::new(
            Coordinates3D::new(0.0, 0.0, 0.0),
            HashMap::from([
                (SignalType::Control, SignalLevel::Green),
                (SignalType::GPS, SignalLevel::Green)
            ]),
            SignalAreaType::Dome(RWD_CONTROL_RADIUS_IN_METRES)
        );

        let mut drone = Drone::new(Coordinates3D::new(0.0, 0.0, 0.0));
        
        let drone_control_signal_level = drone.signal_levels
            .get_mut(&SignalType::Control)
            .unwrap();
        *drone_control_signal_level = SignalLevel::Green;

        let mut command_center = CommandCenter::new(
            Coordinates3D::new(0.0, 0.0, 0.0),
            SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
        );

        rwd.suppress(&mut drone);
        rwd.suppress(&mut command_center);

        assert_eq!(
            *drone.signal_levels.get(&SignalType::Control).unwrap(),
            SignalLevel::Black
        );
        assert_eq!(
            *drone.signal_levels.get(&SignalType::GPS).unwrap(),
            SignalLevel::Black
        );
        assert_eq!(
            *command_center.signal_levels.get(&SignalType::Control).unwrap(),
            SignalLevel::Black
        );
    }
    #[test]
    fn check_delays() {
        let command_center = CommandCenter::new(
            Coordinates3D::new(0.0, 0.0, 0.0),
            SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES)
        );

        let drones: Vec<Drone> = vec![
            Drone::new(Coordinates3D::new(5.0, 0.0, 0.0)),
        ];
        let drone_id = drones[0].id;

        let mut network = ComplexNetwork::new(
            command_center.clone(),
            drones.clone(),
            Vec::new(),
            Vec::new(),
            Topology::Mesh,
            0.0
        );

        let expected_delays = HashMap::from([
            (network.command_center.id, 0),
            (drone_id, 0)
        ]);
        let delays = network.delays();

        assert!(
            expected_delays.len() == delays.len() &&
            expected_delays.keys().all(|k| delays.contains_key(k))
        );

        network.delay_multiplier = 1.0;

        let expected_delays = HashMap::from([
            (network.command_center.id, 0),
            (drone_id, 50)
        ]);
        let delays = network.delays();

        assert!(
            expected_delays.len() == delays.len() &&
            expected_delays.keys().all(|k| delays.contains_key(k))
        );
    }

    #[test]
    fn propagate_signal_level() {
        let command_center = CommandCenter::new(
            Coordinates3D::new(0.0, 0.0, 0.0),
            SignalAreaType::Dome(MAX_DRONE_BROADCAST_RADIUS_IN_METRES)
        );

        // Network 1: full mesh with edge weight 1.0.
        let drones1: Vec<Drone> = vec![
            Drone::new(Coordinates3D::new(1.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(2.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(3.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(4.0, 0.0, 0.0)),
        ];
        let ids1: Vec<u32> = drones1
            .iter()
            .map(|drone| drone.id)
            .collect();
        let mut network1 = ComplexNetwork::new(
            command_center.clone(),
            drones1,
            Vec::new(),
            Vec::new(),
            Topology::Mesh,
            0.0
        );

        network1.update_signal_levels();

        let expected_signal_levels1 = HashMap::from([
            (ids1[0], SignalLevel::Green), 
            (ids1[1], SignalLevel::Green), 
            (ids1[2], SignalLevel::Green), 
            (ids1[3], SignalLevel::Green), 
        ]);
        let signal_levels1: HashMap<u32, SignalLevel> = network1.drones
            .values()
            .map(|drone| (
                drone.id,
                *drone.rx_signal_level(&SignalType::Control).unwrap()
            ))
            .collect();

        // Network 2:
        // 
        // A -(7.0)- B -(7.0)- C -(7.0)- D -(7.0)- E 
        //
        let drones2: Vec<Drone> = vec![
            Drone::new(Coordinates3D::new(7.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(14.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(21.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(28.0, 0.0, 0.0)),
        ];
        let ids2: Vec<u32> = drones2
            .iter()
            .map(|drone| drone.id)
            .collect();
        let mut network2 = ComplexNetwork::new(
            command_center.clone(),
            drones2,
            Vec::new(),
            Vec::new(),
            Topology::Mesh,
            0.0
        );

        network2.update_signal_levels();

        let expected_signal_levels2 = HashMap::from([
            (ids2[0], SignalLevel::Yellow), 
            (ids2[1], SignalLevel::Red), 
            (ids2[2], SignalLevel::Black), 
            (ids2[3], SignalLevel::Black), 
        ]);
        let signal_levels2: HashMap<u32, SignalLevel> = network2.drones
            .values()
            .map(|drone| (
                drone.id,
                *drone.rx_signal_level(&SignalType::Control).unwrap()
            ))
            .collect();

        assert!(
            expected_signal_levels1.len() == signal_levels1.len() &&
            expected_signal_levels1
                .keys()
                .all(|k| signal_levels1.contains_key(k))
        );
        assert!(
            expected_signal_levels2.len() == signal_levels2.len() &&
            expected_signal_levels2
                .keys()
                .all(|k| signal_levels2.contains_key(k))
        );
    }

    #[test]
    fn network_diameter() {
        // Network 1: full mesh with edge weight 1.0.
        let command_center1 = CommandCenter::new(
            Coordinates3D::new(0.0, 0.0, 0.0),
            SignalAreaType::Dome(MAX_DRONE_BROADCAST_RADIUS_IN_METRES)
        );
        let drones1: Vec<Drone> = vec![
            Drone::new(Coordinates3D::new(1.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(2.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(3.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(4.0, 0.0, 0.0)),
        ];
        let network1 = ComplexNetwork::new(
            command_center1,
            drones1,
            Vec::new(),
            Vec::new(),
            Topology::Mesh,
            0.0
        );

        let diameter1 = network1.diameter().unwrap();

        // Network 2:
        // 
        // A -(7.0)- B -(7.0)- C -(7.0)- D -(7.0)- E 
        //
        let command_center2 = CommandCenter::new(
            Coordinates3D::new(0.0, 0.0, 0.0),
            SignalAreaType::Dome(MAX_DRONE_BROADCAST_RADIUS_IN_METRES)
        ); 
        let drones2: Vec<Drone> = vec![
            Drone::new(Coordinates3D::new(7.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(14.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(21.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(28.0, 0.0, 0.0)),
        ];
        let network2 = ComplexNetwork::new(
            command_center2,
            drones2,
            Vec::new(),
            Vec::new(),
            Topology::Mesh,
            0.0
        );

        let diameter2 = network2.diameter().unwrap();
        
        // Network 3:
        //                      D
        //                      |
        //                    (7.0)
        //                      |
        //  A -(7.0)- B -(7.0)- C
        //                      |
        //                    (7.0)
        //                      |
        //                      E
        //
        let command_center3 = CommandCenter::new(
            Coordinates3D::new(0.0, 0.0, 0.0),
            SignalAreaType::Dome(MAX_DRONE_BROADCAST_RADIUS_IN_METRES)
        ); 
        let drones3: Vec<Drone> = vec![
            Drone::new(Coordinates3D::new(7.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(14.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(21.0, 0.0, 0.0)),
            Drone::new(Coordinates3D::new(21.0, 0.0, 0.0)),
        ];
        let network3 = ComplexNetwork::new(
            command_center3,
            drones3,
            Vec::new(),
            Vec::new(),
            Topology::Mesh,
            0.0
        );

        let diameter3 = network3.diameter().unwrap();
    
        assert_eq!(diameter1, 4.0);
        assert_eq!(diameter2, 28.0);
        assert_eq!(diameter3, 21.0);
    }
}
