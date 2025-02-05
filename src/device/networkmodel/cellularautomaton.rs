use std::collections::{HashMap, hash_map::Values};

use rand::prelude::*;

use crate::device::*;

use super::*;


const CHANGE_SIGNAL_LEVEL_FROM_GREEN_PROBABILITY: f32  = 0.95;
const CHANGE_SIGNAL_LEVEL_FROM_YELLOW_PROBABILITY: f32 = 0.70;
const CHANGE_SIGNAL_LEVEL_FROM_RED_PROBABILITY: f32    = 0.50;
const CHANGE_SIGNAL_LEVEL_FROM_BLACK_PROBABILITY: f32  = 0.0;

fn try_set_best_signal_level_with_cc(
    tx: &CommandCenter,
    rx: &Drone,
    signal_levels: &mut HashMap<DeviceId, SignalLevel>,
    signal_type: &SignalType,
) {
    try_set_best_signal_level(tx, rx, signal_levels, signal_type);
}

fn try_set_best_signal_level_with_drones(
    tx: &Drone,
    rx: &Drone,
    signal_levels: &mut HashMap<DeviceId, SignalLevel>,
    signal_type: &SignalType,
) {
    try_set_best_signal_level(tx, rx, signal_levels, signal_type);
}

fn try_set_best_signal_level<T: Transmitter, R: Receiver>(
    tx: &T,
    rx: &R,
    signal_levels: &mut HashMap<DeviceId, SignalLevel>,
    signal_type: &SignalType,
) {
    let current_signal_level = signal_levels
        .get(&rx.id())
        .unwrap_or(&NO_SIGNAL_LEVEL);
    let signal_level = effective_propagated_signal(tx, rx, signal_type);

    if signal_level > *current_signal_level {
        signal_levels.insert(rx.id(), signal_level);
    }
}

fn effective_propagated_signal<T: Transmitter, R: Receiver>(
    tx: &T,
    rx: &R,
    signal_type: &SignalType
) -> SignalLevel {
    let tx_signal_level = tx.tx_signal_level(signal_type);

    let probability = signal_level_change_probability(&tx_signal_level);
    let coefficient: f32 = thread_rng().gen();
    
    if coefficient < probability {
        let propagated_signal_level = tx.propagated_signal_level_at(
            rx, 
            signal_type
        );
        return propagated_signal_level;
    } else {
        let current_signal_level = rx.rx_signal_level(signal_type);
        return *current_signal_level;
    }
}

fn signal_level_change_probability(signal_level: &SignalLevel) -> f32 {
    if signal_level.is_black() {
        CHANGE_SIGNAL_LEVEL_FROM_BLACK_PROBABILITY
    } else if signal_level.is_red() {
        CHANGE_SIGNAL_LEVEL_FROM_RED_PROBABILITY
    } else if signal_level.is_yellow() {
        CHANGE_SIGNAL_LEVEL_FROM_YELLOW_PROBABILITY
    } else {
        CHANGE_SIGNAL_LEVEL_FROM_GREEN_PROBABILITY
    }
}

fn try_preprocess_message(
    current_time: Millisecond,
    message: &mut Message
) -> Result<(), ()> {
    if current_time < message.time() {
        return Err(());
    }
    if message.is_in_progress() {
        return Err(());
    }

    message.process();

    Ok(())
}

fn forward_message_to_drones(
    drones: &mut IdToDroneMap,
    signal_type: SignalType,
    message: &Message,
    current_time: Millisecond
) {
    for drone in drones.drones_mut() {
        if current_time >= message.time() {
            drone.process_message(signal_type, message);
        }
    }
}


#[derive(Clone)]
pub struct CellularAutomatonBuilder {
    command_center: Option<CommandCenter>,
    drones: Option<Vec<Drone>>,
    radar_warfare_devices: Option<Vec<CellularAutomatonRWD>>,
    destination_in_meters: Option<Point3D>,
    topology: Option<Topology>,
    scenario: Option<Scenario>
}

impl CellularAutomatonBuilder {
    pub fn new() -> Self {
        Self {
            command_center: None,
            drones: None,
            radar_warfare_devices: None,
            destination_in_meters: None,
            topology: None,
            scenario: None
        }
    }

    pub fn set_command_center(mut self, command_center: CommandCenter) -> Self {
        self.command_center = Some(command_center);
        self
    }

    pub fn set_drones(mut self, drones: &[Drone]) -> Self {
        self.drones = Some(drones.to_vec());
        self
    }

    pub fn set_radar_warfare_devices(
        mut self, 
        radar_warfare_devices: &[CellularAutomatonRWD]
    ) -> Self {
        self.radar_warfare_devices = Some(radar_warfare_devices.to_vec());
        self
    }

    pub fn set_destination(mut self, destination_in_meters: &Point3D) -> Self {
        self.destination_in_meters = Some(*destination_in_meters);
        self
    }

    pub fn set_topology(mut self, topology: Topology) -> Self {
        self.topology = Some(topology);
        self
    }

    pub fn set_scenario(mut self, scenario: Scenario) -> Self {
        self.scenario = Some(scenario);
        self
    }

    pub fn build(self) -> CellularAutomaton {
        let drones = IdToDroneMap::from(
            self.drones
                .unwrap_or_default()
                .as_slice()
        );

        CellularAutomaton::new(
            self.command_center.unwrap_or_default(),
            drones,
            self.radar_warfare_devices.unwrap_or_default(),
            self.scenario.unwrap_or_default(),
            self.topology.unwrap_or_default(),
        )
    }
}


#[derive(Clone)]
pub struct CellularAutomaton {
    current_time: Millisecond,
    command_center: CommandCenter,
    drones: IdToDroneMap,
    radar_warfare_devices: Vec<CellularAutomatonRWD>,
    destination_in_meters: Point3D,
    topology: Topology,
    connections: ConnectionGraph,
    message_queue: MessageQueue,
}

impl CellularAutomaton {
    fn new(
        command_center: CommandCenter,
        drones: IdToDroneMap,
        radar_warfare_devices: Vec<CellularAutomatonRWD>,
        scenario: Scenario,
        topology: Topology,
    ) -> Self {
        let mut cellular_automaton = Self {
            current_time: 0,
            command_center,
            drones,
            radar_warfare_devices,
            destination_in_meters: Point3D::default(),
            topology,
            connections: ConnectionGraph::new(),
            message_queue: MessageQueue::from(&scenario),
        };

        cellular_automaton.set_initial_state();

        cellular_automaton
    }

    pub fn destination(&self) -> &Point3D {
        &self.destination_in_meters
    }

    pub fn command_center(&self) -> &CommandCenter {
        &self.command_center
    }
    
    pub fn drone_iter(&self) -> Values<'_, DeviceId, Drone> { 
        self.drones.drones()
    }
    
    pub fn drone_count(&self) -> usize {
        self.drones.len() 
    }

    pub fn rwds(&self) -> Vec<RWDType> {
        self.radar_warfare_devices
            .iter()
            .map(|rwd| RWDType::CellularAutomaton(rwd))
            .collect()
    }
    
    fn set_initial_state(&mut self) {
        self.drones.connect_command_center(&self.command_center);
        self.update_connections_graph();
        self.update_signal_levels();
        self.drones.remove_uncontrolled_drones();
    }
    
    pub fn update(&mut self) {
        self.update_connections_graph();
        self.update_signal_levels();
        self.drones.remove_uncontrolled_drones();
        self.suppress_network();
        self.process_message_queue();
        self.drones.update_states();
       
        self.current_time += STEP_DURATION;
    }
    
    fn update_connections_graph(&mut self) {
        self.connections.update(
            &self.command_center, 
            &self.drones,
            self.topology
        );
    }

    fn update_signal_levels(&mut self) {
        self.drones.clear_rx_signal_levels(&SignalType::Control);

        if let Ok(control_signal_levels) = try_find_best_signal_levels(
            &self.command_center,
            &self.drones,
            &self.connections,
            &SignalType::Control,
            try_set_best_signal_level_with_cc,
            try_set_best_signal_level_with_drones
        ) {
            self.drones.set_rx_signal_levels(
                control_signal_levels, 
                &SignalType::Control
            )
        }
    }

    fn process_message_queue(&mut self) {
        if self.message_queue.is_empty() {
            return;
        }
        
        for (signal_type, message) in self.message_queue.iter_mut() {
            if try_preprocess_message(self.current_time, message).is_err() {
                continue;
            }
            
            match message.message_type() {
                MessageType::SetDestination(destination, _) =>
                    self.destination_in_meters = *destination,
                _ => ()
            }
            
            forward_message_to_drones(
                &mut self.drones, 
                *signal_type,
                message, 
                self.current_time
            );
            
            message.finish();
        }

        self.message_queue.remove_finished_messages();
    }

    fn suppress_network(&mut self) {
        for rwd in &self.radar_warfare_devices {
            for drone in self.drones.drones_mut() {
                rwd.suppress_signal_level(drone, &SignalType::Control);
                rwd.suppress_signal_level(drone, &SignalType::GPS);
            }
        }
    }
}


pub struct CellularAutomatonRWDBuilder {
    id: DeviceId,
    position: Option<Point3D>,
    tx_module: Option<TRXModule>,
    rx_module: Option<TRXModule>
}

impl CellularAutomatonRWDBuilder {
    pub fn new() -> Self {
        Self {
            id: generate_device_id(),
            position: None,
            tx_module: None,
            rx_module: None
        }
    }

    pub fn set_position(mut self, position: Point3D) -> Self {
        self.position = Some(position);
        self
    }
    
    pub fn set_tx_module(mut self, tx_module: TRXModule) -> Self {
        self.tx_module = Some(tx_module);
        self
    }
    
    pub fn set_rx_module(mut self, rx_module: TRXModule) -> Self {
        self.rx_module = Some(rx_module);
        self
    }

    pub fn set_trx_module(mut self, mut trx_system: TRXSystem) -> Self {
        self.tx_module = Some(std::mem::take(trx_system.tx_module_mut()));
        self.rx_module = Some(std::mem::take(trx_system.rx_module_mut()));
        self
    }

    pub fn build(self) -> CellularAutomatonRWD {
        let trx_system = TRXSystem::new(
            self.tx_module.unwrap_or_default(),
            self.rx_module.unwrap_or_default()
        );
        
        CellularAutomatonRWD::new(
            self.id,
            self.position.unwrap_or_default(),
            trx_system
        )
    }
}


#[derive(Clone)]
pub struct CellularAutomatonRWD {
    id: DeviceId,
    position_in_meters: Point3D,
    trx_system: TRXSystem
}

impl CellularAutomatonRWD {
    fn new(
        id: DeviceId,
        position_in_meters: Point3D,
        trx_system: TRXSystem
    ) -> Self {
        Self { 
            id,
            position_in_meters, 
            trx_system
        }
    }
}

impl Device for CellularAutomatonRWD {
    fn id(&self) -> DeviceId {
        self.id
    }
}

impl Hash for CellularAutomatonRWD {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Position for CellularAutomatonRWD {
    fn position(&self) -> &Point3D {
        &self.position_in_meters
    }
}

impl Transmitter for CellularAutomatonRWD {
    fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        self.trx_system.tx_signal_levels()
    }
    
    fn tx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.trx_system.tx_signal_level(signal_type)
    }

    fn area(&self, signal_type: &SignalType) -> SignalArea {
        self.trx_system.area(signal_type)
    }

    fn connection_distance<P: Position>(
        &self, 
        object: &P, 
        signal_type: &SignalType
    ) -> Option<Meter> {
        self.trx_system.connection_distance(
            self.distance_to(object), 
            signal_type
        )
    }
    
    fn propagated_signal_level_at<R: Receiver>(
        &self,
        receiver: &R,
        signal_type: &SignalType
    ) -> SignalLevel {
        let distance_to_rx = self.distance_to(receiver);

        self.trx_system.tx_signal_level_at(distance_to_rx, signal_type)
    }

    fn propagate_signal_level<R: Receiver>(
        &self,
        receiver: &mut R,
        signal_type: SignalType
    ) {
        let propagated_signal_level_at_rx = self.propagated_signal_level_at(
            receiver, 
            &signal_type
        );

        receiver.receive_signal(signal_type, propagated_signal_level_at_rx);
    }
}

impl Suppressor for CellularAutomatonRWD {
    fn suppress_signal_level<R: Receiver>(
            &self,
            receiver: &mut R,
            signal_type: &SignalType
    ) {
        let suppressor_signal_level_at_rx = self.propagated_signal_level_at(
            receiver,
            signal_type
        );

        let coefficient: f32 = thread_rng().gen();
        let probability = signal_level_change_probability(
            &suppressor_signal_level_at_rx
        );

        if coefficient < probability {
            receiver.signal_level_suppression(
                *signal_type, 
                suppressor_signal_level_at_rx
            );
        }
    }
}

impl Receiver for CellularAutomatonRWD {
    fn rx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        self.trx_system.rx_signal_levels()
    }

    fn rx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.trx_system.rx_signal_level(signal_type)
    }

    fn receives_signal(&self, signal_type: &SignalType) -> bool {
        self.trx_system.receives_signal_level(signal_type)
    }
    
    fn receive_signal(
        &mut self, 
        signal_type: SignalType,
        signal_level: SignalLevel
    ) {
        self.trx_system.receive_signal_level(signal_type, signal_level); 
    }

    fn receive_message(
        &mut self,
        signal_type: SignalType,
        message: &Message
    ) -> Result<(), ()> {
        self.trx_system.receive_message(signal_type, message) 
    }
    
    fn signal_level_suppression(
        &mut self,
        suppressor_signal_type: SignalType,
        suppressor_signal_level: SignalLevel
    ) {
        self.trx_system.suppress_signal_level(
            suppressor_signal_type, 
            suppressor_signal_level
        );
    }   
}

impl Transceiver for CellularAutomatonRWD {}
