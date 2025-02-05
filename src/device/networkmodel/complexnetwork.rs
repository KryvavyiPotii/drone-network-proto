use std::collections::{hash_map::Values, HashMap};

use crate::device::*;

use super::*;


type DelaySnapshot = HashMap<DeviceId, u32>;


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
    let signal_level = tx.propagated_signal_level_at(rx, signal_type);

    if signal_level > *current_signal_level {
        signal_levels.insert(rx.id(), signal_level);
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

// Making this function a method of ComplexNetwork and calling it from 
// ComplexNetwork::process_message_queue() is impossible due to having mutable
// and immutable references to self simultaneously.
fn forward_message_to_drones(
    drones: &mut IdToDroneMap,
    signal_type: SignalType,
    delays_snapshot: &DelaySnapshot,
    message: &Message,
    current_time: Millisecond
) {
    for drone in drones.drones_mut() {
        if let Some(delay) = delays_snapshot.get(&drone.id) {
            if current_time >= message.time() + delay {
                drone.process_message(signal_type, message);
            }
        }
    }
}

// We assume that the message processing is finished if it was processed by 
// the drone with the longest delay.
fn try_finish_message(
    current_time: Millisecond,
    delays_snapshot: &DelaySnapshot,
    message: &mut Message
) {
    if let Some(longest_delay) = delays_snapshot.values().max() {
        if current_time >= message.time() + longest_delay {
            message.finish(); 
        }
    }
}

fn calculate_delay(distance: Meter, multiplier: f32) -> Millisecond {    
    if multiplier == 0.0 {
        return 0;
    }

    let delay = time_in_millis_from_distance_and_speed(
        distance, 
        mps_to_mpms(SIGNAL_SPEED) 
    ) as f32;
    let multiplied_delay = (delay * multiplier) as Millisecond;

    let reminder = multiplied_delay % STEP_DURATION;
    
    multiplied_delay - reminder
}


#[derive(Clone, Default)]
struct MessageQueueWithDelays(Vec<(SignalType, Message, DelaySnapshot)>);

impl MessageQueueWithDelays {
    fn remove_finished_messages(&mut self) {
        self.0.retain(|(_, message, _)| !message.is_finished());
    }

    fn iter_mut(
        &mut self
    ) -> std::slice::IterMut<'_, (SignalType, Message, DelaySnapshot)> {
        self.0.iter_mut()
    }
    
    fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl From<&Scenario> for MessageQueueWithDelays {
    fn from(scenario: &Scenario) -> Self {
        Self(
            scenario
                .iter()
                .map(|(signal_type, message)| 
                    (*signal_type, *message, DelaySnapshot::default())
                )
                .collect()
        )
    }
}


#[derive(Clone)]
pub struct ComplexNetworkBuilder {
    command_center: Option<CommandCenter>,
    drones: Option<Vec<Drone>>,
    radar_warfare_devices: Option<Vec<ComplexNetworkRWD>>,
    destination_in_meters: Option<Point3D>,
    topology: Option<Topology>,
    delay_multiplier: f32,
    scenario: Option<Scenario>
}

impl ComplexNetworkBuilder {
    pub fn new() -> Self {
        Self {
            command_center: None,
            drones: None,
            radar_warfare_devices: None,
            destination_in_meters: None,
            topology: None,
            delay_multiplier: 0.0,
            scenario: None
        }
    }

    // TODO allow usage of the same command center for different networks.
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
        radar_warfare_devices: &[ComplexNetworkRWD]
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

    pub fn set_delay_multiplier(mut self, delay_multiplier: f32) -> Self {
        self.delay_multiplier = delay_multiplier;
        self
    }

    pub fn set_scenario(mut self, scenario: Scenario) -> Self {
        self.scenario = Some(scenario);
        self
    }

    pub fn build(self) -> ComplexNetwork {
        let drones = IdToDroneMap::from(
            self.drones
                .unwrap_or_default()
                .as_slice()
        );

        ComplexNetwork::new(
            self.command_center.unwrap_or_default(),
            drones,
            self.radar_warfare_devices.unwrap_or_default(),
            self.scenario.unwrap_or_default(),
            self.topology.unwrap_or_default(),
            self.delay_multiplier
        )
    }
}


#[derive(Clone)]
pub struct ComplexNetwork {
    current_time: Millisecond,
    command_center: CommandCenter,
    drones: IdToDroneMap,
    radar_warfare_devices: Vec<ComplexNetworkRWD>,
    destination_in_meters: Point3D,
    topology: Topology,
    connections: ConnectionGraph,
    delay_multiplier: f32,
    message_queue: MessageQueueWithDelays,
}

impl ComplexNetwork {
    fn new(
        command_center: CommandCenter,
        drones: IdToDroneMap,
        radar_warfare_devices: Vec<ComplexNetworkRWD>,
        scenario: Scenario,
        topology: Topology,
        delay_multiplier: f32
    ) -> Self {
        let mut complex_network = Self {
            current_time: 0,
            command_center,
            radar_warfare_devices,
            drones,
            destination_in_meters: Point3D::default(),
            topology,
            connections: ConnectionGraph::new(),
            delay_multiplier,
            message_queue: MessageQueueWithDelays::from(&scenario),
        };

        complex_network.set_initial_state();

        complex_network
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

    // DBG
    pub fn drones(&self) -> &IdToDroneMap {
        &self.drones
    }

    pub fn rwds(&self) -> Vec<RWDType> {
        self.radar_warfare_devices
            .iter()
            .map(|rwd| RWDType::ComplexNetwork(rwd))
            .collect()
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

    fn set_initial_state(&mut self) {
        self.drones.connect_command_center(&self.command_center);
        self.update_connections_graph();
        self.update_signal_levels();
        self.drones.remove_uncontrolled_drones();
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

        // Preprocess delays to avoid unnecessary function calls in the loop.
        let delays = self.delays();

        for (signal_type, message, delays_snapshot) in self.message_queue
            .iter_mut()
        {
            *delays_snapshot = delays.clone();
            
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
                delays_snapshot,
                message,
                self.current_time
            );

            try_finish_message(
                self.current_time,
                delays_snapshot,
                message
            );
        }

        self.message_queue.remove_finished_messages();
    }

    // Connections graph should already be created and populated before delay
    // calculation.
    fn delays(&self) -> DelaySnapshot {
        let distances = self.connections.single_source_dijkstra(
            self.command_center.id
        ).expect("Failed to find the shortest paths");

        distances
            .iter()
            .map(|(device_id, distance)| {
                let delay = calculate_delay(*distance, self.delay_multiplier);
            
                (*device_id, delay)
            })
            .collect()
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


pub struct ComplexNetworkRWDBuilder {
    id: DeviceId,
    position: Option<Point3D>,
    tx_module: Option<TRXModule>,
    rx_module: Option<TRXModule>
}

impl ComplexNetworkRWDBuilder {
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

    pub fn build(self) -> ComplexNetworkRWD {
        let trx_system = TRXSystem::new(
            self.tx_module.unwrap_or_default(),
            self.rx_module.unwrap_or_default()
        );
        
        ComplexNetworkRWD::new(
            self.id,
            self.position.unwrap_or_default(),
            trx_system
        )
    }
}


#[derive(Clone)]
pub struct ComplexNetworkRWD {
    id: DeviceId,
    position_in_meters: Point3D,
    trx_system: TRXSystem
}

impl ComplexNetworkRWD {
    pub fn new(
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

impl Device for ComplexNetworkRWD {
    fn id(&self) -> DeviceId {
        self.id
    }
}

impl Hash for ComplexNetworkRWD {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Position for ComplexNetworkRWD {
    fn position(&self) -> &Point3D {
        &self.position_in_meters
    }
}

impl Transmitter for ComplexNetworkRWD {
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

impl Suppressor for ComplexNetworkRWD {
    fn suppress_signal_level<R: Receiver>(
            &self,
            receiver: &mut R,
            signal_type: &SignalType
    ) {
        let suppressor_signal_level_at_rx = self.propagated_signal_level_at(
            receiver,
            signal_type
        );

        receiver.signal_level_suppression(
            *signal_type, 
            suppressor_signal_level_at_rx
        );
    }
}

impl Receiver for ComplexNetworkRWD {
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

impl Transceiver for ComplexNetworkRWD {}


#[cfg(test)]
mod tests {
    use super::*;

    const CC_TX_CONTROL_RADIUS: f32    = 300.0;
    const DRONE_TX_CONTROL_RADIUS: f32 = 10.0;
    const RWD_TX_CONTROL_RADIUS: f32   = 25.0;
    const RWD_TX_GPS_RADIUS: f32       = 50.0;

    fn rwd_signal_levels() -> HashMap<SignalType, SignalLevel> {
        HashMap::from([
            (
                SignalType::Control, 
                SignalLevel::from(
                    SignalArea::build(RWD_TX_CONTROL_RADIUS).unwrap()
                )
            ),
            (
                SignalType::GPS, 
                SignalLevel::from(
                    SignalArea::build(RWD_TX_GPS_RADIUS).unwrap()
                )
            )
        ])
    }

    fn drone_with_trx_system_set(position: Point3D) -> Drone {
        DroneBuilder::new()
            .set_global_position(position)
            .set_tx_module(
                TRXModule::build(
                    AntennaType::Strength,
                    HashMap::from([(SignalType::Control, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(
                        SignalType::Control, 
                        SignalLevel::from(
                            SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap()
                        )
                    )]),
                ).unwrap()
            )
            .set_rx_module(
                TRXModule::build(
                    AntennaType::Strength,
                    HashMap::from([(SignalType::Control, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(SignalType::Control, NO_SIGNAL_LEVEL)]),
                ).unwrap()
            )
            .build()
    }

    #[test]
    fn better_signal_with_cc() {
        let signal_type = SignalType::Control;

        let command_center = CommandCenterBuilder::new()
            .set_tx_module(
                TRXModule::build(
                    AntennaType::Strength,
                    HashMap::from([(signal_type, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(
                        SignalType::Control, 
                        SignalLevel::from(
                            SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap()
                        )
                    )])
                ).unwrap()
            )
            .build();

        let green_drone = drone_with_trx_system_set(
            Point3D::new(5.0, 0.0, 0.0)
        );    
        let yellow_drone = drone_with_trx_system_set(
            Point3D::new(8.0, 0.0, 0.0)
        );
        let red_drone = drone_with_trx_system_set(
            Point3D::new(14.9, 0.0, 0.0)
        );
        let black_drone = drone_with_trx_system_set(
            Point3D::new(25.0, 0.0, 0.0)
        );

        let mut best_signal_levels = HashMap::new();
        let drones = vec![green_drone, yellow_drone, red_drone, black_drone];
        
        for drone in &drones {
            try_set_best_signal_level_with_cc(
                &command_center, 
                drone, 
                &mut best_signal_levels, 
                &signal_type
            );
        }

        assert!(
            command_center.tx_signal_level(&signal_type) 
            >= drones[0].rx_signal_level(&signal_type)
        ); 
        assert!(
            drones[0].rx_signal_level(&signal_type) 
            >= drones[1].rx_signal_level(&signal_type)
        ); 
        assert!(
            drones[1].rx_signal_level(&signal_type) 
            >= drones[2].rx_signal_level(&signal_type)
        ); 
        assert!(
            drones[2].rx_signal_level(&signal_type) 
            >= drones[3].rx_signal_level(&signal_type)
        ); 
    }

    #[test]
    fn suppress_tranceivers() {
        let signal_type = SignalType::Control;
        let trx_module = TRXModule::build(
            AntennaType::Strength,
            rwd_signal_levels(),
            rwd_signal_levels()
        ).unwrap();

        let rwd = ComplexNetworkRWDBuilder::new()
            .set_tx_module(trx_module.clone())
            .build();
        let mut drone_inside = DroneBuilder::new()
            .set_rx_module(trx_module.clone())
            .build();
        let mut drone_outside = DroneBuilder::new()
            .set_global_position(
                Point3D::new(
                    RWD_TX_CONTROL_RADIUS * 20.0,
                    0.0,
                    0.0
                )
            )
            .set_rx_module(trx_module)
            .build();

        rwd.suppress_signal_level(&mut drone_inside, &signal_type);
        rwd.suppress_signal_level(&mut drone_outside, &signal_type);

        assert!(
            *drone_inside
                .rx_signal_level(&signal_type) < BLACK_SIGNAL_LEVEL
        );
        assert!(
            *drone_outside
                .rx_signal_level(&signal_type) >= BLACK_SIGNAL_LEVEL
        );
    }

    #[test]
    fn check_delays() {
        let command_center = CommandCenterBuilder::new()
            .set_tx_module(
                TRXModule::build(
                    AntennaType::Strength,
                    HashMap::from([(SignalType::Control, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(
                        SignalType::Control, 
                        SignalLevel::from(
                            SignalArea::build(CC_TX_CONTROL_RADIUS).unwrap()
                        )
                    )])
                ).unwrap()
            )
            .build();

        let distance = 5.0;
        let drone = DroneBuilder::new()
            .set_global_position(Point3D::new(distance, 0.0, 0.0))
            .set_rx_module(
                TRXModule::build(
                    AntennaType::Strength,
                    HashMap::from([(SignalType::Control, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(SignalType::Control, NO_SIGNAL_LEVEL)]),
                ).unwrap()
            )
            .build();
        let drone_id = drone.id();

        let mut network = ComplexNetworkBuilder::new()
            .set_command_center(command_center)
            .set_drones(&[drone])
            .set_topology(Topology::Mesh)
            .build();

        let expected_delays = HashMap::from([
            (network.command_center.id, 0),
            (drone_id, 0)
        ]);
        let delays = network.delays();

        assert!(expected_delays.eq(&delays));

        let multiplier = 1.0;
        network.delay_multiplier = multiplier;
        network.update();

        let expected_delays = HashMap::from([
            (network.command_center.id, 0),
            (drone_id, calculate_delay(distance, multiplier))
        ]);
        let delays = network.delays();

        assert!(expected_delays.eq(&delays));
    }

    #[test]
    fn propagate_signal_level_by_strength() {
        let command_center = CommandCenterBuilder::new()
            .set_tx_module(
                TRXModule::build(
                    AntennaType::Strength,
                    HashMap::from([(SignalType::Control, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(
                        SignalType::Control, 
                        SignalLevel::from(
                            SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap()
                        )
                    )])
                ).unwrap()
            )
            .build();

        // Network 1: full mesh with edge weight 1.0.
        let drones1 = [
            drone_with_trx_system_set(Point3D::new(1.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(2.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(3.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(4.0, 0.0, 0.0)),
        ];
        let ids1: Vec<DeviceId> = drones1
            .iter()
            .map(|drone| drone.id())
            .collect();

        let network1 = ComplexNetworkBuilder::new()
            .set_command_center(command_center.clone())
            .set_drones(&drones1)
            .set_topology(Topology::Star)
            .build();

        let expected_signal_levels1 = HashMap::from([
            (ids1[0], RED_SIGNAL_LEVEL), 
            (ids1[1], RED_SIGNAL_LEVEL), 
            (ids1[2], RED_SIGNAL_LEVEL), 
            (ids1[3], RED_SIGNAL_LEVEL), 
        ]);
        let signal_levels1 = network1.drones.all_rx_signal_levels(
            &SignalType::Control
        );

        assert!(
            same_levels_of_id_hashmaps(
                &expected_signal_levels1,
                &signal_levels1
            )
        );
        
        // Network 2:
        // 
        // A -(7.0)- B -(7.0)- C -(7.0)- D -(7.0)- E 
        //
        let drones2 = [
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(21.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(28.0, 0.0, 0.0)),
        ];
        let ids2: Vec<DeviceId> = drones2
            .iter()
            .map(|drone| drone.id)
            .collect();

        let network2 = ComplexNetworkBuilder::new()
            .set_command_center(command_center)
            .set_drones(&drones2)
            .set_topology(Topology::Mesh)
            .build();

        let expected_signal_levels2 = HashMap::from([
            (ids2[0], RED_SIGNAL_LEVEL), 
            (ids2[1], RED_SIGNAL_LEVEL), 
        ]);
        let signal_levels2 = network2.drones.all_rx_signal_levels(
            &SignalType::Control
        );

        assert!(
            same_levels_of_id_hashmaps(
                &expected_signal_levels2,
                &signal_levels2
            )
        );
    }
}
