use std::collections::{HashMap, hash_map::Values};

use crate::device::{
    CommandCenter, Device, DeviceId, Drone, ElectronicWarfare, STEP_DURATION, 
    Suppressor, Transceiver, Transmitter,
    networkmodel::{
        ConnectionGraph, FindSignalLevel, IdToDroneMap, IdToLevelMap, 
        MessagePreprocessError, Topology, try_preprocess_message
    }
};
use crate::mathphysics::{
    Megahertz, Meter, Millisecond, Point3D, SPEED_OF_LIGHT, kmps_to_mpms,
    time_in_millis_from_distance_and_speed
};
use crate::communication::{
    GPS_L1_FREQUENCY, Message, MessageType, NO_SIGNAL_LEVEL, Scenario, 
    WIFI_2_4GHZ_FREQUENCY
};


type DelaySnapshot = HashMap<DeviceId, u32>;


const DELAY_DISTANCE_COEFFICIENT: f32 = 1_500_000.0;


// Making this function a method of ComplexNetwork and calling it from 
// ComplexNetwork::process_message_queue() is impossible due to having mutable
// and immutable references to self simultaneously.
fn forward_message_to_drones(
    drone_map: &mut IdToDroneMap,
    frequency: Megahertz,
    delays_snapshot: &DelaySnapshot,
    message: &Message,
    current_time: Millisecond
) {
    for drone in drone_map.drones_mut() {
        if let Some(delay) = delays_snapshot.get(&drone.id()) {
            if current_time >= message.time() + delay {
                drone.process_message(frequency, message);
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

    let delay = (multiplier * time_in_millis_from_distance_and_speed(
        distance * DELAY_DISTANCE_COEFFICIENT, 
        kmps_to_mpms(SPEED_OF_LIGHT) 
    )) as Millisecond;

    let reminder = delay % STEP_DURATION;
    
    delay - reminder
}


#[derive(Clone, Default)]
struct MessageQueue(Vec<(Megahertz, Message, DelaySnapshot)>);

impl MessageQueue {
    fn remove_finished_messages(&mut self) {
        self.0.retain(|(_, message, _)| !message.is_finished());
    }

    fn iter_mut(
        &mut self
    ) -> std::slice::IterMut<'_, (Megahertz, Message, DelaySnapshot)> {
        self.0.iter_mut()
    }
   
    #[must_use]
    fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl<'a> IntoIterator for &'a mut MessageQueue{
    type Item = &'a mut (Megahertz, Message, DelaySnapshot);
    type IntoIter = std::slice::IterMut<
        'a, 
        (Megahertz, Message, DelaySnapshot)
    >;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter_mut()
    }
}

impl From<&Scenario> for MessageQueue {
    fn from(scenario: &Scenario) -> Self {
        Self(
            scenario
                .iter()
                .map(|(frequency, message)| 
                    (*frequency, *message, DelaySnapshot::default())
                )
                .collect()
        )
    }
}


#[derive(Clone)]
pub struct ComplexNetwork {
    current_time: Millisecond,
    command_center: CommandCenter,
    drone_map: IdToDroneMap,
    electronic_warfare_devices: Vec<ElectronicWarfare>,
    destination_in_meters: Point3D,
    topology: Topology,
    connections: ConnectionGraph,
    delay_multiplier: f32,
    message_queue: MessageQueue,
    // TODO add control frequency Vec
}

impl ComplexNetwork {
    #[must_use]
    pub fn new(
        command_center: CommandCenter,
        drone_map: IdToDroneMap,
        electronic_warfare_devices: Vec<ElectronicWarfare>,
        scenario: &Scenario,
        topology: Topology,
        delay_multiplier: f32
    ) -> Self {
        let mut complex_network = Self {
            current_time: 0,
            command_center,
            electronic_warfare_devices,
            drone_map,
            destination_in_meters: Point3D::default(),
            topology,
            connections: ConnectionGraph::new(),
            delay_multiplier,
            message_queue: MessageQueue::from(scenario),
        };

        complex_network.set_initial_state();

        complex_network
    }
    
    pub fn update(&mut self) {
        self.update_connections_graph();
        self.update_signal_levels();
        self.suppress_network();
        self.drone_map.remove_uncontrolled_drones(WIFI_2_4GHZ_FREQUENCY);
        self.process_message_queue();
        self.drone_map.update_states();
      
        self.current_time += STEP_DURATION;
    }

    #[must_use]
    pub fn destination(&self) -> &Point3D {
        &self.destination_in_meters
    }
    
    #[must_use]
    pub fn command_center(&self) -> &CommandCenter {
        &self.command_center
    }

    #[must_use]
    pub fn drone_map(&self) -> &IdToDroneMap {
        &self.drone_map
    }

    #[must_use]
    pub fn drone_iter(&self) -> Values<'_, DeviceId, Drone> {
        self.drone_map.drones() 
    }
    
    #[must_use]
    pub fn drone_count(&self) -> usize {
        self.drone_map.len() 
    }

    #[must_use]
    pub fn ewds(&self) -> &[ElectronicWarfare] {
        self.electronic_warfare_devices.as_slice()
    }   

    #[must_use]
    pub fn connections(&self) -> &ConnectionGraph {
        &self.connections
    }

    fn set_initial_state(&mut self) {
        self.drone_map.connect_command_center(&self.command_center);
        self.update_connections_graph();
        self.update_signal_levels();
        self.drone_map.remove_uncontrolled_drones(WIFI_2_4GHZ_FREQUENCY);
    }

    fn update_connections_graph(&mut self) {
        self.connections.update(
            &self.command_center, 
            &self.drone_map,
            self.topology,
            WIFI_2_4GHZ_FREQUENCY
        );
    }

    fn update_signal_levels(&mut self) {
        self.drone_map.clear_rx_signal_levels(WIFI_2_4GHZ_FREQUENCY);
        
        if let Ok(
            control_signal_levels
        ) = Self::try_find_best_signal_levels(
            &self.command_center,
            &self.drone_map,
            &self.connections,
            WIFI_2_4GHZ_FREQUENCY
        ) {
            self.drone_map.all_receive_signals(
                &control_signal_levels, 
                WIFI_2_4GHZ_FREQUENCY
            );
            self.drone_map.all_rx_signals_to_tx(WIFI_2_4GHZ_FREQUENCY);
        }
    }
    
    fn process_message_queue(&mut self) {
        if self.message_queue.is_empty() {
            return;
        }

        // Preprocess delays to avoid unnecessary function calls in the loop.
        let delays = self.delays();

        for (frequency, message, delays_snapshot) in &mut self.message_queue {
            match try_preprocess_message(self.current_time, message) {
                Err(MessagePreprocessError::TooEarly) => continue, 
                Err(MessagePreprocessError::AlreadyPreprocessed) => (),
                Ok(()) =>  {
                    delays_snapshot.clone_from(&delays);

                    match message.message_type() {
                        MessageType::SetDestination(destination, _) =>
                            self.destination_in_meters = *destination,
                        MessageType::ChangeGoal(_) => ()
                    }
                }
            }
            
            forward_message_to_drones(
                &mut self.drone_map,
                *frequency,
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
    #[must_use]
    fn delays(&self) -> DelaySnapshot {
        let distances = self.connections.single_source_dijkstra(
            self.command_center.id()
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
        for ewd in &self.electronic_warfare_devices {
            for drone in self.drone_map.drones_mut() {
                ewd.suppress_signal(drone, WIFI_2_4GHZ_FREQUENCY);
                ewd.suppress_signal(drone, GPS_L1_FREQUENCY);
            }
        }
    }
}

impl FindSignalLevel for ComplexNetwork {
    fn try_set_better_signal_level<T, TR>(
        tx: &T,
        rx: &TR,
        signal_levels: &mut IdToLevelMap,
        frequency: Megahertz
    ) 
    where
        T: Transmitter + Clone,
        TR: Transceiver
    {
        // TODO find a way to avoid cloning (without multiple mutable borrows)
        let mut tx = tx.clone();

        if let Some(tx_signal_level) = signal_levels.get(&tx.id()) {
            tx.set_signal_level(frequency, *tx_signal_level);
        }

        let rx_signal_level = signal_levels
            .get(&rx.id())
            .unwrap_or(&NO_SIGNAL_LEVEL);
        let signal_level_at_rx = tx.propagated_signal_level_at(rx, frequency);

        if signal_level_at_rx > *rx_signal_level {
            signal_levels.insert(rx.id(), signal_level_at_rx);
        }
    }
}


#[cfg(test)]
mod tests {
    use crate::communication::{
        GREEN_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE, RED_SIGNAL_LEVEL, 
        SignalArea, SignalLevel, YELLOW_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY
    };
    use crate::device::{
        CommandCenterBuilder, DroneBuilder,
        modules::{TRXModule, TRXSystem}
    };
    use crate::mathphysics::Meter;
    
    use super::*;

    // These constants were introduced for testing independently from the global
    // constants.
    const CC_TX_CONTROL_RADIUS: Meter    = 300.0;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const VERY_BIG_STRENGTH_VALUE: f32   = GREEN_SIGNAL_STRENGTH_VALUE * 1000.0;


    fn same_levels_of_id_to_level_maps(
        signal_levels1: &IdToLevelMap,
        signal_levels2: &IdToLevelMap
    ) -> bool {
        let same_levels = |map1: &IdToLevelMap, map2: &IdToLevelMap| -> bool {
            map1
                .iter()
                .all(|(id, signal_level1)| 
                    match map2.get(id) {
                        Some(signal_level2) => signal_level2
                            .same_level(signal_level1),
                        None => signal_level1
                            .same_level(&NO_SIGNAL_LEVEL)
                    }
                )
        };
        
        let same_from_1 = same_levels(signal_levels1, signal_levels2);
        let same_from_2 = same_levels(signal_levels2, signal_levels1);

        same_from_1 && same_from_2
    }

    fn cc_tx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let max_tx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let tx_signal_levels = HashMap::from([(
            frequency, 
            SignalLevel::from_area(
                SignalArea::build(CC_TX_CONTROL_RADIUS).unwrap(),
                frequency
            )
        )]);

        TRXModule::build(
            max_tx_signal_levels,
            tx_signal_levels
        ).unwrap()
    }

    fn drone_tx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        let max_tx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let tx_signal_levels = HashMap::from([(
            frequency, 
            SignalLevel::from_area(
                SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap(), 
                frequency
            )
        )]);

        TRXModule::build(
            max_tx_signal_levels,
            tx_signal_levels,
        ).unwrap()
    }

    fn drone_rx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        let max_rx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let rx_signal_levels = HashMap::from([
            (frequency, NO_SIGNAL_LEVEL)
        ]);

        TRXModule::build(
            max_rx_signal_levels,
            rx_signal_levels,
        ).unwrap()   
    }

    fn drone_with_trx_system_set(position: Point3D) -> Drone {
        DroneBuilder::new()
            .set_global_position(position)
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module() 
                }
            )
            .build()
    }


    #[test]
    fn better_signal_with_cc() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build();

        let mut best_signal_levels = HashMap::new();
        
        let drones = [
            drone_with_trx_system_set(Point3D::new(1.0, 0.0, 0.0)), // Green
            drone_with_trx_system_set(Point3D::new(1.5, 0.0, 0.0)), // Yellow
            drone_with_trx_system_set(Point3D::new(4.0, 0.0, 0.0)), // Red
            drone_with_trx_system_set(Point3D::new(10.0, 0.0, 0.0)) // Black
        ]; 

        for drone in &drones {
            ComplexNetwork::try_set_better_signal_level(
                &command_center,
                drone, 
                &mut best_signal_levels, 
                frequency
            );
        }

        assert!(
            best_signal_levels
                .get(&drones[0].id())
                .unwrap_or(&NO_SIGNAL_LEVEL)
                .is_green()
        );
        assert!(
            best_signal_levels
                .get(&drones[1].id())
                .unwrap_or(&NO_SIGNAL_LEVEL)
                .is_yellow()
        );
        assert!(
            best_signal_levels
                .get(&drones[2].id())
                .unwrap_or(&NO_SIGNAL_LEVEL)
                .is_red()
        );
        assert!(
            best_signal_levels
                .get(&drones[3].id())
                .unwrap_or(&NO_SIGNAL_LEVEL)
                .is_black()
        );
    }

    #[test]
    fn check_delays() {
        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: cc_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build();
        let command_center_id = command_center.id();

        let distance = 50.0;
        let drone = DroneBuilder::new()
            .set_global_position(Point3D::new(distance, 0.0, 0.0))
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: TRXModule::default(),
                    rx_module: drone_rx_module() 
                }
            )
            .build();
        let drone_id = drone.id();

        let mut network = ComplexNetwork::new(
            command_center, 
            IdToDroneMap::from([drone]), 
            Vec::new(), 
            &Scenario::default(), 
            Topology::Mesh, 
            0.0
        );

        let expected_delays = HashMap::from([
            (network.command_center.id(), 0),
            (drone_id, 0)
        ]);
        let delays = network.delays();

        assert!(expected_delays.eq(&delays));

        let multiplier = 1.0;
        network.delay_multiplier = multiplier;

        let expected_delays = HashMap::from([
            (command_center_id, 0),
            (drone_id, 250)
        ]);
        let delays = network.delays();

        assert!(expected_delays.eq(&delays));
    }

    #[test]
    fn propagate_signal_level_in_star() {
        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: cc_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build();

        // Network 1: full mesh with edge weight 25.0.
        let drones1 = [
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
        ];

        let expected_signal_levels1 = HashMap::from([
            (drones1[0].id(), GREEN_SIGNAL_LEVEL), 
            (drones1[1].id(), GREEN_SIGNAL_LEVEL), 
            (drones1[2].id(), GREEN_SIGNAL_LEVEL), 
            (drones1[3].id(), GREEN_SIGNAL_LEVEL), 
        ]);

        let network1 = ComplexNetwork::new(
            command_center.clone(), 
            IdToDroneMap::from(drones1), 
            Vec::new(), 
            &Scenario::default(), 
            Topology::Star, 
            0.0
        );

        let signal_levels1 = network1.drone_map.all_rx_signal_levels(
            WIFI_2_4GHZ_FREQUENCY
        );

        assert!(
            same_levels_of_id_to_level_maps(
                &expected_signal_levels1,
                &signal_levels1
            )
        );
        
        // Network 2 Edges:
        // 
        // A -(25.0)- B
        // A -(50.0)- C
        // A -(100.0)- D
        // A -(400.0)- E
        //
        let drones2 = [
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(50.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(100.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(400.0, 0.0, 0.0)),
        ];
        
        let expected_signal_levels2 = HashMap::from([
            (drones2[0].id(), GREEN_SIGNAL_LEVEL), 
            (drones2[1].id(), YELLOW_SIGNAL_LEVEL), 
            (drones2[2].id(), RED_SIGNAL_LEVEL), 
        ]);

        let network2 = ComplexNetwork::new(
            command_center, 
            IdToDroneMap::from(drones2), 
            Vec::new(), 
            &Scenario::default(), 
            Topology::Mesh, 
            0.0
        );

        let signal_levels2 = network2.drone_map.all_rx_signal_levels(
            WIFI_2_4GHZ_FREQUENCY
        );

        assert!(
            same_levels_of_id_to_level_maps(
                &expected_signal_levels2,
                &signal_levels2
            )
        );
    }

    #[test]
    fn propagate_signal_level_in_mesh() {
        // Network Edges:
        // 
        // A -(1.1)- B
        // A -(2.0)- C
        // A -(3.0)- D
        // A -(10.0)- E
        //
        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build();
        let drones = [
            drone_with_trx_system_set(Point3D::new(1.1, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(2.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(3.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(10.0, 0.0, 0.0)),
        ];
        
        let expected_signal_levels = HashMap::from([
            (drones[0].id(), GREEN_SIGNAL_LEVEL), 
            (drones[1].id(), YELLOW_SIGNAL_LEVEL),
            (drones[2].id(), RED_SIGNAL_LEVEL),   
            (drones[3].id(), YELLOW_SIGNAL_LEVEL), // Black in Star topology 
        ]);

        let network = ComplexNetwork::new(
            command_center, 
            IdToDroneMap::from(drones), 
            Vec::new(), 
            &Scenario::default(), 
            Topology::Mesh, 
            0.0
        );

        let signal_levels = network.drone_map.all_rx_signal_levels(
            WIFI_2_4GHZ_FREQUENCY
        );

        assert!(
            same_levels_of_id_to_level_maps(
                &expected_signal_levels,
                &signal_levels
            )
        );
    }
}
