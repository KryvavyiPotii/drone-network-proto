use std::collections::hash_map::Values;

use crate::device::{
    CommandCenter, ConnectionGraph, DelaySnapshot, Device, DeviceId, Drone, 
    ElectronicWarfare, FindSignalLevel, IdToDroneMap, IdToLevelMap, Topology, 
    Suppressor, Transceiver, Transmitter, STEP_DURATION, UNKNOWN_ID
};
use crate::device::networkmodel::{
    enqueue_infection_messages, spread_infection_messages
};
use crate::mathphysics::{Megahertz, Millisecond, Point3D};
use crate::message::{
    Goal, Message, MessagePreprocessError, MessageQueue, MessageType
};
use crate::signal::{
    GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, NO_SIGNAL_LEVEL, 
    WIFI_2_4GHZ_FREQUENCY
};


fn send_message(
    drone_map: &mut IdToDroneMap,
    frequency: Megahertz,
    delays_snapshot: &DelaySnapshot,
    message: &Message,
    current_time: Millisecond
) {
    let destination_id = message.destination_id();

    if destination_id == UNKNOWN_ID {
        broadcast_message(
            drone_map,
            frequency,
            delays_snapshot,
            message,
            current_time
        );
    } else {
        let Some(drone) = drone_map.get_mut(&destination_id) else {
            return;
        };
        let delay = delays_snapshot
            .get(&drone.id())
            .unwrap_or(&0);

        unicast_message(drone, frequency, *delay, message, current_time);
    }
}

fn unicast_message(
    drone: &mut Drone,
    frequency: Megahertz,
    delay: Millisecond,
    message: &Message,
    current_time: Millisecond
) {
    if current_time >= message.time() + delay {
        let _ = drone.receive_and_process_message(frequency, message);
    }
}

fn broadcast_message(
    drone_map: &mut IdToDroneMap,
    frequency: Megahertz,
    delays_snapshot: &DelaySnapshot,
    message: &Message,
    current_time: Millisecond
) {
    for drone in drone_map.drones_mut() {
        let delay = delays_snapshot
            .get(&drone.id())
            .unwrap_or(&0);

        unicast_message(drone, frequency, *delay, message, current_time);
    }
}

// We assume that the message processing is finished if it was processed by 
// the drone with the longest delay.
fn try_finish_message(
    current_time: Millisecond,
    delays_snapshot: &DelaySnapshot,
    message: &mut Message
) {
    if let MessageType::Infection(_) = message.message_type() {
        message.finish();
        return;
    }

    if let Some(longest_delay) = delays_snapshot.values().max() {
        if current_time >= message.time() + longest_delay {
            message.finish(); 
        }
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
        scenario: &[(Megahertz, Message)],
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
        
        self.drone_map.handle_infection();
        self.drone_map.remove_uncontrolled_drones(WIFI_2_4GHZ_FREQUENCY);
        
        self.drone_map.update_states();
        self.drone_map.set_rx_signal_level(
            GPS_L1_FREQUENCY, 
            &GREEN_SIGNAL_LEVEL
        );
     
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
        let delays_from_cc = self.connections.delays(
            self.command_center.id(), 
            self.delay_multiplier
        );

        let mut infection_messages = Vec::new();

        for (frequency, message, delays_snapshot) in &mut self.message_queue {
            match message.try_preprocess(self.current_time) {
                Err(MessagePreprocessError::TooEarly) => continue, 
                Err(MessagePreprocessError::AlreadyPreprocessed) => (),
                Ok(()) => {
                    match message.message_type() {
                        MessageType::SetGoal(
                            Goal::Attack(destination)
                            | Goal::Reposition(destination)
                        ) => {
                            self.destination_in_meters = *destination;
                            delays_snapshot.clone_from(&delays_from_cc);
                        },
                        MessageType::SetGoal(_) => 
                            delays_snapshot.clone_from(&delays_from_cc),
                        MessageType::Infection(_) => { 
                            delays_snapshot.clone_from(
                                &self.connections.delays(
                                    message.destination_id(), 
                                    self.delay_multiplier
                                )
                            );

                            spread_infection_messages(
                                *frequency,
                                message,
                                &self.connections,
                                &mut infection_messages
                            )
                        },
                    }
                }
            }
            
            send_message(
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

        enqueue_infection_messages(
            &mut self.message_queue,
            &self.drone_map,
            &infection_messages
        );
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
    use std::collections::HashMap;

    use crate::device::{CommandCenterBuilder, Device, DroneBuilder};
    use crate::device::systems::{TRXModule, TRXSystem};
    use crate::infection::{InfectionType, INFECTION_DELAY};
    use crate::mathphysics::Meter;
    use crate::signal::{
        GREEN_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE, RED_SIGNAL_LEVEL, 
        SignalArea, SignalLevel, YELLOW_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY
    };
    
    use super::*;


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

    fn wait_for_infection(network: &mut ComplexNetwork) {
        for _ in (0..=INFECTION_DELAY).step_by(STEP_DURATION as usize) {
            network.update();
        }
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
            &Vec::new(), 
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
            &Vec::new(), 
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
            &Vec::new(), 
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

    #[test]
    fn spread_infection_in_star() {
        // Network topology:
        //
        // B -(1.1)- A -(1.1)- C
        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build();
        
        let vulnerable_drone_builder = DroneBuilder::new()
            .set_vulnerabilities(&[InfectionType::Indicator])
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module() 
                }
            );
        let infected_drone = vulnerable_drone_builder
            .clone()
            .set_global_position(Point3D::new(1.1, 0.0, 0.0))
            .build();
        let infected_drone_id = infected_drone.id();
        let vulnerable_drone = vulnerable_drone_builder
            .set_global_position(Point3D::new(-1.1, 0.0, 0.0))
            .build();
        let vulnerable_drone_id = vulnerable_drone.id();

        let drones = [infected_drone, vulnerable_drone];
        let scenario = vec!((
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center.id(),
                infected_drone_id,
                0, 
                MessageType::Infection(InfectionType::Indicator)
            ),
        ));
        
        let mut network = ComplexNetwork::new(
            command_center, 
            IdToDroneMap::from(drones), 
            Vec::new(), 
            &scenario, 
            Topology::Star, 
            0.0
        );

        assert!(
            !network.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.drone_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_center.is_infected());

        network.update();
        
        assert!(
            network.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.drone_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_center.is_infected());

        wait_for_infection(&mut network);

        assert!(
            network.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.drone_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        // At version 0.6.0 must panic.
        // Command center infection should be implemented in next versions.
        assert!(network.command_center.is_infected());

        wait_for_infection(&mut network);

        assert!(
            network.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.drone_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(network.command_center.is_infected());
    }

    #[test]
    fn spread_infection_in_mesh() {
        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build();
        
        let vulnerable_drone_builder = DroneBuilder::new()
            .set_vulnerabilities(&[InfectionType::Indicator])
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module() 
                }
            );
        let infected_drone = vulnerable_drone_builder
            .clone()
            .set_global_position(Point3D::new(10.0, 0.0, 0.0))
            .build();
        let infected_drone_id = infected_drone.id();
        let vulnerable_drone1 = vulnerable_drone_builder
            .clone()
            .set_global_position(Point3D::new(1.1, 0.0, 0.0))
            .build();
        let vulnerable_drone_id1 = vulnerable_drone1.id();
        let vulnerable_drone2 = vulnerable_drone_builder
            .clone()
            .set_global_position(Point3D::new(2.0, 0.0, 0.0))
            .build();
        let vulnerable_drone_id2 = vulnerable_drone2.id();
        let vulnerable_drone3 = vulnerable_drone_builder
            .clone()
            .set_global_position(Point3D::new(3.0, 0.0, 0.0))
            .build();
        let vulnerable_drone_id3 = vulnerable_drone3.id();
        
        let drones = [
            infected_drone, 
            vulnerable_drone1,
            vulnerable_drone2,
            vulnerable_drone3,
        ];
        let scenario = vec!((
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center.id(),
                infected_drone_id,
                0, 
                MessageType::Infection(InfectionType::Indicator)
            ),
        ));
        
        let mut network = ComplexNetwork::new(
            command_center, 
            IdToDroneMap::from(drones), 
            Vec::new(), 
            &scenario, 
            Topology::Mesh, 
            0.0
        );

        assert!(
            !network.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.drone_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.drone_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.drone_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_center.is_infected());

        network.update();       
        
        assert!(
            network.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.drone_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.drone_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.drone_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_center.is_infected());

        wait_for_infection(&mut network);

        assert!(
            network.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.drone_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.drone_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.drone_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_center.is_infected());

        wait_for_infection(&mut network);
        
        assert!(
            network.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.drone_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.drone_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.drone_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        // At version 0.6.0 must panic.
        // Command center infection should be implemented in next versions.
        assert!(network.command_center.is_infected());
    }
}
