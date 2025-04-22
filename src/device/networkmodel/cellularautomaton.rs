use std::collections::hash_map::Values;

use crate::device::{
    CommandCenter, ConnectionGraph, DeviceId, Drone, 
    ElectronicWarfare, FindSignalLevel, IdToDroneMap, IdToLevelMap, Receiver, 
    STEP_DURATION, Suppressor, Topology, Transceiver, Transmitter, UNKNOWN_ID
};
use crate::device::networkmodel::{
    enqueue_infection_messages, spread_infection_messages,
};
use crate::mathphysics::{Megahertz, Millisecond, Point3D};
use crate::message::{
    Goal, Message, MessagePreprocessError, MessageQueue, MessageType
};
use crate::signal::{
    GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, NO_SIGNAL_LEVEL, 
    SignalLevel, WIFI_2_4GHZ_FREQUENCY
};


const CHANGE_SIGNAL_LEVEL_FROM_GREEN_PROBABILITY: f64  = 0.95;
const CHANGE_SIGNAL_LEVEL_FROM_YELLOW_PROBABILITY: f64 = 0.70;
const CHANGE_SIGNAL_LEVEL_FROM_RED_PROBABILITY: f64    = 0.50;
const CHANGE_SIGNAL_LEVEL_FROM_BLACK_PROBABILITY: f64  = 0.0;


fn send_message_and_handle_infection(
    drone_map: &mut IdToDroneMap,
    frequency: Megahertz,
    message: &Message,
) {
    let destination_id = message.destination_id();

    if destination_id == UNKNOWN_ID {
        broadcast_message_and_handle_infection(
            drone_map,
            frequency,
            message,
        );
    } else {
        let Some(drone) = drone_map.get_mut(&destination_id) else {
            return;
        };

        unicast_message_and_handle_infection(
            drone, 
            frequency, 
            message, 
        );
    }
}

fn unicast_message_and_handle_infection(
    drone: &mut Drone,
    frequency: Megahertz,
    message: &Message,
) {
    if drone.receive_and_process_message(frequency, message).is_ok() {
        drone.handle_infection();
    }
}

fn broadcast_message_and_handle_infection(
    drone_map: &mut IdToDroneMap,
    frequency: Megahertz,
    message: &Message,
) {
    for drone in drone_map.drones_mut() {
        unicast_message_and_handle_infection(
            drone, 
            frequency, 
            message, 
        );
    }
}

fn suppress_with_probability<S, R>(
    tx: &S,
    rx: &mut R,
    frequency: Megahertz
) 
where
    S: Suppressor,
    R: Receiver
{
    let suppressor_signal_level_at_rx = tx.propagated_signal_level_at(
        rx,
        frequency
    );

    if signal_level_change_happens(suppressor_signal_level_at_rx) {
        rx.signal_level_suppression(
            frequency, 
            suppressor_signal_level_at_rx
        );
    }
}

// TODO optimize to avoid double rx.signal_level: in propagated_signal_level_at
// and in rx.signal_level itself
fn effective_propagated_signal<T, R>(
    tx: &T,
    rx: &R,
    frequency: Megahertz
) -> SignalLevel 
where
    T: Transmitter,
    R: Receiver
{
    let tx_signal_level_at_rx = tx.propagated_signal_level_at(rx, frequency);

    if signal_level_change_happens(tx_signal_level_at_rx) {
        tx_signal_level_at_rx
    } else {
        let current_signal_level = *rx.signal_level(frequency);
        
        current_signal_level
    }
}

fn signal_level_change_happens(signal_level: SignalLevel) -> bool {
    let probability = signal_level_change_probability(signal_level);
   
    rand::random_bool(probability)
}

fn signal_level_change_probability(signal_level: SignalLevel) -> f64 {
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


#[derive(Clone)]
pub struct CellularAutomaton {
    current_time: Millisecond,
    command_center: CommandCenter,
    drone_map: IdToDroneMap,
    electronic_warfare_devices: Vec<ElectronicWarfare>,
    destination_in_meters: Point3D,
    topology: Topology,
    connections: ConnectionGraph,
    message_queue: MessageQueue,
}

impl CellularAutomaton {
    #[must_use]
    pub fn new(
        command_center: CommandCenter,
        drone_map: IdToDroneMap,
        electronic_warfare_devices: Vec<ElectronicWarfare>,
        scenario: &[(Megahertz, Message)],
        topology: Topology,
    ) -> Self {
        let mut cellular_automaton = Self {
            current_time: 0,
            command_center,
            drone_map,
            electronic_warfare_devices,
            destination_in_meters: Point3D::default(),
            topology,
            connections: ConnectionGraph::new(),
            message_queue: MessageQueue::from(scenario),
        };

        cellular_automaton.set_initial_state();

        cellular_automaton
    }
    
    pub fn update(&mut self) {
        self.update_connections_graph();
        self.update_signal_levels();
        self.suppress_network();
        self.drone_map.remove_uncontrolled_drones(WIFI_2_4GHZ_FREQUENCY);
        self.process_message_queue();
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
        if let Ok(
            control_signal_levels
        ) = Self::try_find_best_signal_levels(
            &self.command_center,
            &self.drone_map,
            &self.connections,
            WIFI_2_4GHZ_FREQUENCY
        ) {
            self.drone_map.set_rx_signal_levels(
                &control_signal_levels, 
                WIFI_2_4GHZ_FREQUENCY
            );
        }
    }

    fn process_message_queue(&mut self) {
        if self.message_queue.is_empty() {
            return;
        }

        let mut infection_messages = Vec::new();
        
        for (frequency, message, _) in &mut self.message_queue {
            match message.try_preprocess(self.current_time) {
                Err(MessagePreprocessError::TooEarly) => continue, 
                Err(MessagePreprocessError::AlreadyPreprocessed) => (),
                Ok(()) =>
                    match message.message_type() {
                        MessageType::SetGoal(
                            Goal::Attack(destination)
                            | Goal::Reposition(destination)
                        ) => self.destination_in_meters = *destination,
                        MessageType::SetGoal(_) => (),
                        MessageType::Infection(_) => 
                            spread_infection_messages(
                                *frequency,
                                message,
                                &self.connections,
                                &mut infection_messages
                            ),
                    }
            }
            
            send_message_and_handle_infection(
                &mut self.drone_map, 
                *frequency,
                message,
            );

            message.finish();
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
                suppress_with_probability(ewd, drone, WIFI_2_4GHZ_FREQUENCY);
                suppress_with_probability(ewd, drone, GPS_L1_FREQUENCY);
            }
        }
    }
}

impl FindSignalLevel for CellularAutomaton {
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
        let rx_signal_level = signal_levels
            .get(&rx.id())
            .unwrap_or(&NO_SIGNAL_LEVEL);
        let signal_level_at_rx = effective_propagated_signal(tx, rx, frequency);

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
        GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE,
        SignalArea
    };
    
    use super::*;


    const SOME_FREQUENCY: Megahertz      = 5_000;
    const DRONE_TX_CONTROL_RADIUS: Meter = 30.0;
    const VERY_BIG_STRENGTH_VALUE: f32  = GREEN_SIGNAL_STRENGTH_VALUE * 1_000.0;


    fn drone_tx_area() -> SignalArea {
        SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap()
    }
    
    fn wait_for_infection(automaton: &mut CellularAutomaton) {
        for _ in (0..=INFECTION_DELAY).step_by(STEP_DURATION as usize) {
            automaton.update();
        }
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


    #[test]
    fn better_signal_with_cc() {
        let frequency = SOME_FREQUENCY;

        let max_cc_signal_level = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let cc_signal_level = HashMap::from([(
            frequency, 
            SignalLevel::from_area(drone_tx_area(), frequency)
        )]);
        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Color(
                    TRXModule::build(max_cc_signal_level, cc_signal_level)
                        .unwrap()
                )
            )
            .build();

        let trx_module = TRXModule::build(
            HashMap::from([
                (
                    frequency, 
                    SignalLevel::from_area(drone_tx_area(), frequency)
                ),
                (GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL)
            ]),
            HashMap::new()
        ).unwrap();

        let drone_trx_system = TRXSystem::Color(trx_module);

        let drones = [
            DroneBuilder::new()
                .set_global_position(Point3D::new(10.0, 0.0, 0.0))
                .set_trx_system(drone_trx_system.clone())
                .build(),
            DroneBuilder::new()
                .set_global_position(Point3D::new(20.0, 0.0, 0.0))
                .set_trx_system(drone_trx_system.clone())
                .build(),
            DroneBuilder::new()
                .set_global_position(Point3D::new(25.0, 0.0, 0.0))
                .set_trx_system(drone_trx_system.clone())
                .build(),
            DroneBuilder::new()
                .set_global_position(Point3D::new(35.0, 0.0, 0.0))
                .set_trx_system(drone_trx_system)
                .build(),
        ]; 

        let mut best_signal_levels = HashMap::new();

        for drone in &drones {
            CellularAutomaton::try_set_better_signal_level(
                &command_center,
                drone, 
                &mut best_signal_levels, 
                frequency
            );
        }

        assert!(
            {
                let tx_signal_level = best_signal_levels
                    .get(&drones[0].id())
                    .unwrap_or(&NO_SIGNAL_LEVEL);

                tx_signal_level.is_green() || tx_signal_level.is_black()
            }
        ); 
        assert!(
            {
                let tx_signal_level = best_signal_levels
                    .get(&drones[1].id())
                    .unwrap_or(&NO_SIGNAL_LEVEL);

                tx_signal_level.is_yellow() || tx_signal_level.is_black()
            }
        ); 
        assert!(
            {
                let tx_signal_level = best_signal_levels
                    .get(&drones[2].id())
                    .unwrap_or(&NO_SIGNAL_LEVEL);

                tx_signal_level.is_red() || tx_signal_level.is_black()
            }
        ); 
        assert!(
            {
                let tx_signal_level = best_signal_levels
                    .get(&drones[3].id())
                    .unwrap_or(&NO_SIGNAL_LEVEL);

                tx_signal_level.is_black()
            }
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
        
        let mut automaton = CellularAutomaton::new(
            command_center, 
            IdToDroneMap::from(drones), 
            Vec::new(), 
            &scenario, 
            Topology::Star, 
        );

        assert!(
            !automaton.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.drone_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_center.is_infected());

        automaton.update();
        
        assert!(
            automaton.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.drone_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_center.is_infected());

        wait_for_infection(&mut automaton);

        assert!(
            automaton.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.drone_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        // At version 0.6.0 must panic.
        // Command center infection should be implemented in next versions.
        assert!(automaton.command_center.is_infected());

        wait_for_infection(&mut automaton);

        assert!(
            automaton.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.drone_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(automaton.command_center.is_infected());
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
        
        let mut automaton = CellularAutomaton::new(
            command_center, 
            IdToDroneMap::from(drones), 
            Vec::new(), 
            &scenario, 
            Topology::Mesh, 
        );

        assert!(
            !automaton.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.drone_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.drone_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.drone_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_center.is_infected());

        automaton.update();       
        
        assert!(
            automaton.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.drone_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.drone_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.drone_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_center.is_infected());

        wait_for_infection(&mut automaton);

        assert!(
            automaton.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.drone_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.drone_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.drone_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_center.is_infected());

        wait_for_infection(&mut automaton);
        
        assert!(
            automaton.drone_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.drone_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.drone_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.drone_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        // At version 0.6.0 must panic.
        // Command center infection should be implemented in next versions.
        assert!(automaton.command_center.is_infected());
    }
}
