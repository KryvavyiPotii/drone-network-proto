use std::collections::hash_map::Values;

use rand::prelude::*;

use crate::communication::{
    GPS_L1_FREQUENCY, Message, MessagePreprocessError, MessageQueue, 
    MessageType, NO_SIGNAL_LEVEL, SignalLevel, WIFI_2_4GHZ_FREQUENCY
};
use crate::device::{
    CommandCenter, Device, DeviceId, Drone, ElectronicWarfare, Receiver, 
    STEP_DURATION, Suppressor, Transceiver, Transmitter, UNKNOWN_ID,
};
use crate::device::networkmodel::{
    ConnectionGraph, FindSignalLevel, IdToDroneMap, IdToLevelMap, Topology
};
use crate::mathphysics::{Megahertz, Millisecond, Point3D};


const CHANGE_SIGNAL_LEVEL_FROM_GREEN_PROBABILITY: f32  = 0.95;
const CHANGE_SIGNAL_LEVEL_FROM_YELLOW_PROBABILITY: f32 = 0.70;
const CHANGE_SIGNAL_LEVEL_FROM_RED_PROBABILITY: f32    = 0.50;
const CHANGE_SIGNAL_LEVEL_FROM_BLACK_PROBABILITY: f32  = 0.0;


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
    let coefficient: f32 = rand::rng().random();
   
    coefficient <= probability
}

fn signal_level_change_probability(signal_level: SignalLevel) -> f32 {
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

fn send_message(
    drone_map: &mut IdToDroneMap,
    frequency: Megahertz,
    message: &Message,
) {
    let destination_id = message.destination_id();

    if destination_id == UNKNOWN_ID {
        broadcast_message(drone_map, frequency, message);
    } else {
        let Some(drone) = drone_map.get_mut(&destination_id) else {
            return;
        };

        let _ = drone.process_message(frequency, message);
    }
}

fn broadcast_message(
    drone_map: &mut IdToDroneMap,
    frequency: Megahertz,
    message: &Message,
) {
    for drone in drone_map.drones_mut() {
        let _ = drone.process_message(frequency, message);
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
        
        for (frequency, message) in &mut self.message_queue {
            match message.try_preprocess(
                self.current_time, 
                &self.connections
            ) {
                Err(
                    MessagePreprocessError::TooEarly
                    | MessagePreprocessError::UnknownSource
                ) => continue, 
                Err(MessagePreprocessError::AlreadyPreprocessed) => (),
                Ok(()) =>
                    match message.message_type() {
                        MessageType::ChangeGoal(_) => 
                            (),
                        MessageType::Infection => {
                            let destination_id = message.destination_id();

                            let neighbors = self.connections.neighbors_outgoing(
                                destination_id
                            );

                            for node in neighbors {
                                let infection_message = Message::new(
                                    destination_id,
                                    node,
                                    message.time(),
                                    MessageType::Infection
                                );
                                
                                infection_messages.push(infection_message);
                            }
                        },
                        MessageType::SetDestination(destination) =>
                            self.destination_in_meters = *destination,
                    }
            }
            
            send_message(&mut self.drone_map, *frequency, message);
            
            message.finish();
        }
        
        self.message_queue.remove_finished_messages();
        
        let mut infected_drones = Vec::new();

        for infection_message in infection_messages {
            let Some(drone) = self.drone_map.get(
                &infection_message.destination_id()
            ) else {
                continue;
            };

            if drone.is_infected() || infected_drones.contains(&drone.id()) {
                continue;
            }

            self.message_queue.add_message(
                WIFI_2_4GHZ_FREQUENCY,
                infection_message
            );
            
            infected_drones.push(drone.id());
        }
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
    
    use crate::communication::{
        GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE,
        SignalArea
    };
    use crate::device::{CommandCenterBuilder, Device, DroneBuilder};
    use crate::device::modules::{TRXModule, TRXSystem};
    use crate::mathphysics::Meter;
    
    use super::*;


    const SOME_FREQUENCY: Megahertz = 5_000;
    // These constants were introduced for testing independently from the global
    // constants.
    const DRONE_TX_CONTROL_RADIUS: Meter = 30.0;
    const VERY_BIG_STRENGTH_VALUE: f32 = GREEN_SIGNAL_STRENGTH_VALUE * 1_000.0;


    fn drone_tx_area() -> SignalArea {
        SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap()
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
}
