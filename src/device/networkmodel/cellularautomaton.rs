use std::collections::hash_map::Values;

use crate::device::{
    BROADCAST_ID, ConnectionGraph, Device, DeviceId, FindSignalLevel, 
    IdToDeviceMap, IdToGoalMap, IdToLevelMap, STEP_DURATION, Topology, 
};
use crate::device::networkmodel::{
    AttackerDevice, AttackType, connect_gps_to_all_devices, 
    enqueue_infection_messages, process_gps_spoofing, send_gps_messages, 
    try_multiply_infection_message_from_receivers
};
use crate::mathphysics::{Megahertz, Millisecond};
use crate::message::{Message, MessagePreprocessError, MessageQueue};
use crate::signal::{
    GPS_L1_FREQUENCY, NO_SIGNAL_LEVEL, SignalLevel, WIFI_2_4GHZ_FREQUENCY
};

use super::UnicastMessageError;


const CHANGE_SIGNAL_LEVEL_FROM_GREEN_PROBABILITY: f64  = 0.95;
const CHANGE_SIGNAL_LEVEL_FROM_YELLOW_PROBABILITY: f64 = 0.70;
const CHANGE_SIGNAL_LEVEL_FROM_RED_PROBABILITY: f64    = 0.50;
const CHANGE_SIGNAL_LEVEL_FROM_BLACK_PROBABILITY: f64  = 0.0;


fn send_message_and_handle_infection(
    message: &Message,
    frequency: Megahertz,
    device_map: &mut IdToDeviceMap,
) -> Vec<DeviceId> {
    let destination_id = message.destination_id();

    if destination_id == BROADCAST_ID {
        broadcast_message_and_handle_infection(
            message,
            frequency,
            device_map,
        )
    } else {
        let Some(drone) = device_map.get_mut(&destination_id) else {
            return Vec::new()
        };

        if let Ok(device_id) = unicast_message_and_handle_infection(
            message, 
            frequency, 
            drone, 
        ) {
            vec![device_id]
        } else {
            Vec::new()
        }
    }
}

fn unicast_message_and_handle_infection(
    message: &Message,
    frequency: Megahertz,
    device: &mut Device,
) -> Result<DeviceId, UnicastMessageError> {
    if device.receive_and_process_message(message, frequency).is_err() {
        return Err(UnicastMessageError::NotReceived);
    }
    
    device.handle_infection();

    Ok(device.id())
}

fn broadcast_message_and_handle_infection(
    message: &Message,
    frequency: Megahertz,
    device_map: &mut IdToDeviceMap,
) -> Vec<DeviceId> {
    let mut receiver_ids = Vec::new();

    for device in device_map.devices_mut() {
        if let Ok(device_id) = unicast_message_and_handle_infection(
            message, 
            frequency, 
            device
        ) {
            receiver_ids.push(device_id);
        }
    }

    receiver_ids
}

fn process_attack(
    device_map: &mut IdToDeviceMap,
    message_queue: &mut MessageQueue,
    attacker_device: &AttackerDevice, 
    current_time: Millisecond
) {
    match attacker_device.attack_type() {
        AttackType::ElectronicWarfare => 
            process_electronic_warfare(device_map, attacker_device.device()),
        AttackType::GPSSpoofing(spoofed_position) => 
            process_gps_spoofing(
                device_map, 
                message_queue, 
                attacker_device.device(),
                &spoofed_position,
                current_time
            )
    }
}

fn process_electronic_warfare(device_map: &mut IdToDeviceMap, ewd: &Device) {
    for device in device_map.devices_mut() {
        suppress_with_probability(ewd, device, WIFI_2_4GHZ_FREQUENCY);
        suppress_with_probability(ewd, device, GPS_L1_FREQUENCY);
    }
}

fn suppress_with_probability(
    tx: &Device,
    rx: &mut Device,
    frequency: Megahertz
) {
    let suppressor_signal_level_at_rx = tx.propagated_signal_level_at(
        rx,
        frequency
    );

    if signal_level_change_happens(suppressor_signal_level_at_rx) {
        rx.signal_level_suppression(
            suppressor_signal_level_at_rx,
            frequency, 
        );
    }
}

// TODO optimize to avoid double rx.signal_level: in propagated_signal_level_at
// and in rx.signal_level itself
fn effective_propagated_signal(
    tx: &Device,
    rx: &Device,
    frequency: Megahertz
) -> SignalLevel {
    let tx_signal_level_at_rx = tx.propagated_signal_level_at(rx, frequency);

    if signal_level_change_happens(tx_signal_level_at_rx) {
        tx_signal_level_at_rx
    } else {
        let current_signal_level = *rx.rx_signal_level(frequency);
        
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
    command_device_id: DeviceId,
    device_map: IdToDeviceMap,
    attacker_devices: Vec<AttackerDevice>,
    topology: Topology,
    connections: ConnectionGraph,
    message_queue: MessageQueue,
}

impl CellularAutomaton {
    #[must_use]
    pub fn new(
        command_device_id: DeviceId,
        device_map: IdToDeviceMap,
        attacker_devices: Vec<AttackerDevice>,
        scenario: &[(Megahertz, Message)],
        topology: Topology,
    ) -> Self {
        let mut cellular_automaton = Self {
            current_time: 0,
            command_device_id,
            device_map,
            attacker_devices,
            topology,
            connections: ConnectionGraph::new(),
            message_queue: MessageQueue::from(scenario),
        };

        cellular_automaton.set_initial_state();

        cellular_automaton
    }
    
    #[must_use]
    pub fn goals(&self) -> IdToGoalMap {
        self.device_map.goals()
    }
    
    /// # Panics
    ///
    /// Will panic if a command device is not in `device_map`.
    #[must_use]
    pub fn command_device(&self) -> &Device {
        self.device_map
            .get(&self.command_device_id)
            .unwrap()
    }

    #[must_use]
    pub fn device_map(&self) -> &IdToDeviceMap {
        &self.device_map
    }

    #[must_use]
    pub fn device_iter(&self) -> Values<'_, DeviceId, Device> {
        self.device_map.devices() 
    }

    #[must_use]
    pub fn device_count(&self) -> usize {
        self.device_map.len() 
    }

    #[must_use]
    pub fn attacker_devices(&self) -> &[AttackerDevice] {
        self.attacker_devices.as_slice()
    }   

    #[must_use]
    pub fn connections(&self) -> &ConnectionGraph {
        &self.connections
    }   
    
    fn set_initial_state(&mut self) {
        self.update_connections_graph();
        self.update_signal_levels();
        self.remove_disconnected_devices();
        connect_gps_to_all_devices(&mut self.device_map);
        self.send_gps_messages();
        self.process_message_queue();
    }
     
    pub fn update(&mut self) {
        self.update_connections_graph();
        self.update_signal_levels();
        self.process_attacks();
        self.remove_disconnected_devices();
        self.process_message_queue();
        self.device_map.update_states(); 
        self.send_gps_messages();
        connect_gps_to_all_devices(&mut self.device_map);
      
        self.current_time += STEP_DURATION;
    }
   
    fn update_connections_graph(&mut self) {
        self.connections.update(
            self.command_device_id, 
            &self.device_map,
            self.topology,
            WIFI_2_4GHZ_FREQUENCY
        );
    }

    fn update_signal_levels(&mut self) {
        if let Ok(
            control_signal_levels
        ) = Self::try_find_best_signal_levels(
            self.command_device_id,
            &self.device_map,
            &self.connections,
            WIFI_2_4GHZ_FREQUENCY
        ) {
            self.device_map.set_rx_signal_levels(
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
            if let Err(
                MessagePreprocessError::TooEarly
            ) = message.try_preprocess(self.current_time) {
                continue;
            }
            
            let receiver_ids = send_message_and_handle_infection(
                message,
                *frequency,
                &mut self.device_map, 
            );

            message.finish();

            try_multiply_infection_message_from_receivers(
                message,
                *frequency,
                &receiver_ids,
                &self.device_map,
                &self.connections,
                &mut infection_messages
            );
        }
        
        self.message_queue.remove_finished_messages();
        
        enqueue_infection_messages(
            &infection_messages,
            &mut self.message_queue,
            &self.device_map,
        );
    }

    fn process_attacks(&mut self) {
        for attacker_device in &self.attacker_devices {
            process_attack(
                &mut self.device_map, 
                &mut self.message_queue, 
                attacker_device,
                self.current_time
            ); 
        }
    }

    fn send_gps_messages(&mut self) {
        send_gps_messages(
            &self.device_map, 
            &mut self.message_queue, 
            self.current_time
        );
    }
    
    fn remove_disconnected_devices(&mut self) {
        self.device_map.remove_not_receiving_devices(
            &self.command_device_id,
            WIFI_2_4GHZ_FREQUENCY
        );
    }
}

impl FindSignalLevel for CellularAutomaton {
    fn try_set_better_signal_level(
        tx: &Device,
        rx: &Device,
        signal_levels: &mut IdToLevelMap,
        frequency: Megahertz
    ) {
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
    
    use crate::device::DeviceBuilder;
    use crate::device::systems::{PowerSystem, TRXModule, TRXSystem};
    use crate::infection::{InfectionType, INFECTION_DELAY};
    use crate::mathphysics::{Meter, Point3D, PowerUnit};
    use crate::message::MessageType;
    use crate::signal::{
        GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE,
        SignalArea
    };
    
    use super::*;


    const SOME_FREQUENCY: Megahertz      = 5_000;
    const DRONE_TX_CONTROL_RADIUS: Meter = 30.0;
    const VERY_BIG_STRENGTH_VALUE: f32  = GREEN_SIGNAL_STRENGTH_VALUE * 1_000.0;
    const DEVICE_MAX_POWER: PowerUnit    = 10_000;

    
    fn device_power_system() -> PowerSystem {
        PowerSystem::build(DEVICE_MAX_POWER, DEVICE_MAX_POWER)
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn drone_tx_area() -> SignalArea {
        SignalArea::build(DRONE_TX_CONTROL_RADIUS)
            .unwrap_or_else(|error| panic!("{}", error))
           
    }
    
    fn wait_for_infection(automaton: &mut CellularAutomaton) {
        for _ in (0..=INFECTION_DELAY).step_by(STEP_DURATION as usize) {
            automaton.update();
        }
    }
    
    fn drone_trx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        let max_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let signal_levels = HashMap::from([
            (frequency, NO_SIGNAL_LEVEL)
        ]);

        TRXModule::build(max_signal_levels, signal_levels)
            .unwrap_or_else(|error| panic!("{}", error))
           
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
        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::Color(
                    TRXModule::build(max_cc_signal_level, cc_signal_level)
                        .unwrap_or_else(|error| panic!("{}", error))
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
        ).unwrap_or_else(|error| panic!("{}", error));

        let drone_trx_system = TRXSystem::Color(trx_module);

        let drones = [
            DeviceBuilder::new()
                .set_real_position(Point3D::new(10.0, 0.0, 0.0))
                .set_power_system(device_power_system())
                .set_trx_system(drone_trx_system.clone())
                .build(),
            DeviceBuilder::new()
                .set_real_position(Point3D::new(20.0, 0.0, 0.0))
                .set_power_system(device_power_system())
                .set_trx_system(drone_trx_system.clone())
                .build(),
            DeviceBuilder::new()
                .set_real_position(Point3D::new(25.0, 0.0, 0.0))
                .set_power_system(device_power_system())
                .set_trx_system(drone_trx_system.clone())
                .build(),
            DeviceBuilder::new()
                .set_real_position(Point3D::new(35.0, 0.0, 0.0))
                .set_power_system(device_power_system())
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
        let vulnerable_device_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_vulnerabilities(&[InfectionType::Indicator])
            .set_trx_system(TRXSystem::Color(drone_trx_module()));
        
        // Network topology:
        //
        // B -(1.1)- A -(1.1)- C
        //
        let command_center = vulnerable_device_builder
            .clone()
            .build();
           
        let command_center_id = command_center.id();
        
        let infected_drone = vulnerable_device_builder
            .clone()
            .set_real_position(Point3D::new(1.1, 0.0, 0.0))
            .build();
           
        let infected_drone_id = infected_drone.id();
        let vulnerable_drone = vulnerable_device_builder
            .set_real_position(Point3D::new(-1.1, 0.0, 0.0))
            .build();
           
        let vulnerable_drone_id = vulnerable_drone.id();

        let mut devices = [command_center, infected_drone, vulnerable_drone];
        // We need to manually set all rx signal levels to GREEN because in 
        // `CellularAutomaton` signal level changes happen non-
        // deterministically.
        for device in &mut devices {
            device.set_rx_signal_level(
                GREEN_SIGNAL_LEVEL,
                WIFI_2_4GHZ_FREQUENCY, 
            );
        }
        let scenario = vec!((
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                infected_drone_id,
                0, 
                MessageType::Infection(InfectionType::Indicator)
            ),
        ));
        let device_map = IdToDeviceMap::from(devices);
       
        let mut automaton = CellularAutomaton::new(
            command_center_id, 
            device_map, 
            Vec::new(), 
            &scenario, 
            Topology::Star, 
        );

        assert!(
            !automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_device().is_infected());

        automaton.update();
        
        assert!(
            automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_device().is_infected());

        wait_for_infection(&mut automaton);

        assert!(
            automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(automaton.command_device().is_infected());

        wait_for_infection(&mut automaton);

        assert!(
            automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(automaton.command_device().is_infected());
    }

    #[test]
    fn patched_devices_do_not_spread_infection() {
        let trx_system = TRXSystem::Color(drone_trx_module());
        let vulnerable_device_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_vulnerabilities(&[InfectionType::Indicator])
            .set_trx_system(trx_system.clone());
        
        // Network topology:
        //
        // B -(1.1)- A -(1.1)- C
        //
        let patched_command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(trx_system)
            .build();
           
        let patched_command_center_id = patched_command_center.id();
        
        let infected_drone = vulnerable_device_builder
            .clone()
            .set_real_position(Point3D::new(1.1, 0.0, 0.0))
            .build();
           
        let infected_drone_id = infected_drone.id();
        let vulnerable_drone = vulnerable_device_builder
            .set_real_position(Point3D::new(-1.1, 0.0, 0.0))
            .build();
           
        let vulnerable_drone_id = vulnerable_drone.id();

        let mut devices = [
            patched_command_center, 
            infected_drone, 
            vulnerable_drone
        ];
        // We need to manually set all rx signal levels to GREEN because in 
        // `CellularAutomaton` signal level changes happen non-
        // deterministically.
        for device in &mut devices {
            device.set_rx_signal_level(
                GREEN_SIGNAL_LEVEL,
                WIFI_2_4GHZ_FREQUENCY, 
            );
        }
        let scenario = vec!((
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                patched_command_center_id,
                infected_drone_id,
                0, 
                MessageType::Infection(InfectionType::Indicator)
            ),
        ));
        let device_map = IdToDeviceMap::from(devices);
       
        let mut automaton = CellularAutomaton::new(
            patched_command_center_id, 
            device_map, 
            Vec::new(), 
            &scenario, 
            Topology::Star, 
        );

        assert!(
            !automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_device().is_infected());

        automaton.update();
        
        assert!(
            automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_device().is_infected());

        wait_for_infection(&mut automaton);

        assert!(
            automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_device().is_infected());

        wait_for_infection(&mut automaton);

        assert!(
            automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_device().is_infected());
    }

    #[test]
    fn spread_infection_in_mesh() {
        let vulnerable_device_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_vulnerabilities(&[InfectionType::Indicator])
            .set_trx_system(TRXSystem::Color(drone_trx_module()));
        
        // Network Edges:
        // 
        // A -(1.1)- B  (undirected)
        // A -(2.0)- C  (undirected)
        // A -(3.0)- D  (undirected)
        // A -(10.0)- E (undirected)
        //
        let command_center = vulnerable_device_builder
            .clone()
            .build();
           
        let command_center_id = command_center.id();
        
        let infected_drone = vulnerable_device_builder
            .clone()
            .set_real_position(Point3D::new(10.0, 0.0, 0.0))
            .build();
           
        let infected_drone_id = infected_drone.id();
        let vulnerable_drone1 = vulnerable_device_builder
            .clone()
            .set_real_position(Point3D::new(1.1, 0.0, 0.0))
            .build();
           
        let vulnerable_drone_id1 = vulnerable_drone1.id();
        let vulnerable_drone2 = vulnerable_device_builder
            .clone()
            .set_real_position(Point3D::new(2.0, 0.0, 0.0))
            .build();
           
        let vulnerable_drone_id2 = vulnerable_drone2.id();
        let vulnerable_drone3 = vulnerable_device_builder
            .clone()
            .set_real_position(Point3D::new(3.0, 0.0, 0.0))
            .build();
           
        let vulnerable_drone_id3 = vulnerable_drone3.id();
        
        let mut devices = [
            command_center,
            infected_drone, 
            vulnerable_drone1,
            vulnerable_drone2,
            vulnerable_drone3,
        ];
        // We need to manually set all rx signal levels to GREEN because in 
        // `CellularAutomaton` signal level changes happen non-
        // deterministically.
        for device in &mut devices {
            device.set_rx_signal_level(
                GREEN_SIGNAL_LEVEL,
                WIFI_2_4GHZ_FREQUENCY, 
            );
        }
        let scenario = vec!((
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                infected_drone_id,
                0, 
                MessageType::Infection(InfectionType::Indicator)
            ),
        ));
        
        let mut automaton = CellularAutomaton::new(
            command_center_id, 
            IdToDeviceMap::from(devices), 
            Vec::new(), 
            &scenario, 
            Topology::Mesh, 
        );

        assert!(
            !automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_device().is_infected());

        automaton.update();       
        
        assert!(
            automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !automaton.device_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!automaton.command_device().is_infected());

        wait_for_infection(&mut automaton);

        assert!(
            automaton.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.device_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.device_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            automaton.device_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(automaton.command_device().is_infected());
    }
}
