use std::collections::hash_map::Values;

use crate::device::{
    ConnectionGraph, DelaySnapshot, Device, DeviceId, FindSignalLevel, 
    IdToDeviceMap, IdToGoalMap, IdToLevelMap, Topology, BROADCAST_ID, 
    STEP_DURATION
};
use crate::device::networkmodel::{
    AttackerDevice, UnicastMessageError,  
    connect_gps_to_all_devices, enqueue_infection_messages, 
    process_gps_spoofing, send_gps_messages, 
    try_multiply_infection_message_from_receivers
};
use crate::mathphysics::{Megahertz, Millisecond};
use crate::message::{
    Message, MessagePreprocessError, MessageQueue, MessageType
};
use crate::signal::{GPS_L1_FREQUENCY, NO_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY};

use super::AttackType;


fn set_delays_snapshot_for_message(
    message: &Message,
    delays_snapshot: &mut DelaySnapshot,
    delays_snapshot_from_command_device: &DelaySnapshot,
    connections: &ConnectionGraph,
    delay_multiplier: f32,
) {
    match message.message_type() {
        // Currently (version 0.8.0) GPS data is given without delays.
        MessageType::GPS(_)       => (),
        MessageType::Infection(_) => { 
            let delays_snapshot_from_destination = connections.delays(
                message.destination_id(), 
                delay_multiplier
            );

            delays_snapshot.clone_from(&delays_snapshot_from_destination);
        },
        MessageType::SetGoal(_)   => 
            delays_snapshot.clone_from(delays_snapshot_from_command_device),
    }
}

fn send_message(
    message: &Message,
    frequency: Megahertz,
    device_map: &mut IdToDeviceMap,
    delays_snapshot: &DelaySnapshot,
    current_time: Millisecond
) -> Vec<DeviceId> {
    let destination_id = message.destination_id();

    if destination_id == BROADCAST_ID {
        broadcast_message(
            message,
            frequency,
            device_map,
            delays_snapshot,
            current_time
        )
    } else {
        let Some(device) = device_map.get_mut(&destination_id) else {
            return Vec::new();
        };
        let delay = delays_snapshot
            .get(&device.id())
            .unwrap_or(&0);

        if let Ok(device_id) = unicast_message(
            message, 
            frequency, 
            device, 
            *delay, 
            current_time
        ) {
            vec![device_id]
        } else {
            Vec::new()
        }
    }
}

fn unicast_message(
    message: &Message,
    frequency: Megahertz,
    device: &mut Device,
    delay: Millisecond,
    current_time: Millisecond
) -> Result<DeviceId, UnicastMessageError> {
    if current_time < message.time() + delay {
        return Err(UnicastMessageError::TooEarly);
    }
    if device.receive_and_process_message(message, frequency).is_err() {
        return Err(UnicastMessageError::NotReceived);
    }
    
    Ok(device.id())
}

fn broadcast_message(
    message: &Message,
    frequency: Megahertz,
    device_map: &mut IdToDeviceMap,
    delays_snapshot: &DelaySnapshot,
    current_time: Millisecond
) -> Vec<DeviceId> {
    let mut receiver_ids = Vec::new();

    for device in device_map.devices_mut() {
        let delay = delays_snapshot
            .get(&device.id())
            .unwrap_or(&0);

        if let Ok(device_id) = unicast_message(
            message, 
            frequency, 
            device, 
            *delay, 
            current_time
        ) {
            receiver_ids.push(device_id);
        }
    }

    receiver_ids
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

    let longest_delay = match delays_snapshot.values().max() {
        Some(longest_delay) => *longest_delay,
        None                => 0,
    };

    if current_time >= message.time() + longest_delay {
        message.finish(); 
    }
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
        ewd.suppress_signal(device, WIFI_2_4GHZ_FREQUENCY);
        ewd.suppress_signal(device, GPS_L1_FREQUENCY);
    }
}


#[derive(Clone)]
pub struct ComplexNetwork {
    current_time: Millisecond,
    command_device_id: DeviceId,
    device_map: IdToDeviceMap,
    attacker_devices: Vec<AttackerDevice>,
    // TODO add GPS message transmitters 
    topology: Topology,
    connections: ConnectionGraph,
    delay_multiplier: f32,
    message_queue: MessageQueue,
    // TODO add control frequency Vec
}

impl ComplexNetwork {
    #[must_use]
    pub fn new(
        command_device_id: DeviceId,
        device_map: IdToDeviceMap,
        attacker_devices: Vec<AttackerDevice>,
        scenario: &[(Megahertz, Message)],
        topology: Topology,
        delay_multiplier: f32
    ) -> Self {
        let mut complex_network = Self {
            current_time: 0,
            command_device_id,
            attacker_devices,
            device_map,
            topology,
            connections: ConnectionGraph::new(),
            delay_multiplier,
            message_queue: MessageQueue::from(scenario),
        };

        complex_network.set_initial_state();

        complex_network
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
        self.device_map.handle_infection();
        self.remove_disconnected_devices();
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
        self.device_map.clear_rx_signal_levels(WIFI_2_4GHZ_FREQUENCY);
        
        if let Ok(
            control_signal_levels
        ) = Self::try_find_best_signal_levels(
            self.command_device_id,
            &self.device_map,
            &self.connections,
            WIFI_2_4GHZ_FREQUENCY
        ) {
            self.device_map.all_receive_signal_levels(
                &control_signal_levels, 
                WIFI_2_4GHZ_FREQUENCY
            );
            self.device_map.copy_all_rx_signal_levels_to_tx(
                &self.command_device_id,
                WIFI_2_4GHZ_FREQUENCY
            );
        }
    }
    
    fn process_message_queue(&mut self) {
        //dbg!(&self.message_queue);
        if self.message_queue.is_empty() {
            return;
        }

        // Preprocess delays to avoid unnecessary function calls in the loop.
        let delays_snapshot_from_command_device = self.connections.delays(
            self.command_device_id, 
            self.delay_multiplier
        );

        let mut infection_messages = Vec::new();

        for (frequency, message, delays_snapshot) in &mut self.message_queue {
            let preprocess_result = message.try_preprocess(self.current_time);

            if let Err(MessagePreprocessError::TooEarly) = preprocess_result {
                continue;
            }
            if preprocess_result.is_ok() {
                set_delays_snapshot_for_message(
                    message,
                    delays_snapshot,
                    &delays_snapshot_from_command_device,
                    &self.connections,
                    self.delay_multiplier
                );
            }
            
            let receiver_ids = send_message(
                message,
                *frequency,
                &mut self.device_map,
                delays_snapshot,
                self.current_time
            );

            try_finish_message(
                self.current_time,
                delays_snapshot,
                message
            );

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

impl FindSignalLevel for ComplexNetwork {
    fn try_set_better_signal_level(
        tx: &Device,
        rx: &Device,
        signal_levels: &mut IdToLevelMap,
        frequency: Megahertz
    ) {
        // TODO find a way to avoid cloning (without multiple mutable borrows)
        let mut tx = tx.clone();

        if let Some(tx_signal_level) = signal_levels.get(&tx.id()) {
            tx.set_tx_signal_level(*tx_signal_level, frequency);
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

    use crate::device::{Device, DeviceBuilder};
    use crate::device::systems::{PowerSystem, TRXModule, TRXSystem};
    use crate::infection::{InfectionType, INFECTION_DELAY};
    use crate::mathphysics::{Meter, Point3D, PowerUnit};
    use crate::signal::{
        GREEN_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE, RED_SIGNAL_LEVEL, 
        SignalArea, SignalLevel, YELLOW_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY
    };
    
    use super::*;


    const CC_TX_CONTROL_RADIUS: Meter    = 300.0;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const VERY_BIG_STRENGTH_VALUE: f32   = 
        GREEN_SIGNAL_STRENGTH_VALUE * 1_000.0;
    const DEVICE_MAX_POWER: PowerUnit    = 10_000;

    
    fn device_power_system() -> PowerSystem {
        PowerSystem::build(DEVICE_MAX_POWER, DEVICE_MAX_POWER)
            .unwrap_or_else(|error| panic!("{}", error))
    }

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
                        None                => signal_level1
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
                SignalArea::build(CC_TX_CONTROL_RADIUS)
                    .unwrap_or_else(|error| panic!("{}", error)),
                frequency
            )
        )]);

        TRXModule::build(max_tx_signal_levels, tx_signal_levels)
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn drone_tx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        let max_tx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let tx_signal_levels = HashMap::from([(
            frequency, 
            SignalLevel::from_area(
                SignalArea::build(DRONE_TX_CONTROL_RADIUS) 
                    .unwrap_or_else(|error| panic!("{}", error)),
                frequency
            )
        )]);

        TRXModule::build(max_tx_signal_levels, tx_signal_levels)
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn drone_rx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        let max_rx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let rx_signal_levels = HashMap::from([
            (frequency, NO_SIGNAL_LEVEL)
        ]);

        TRXModule::build(max_rx_signal_levels, rx_signal_levels)
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn drone_with_trx_system_set(position: Point3D) -> Device {
        DeviceBuilder::new()
            .set_real_position(position)
            .set_power_system(device_power_system())
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

        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
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
        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: cc_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build();
           
        let command_center_id = command_center.id();

        // Network 1: full mesh with edge weight 25.0.
        let devices1 = [
            command_center.clone(),
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
        ];

        let expected_signal_levels1 = HashMap::from([
            (devices1[1].id(), GREEN_SIGNAL_LEVEL), 
            (devices1[2].id(), GREEN_SIGNAL_LEVEL), 
            (devices1[3].id(), GREEN_SIGNAL_LEVEL), 
            (devices1[4].id(), GREEN_SIGNAL_LEVEL), 
        ]);

        let network1 = ComplexNetwork::new(
            command_center_id, 
            IdToDeviceMap::from(devices1), 
            Vec::new(), 
            &Vec::new(), 
            Topology::Star, 
            0.0
        );

        let mut signal_levels1 = network1.device_map.all_rx_signal_levels(
            WIFI_2_4GHZ_FREQUENCY
        );
        signal_levels1.retain(|device_id, _| *device_id != command_center_id); 

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
        let devices2 = [
            command_center,
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(50.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(100.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(400.0, 0.0, 0.0)),
        ];
        
        let expected_signal_levels2 = HashMap::from([
            (devices2[1].id(), GREEN_SIGNAL_LEVEL), 
            (devices2[2].id(), YELLOW_SIGNAL_LEVEL), 
            (devices2[3].id(), RED_SIGNAL_LEVEL), 
        ]);

        let network2 = ComplexNetwork::new(
            command_center_id, 
            IdToDeviceMap::from(devices2), 
            Vec::new(), 
            &Vec::new(), 
            Topology::Mesh, 
            0.0
        );

        let mut signal_levels2 = network2.device_map.all_rx_signal_levels(
            WIFI_2_4GHZ_FREQUENCY
        );
        signal_levels2.retain(|device_id, _| *device_id != command_center_id); 

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
        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build();
           
        let command_center_id = command_center.id();

        let devices = [
            command_center,
            drone_with_trx_system_set(Point3D::new(1.1, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(2.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(3.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(10.0, 0.0, 0.0)),
        ];
        
        let expected_signal_levels = HashMap::from([
            (devices[1].id(), GREEN_SIGNAL_LEVEL), 
            (devices[2].id(), YELLOW_SIGNAL_LEVEL),
            (devices[3].id(), RED_SIGNAL_LEVEL),   
            (devices[4].id(), YELLOW_SIGNAL_LEVEL), // Black in Star topology 
        ]);

        let network = ComplexNetwork::new(
            command_center_id, 
            IdToDeviceMap::from(devices), 
            Vec::new(), 
            &Vec::new(), 
            Topology::Mesh, 
            0.0
        );

        let mut signal_levels = network.device_map.all_rx_signal_levels(
            WIFI_2_4GHZ_FREQUENCY
        );
        signal_levels.retain(|device_id, _| *device_id != command_center_id); 

        assert!(
            same_levels_of_id_to_level_maps(
                &expected_signal_levels,
                &signal_levels
            )
        );
    }

    #[test]
    fn spread_infection_in_star() {
        let vulnerable_device_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_vulnerabilities(&[InfectionType::Indicator])
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module() 
                }
            );
        
        // Network topology:
        //
        // B -(1.1)- A -(1.1)- C
        //
        let mut command_center = vulnerable_device_builder
            .clone()
            .build();
           
        command_center.set_rx_signal_level(
            GREEN_SIGNAL_LEVEL,
            WIFI_2_4GHZ_FREQUENCY, 
        );
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

        let devices = [command_center, infected_drone, vulnerable_drone];
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
       
        let mut network = ComplexNetwork::new(
            command_center_id, 
            device_map, 
            Vec::new(), 
            &scenario, 
            Topology::Star, 
            0.0
        );

        assert!(
            !network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_device().is_infected());

        network.update();
        
        assert!(
            network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_device().is_infected());

        wait_for_infection(&mut network);

        assert!(
            network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(network.command_device().is_infected());

        wait_for_infection(&mut network);

        assert!(
            network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(network.command_device().is_infected());
    }

    #[test]
    fn patched_devices_do_not_spread_infection() {
        let trx_system = TRXSystem::Strength { 
            tx_module: drone_tx_module(),
            rx_module: drone_rx_module() 
        };

        let vulnerable_device_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_vulnerabilities(&[InfectionType::Indicator])
            .set_trx_system(trx_system.clone());
        
        // Network topology:
        //
        // B -(1.1)- A -(1.1)- C
        //
        let mut patched_command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(trx_system)
            .build();
           
        patched_command_center.set_rx_signal_level(
            GREEN_SIGNAL_LEVEL,
            WIFI_2_4GHZ_FREQUENCY, 
        );
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

        let devices = [
            patched_command_center, 
            infected_drone, 
            vulnerable_drone
        ];
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
       
        let mut network = ComplexNetwork::new(
            patched_command_center_id, 
            device_map, 
            Vec::new(), 
            &scenario, 
            Topology::Star, 
            0.0
        );

        assert!(
            !network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_device().is_infected());

        network.update();
        
        assert!(
            network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_device().is_infected());

        wait_for_infection(&mut network);

        assert!(
            network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_device().is_infected());

        wait_for_infection(&mut network);

        assert!(
            network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_device().is_infected());
    }


    #[test]
    fn spread_infection_in_mesh() {
        let vulnerable_device_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_vulnerabilities(&[InfectionType::Indicator])
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module() 
                }
            );
        
        // Network Edges:
        // 
        // A -(1.1)- B  (undirected)
        // A -(2.0)- C  (undirected)
        // A -(3.0)- D  (undirected)
        // A -(10.0)- E (undirected)
        //
        let mut command_center = vulnerable_device_builder
            .clone()
            .build();
           
        command_center.set_rx_signal_level(
            GREEN_SIGNAL_LEVEL,
            WIFI_2_4GHZ_FREQUENCY, 
        );
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
        
        let devices = [
            command_center,
            infected_drone, 
            vulnerable_drone1,
            vulnerable_drone2,
            vulnerable_drone3,
        ];
        let scenario = vec!((
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                infected_drone_id,
                0, 
                MessageType::Infection(InfectionType::Indicator)
            ),
        ));
        
        let mut network = ComplexNetwork::new(
            command_center_id, 
            IdToDeviceMap::from(devices), 
            Vec::new(), 
            &scenario, 
            Topology::Mesh, 
            0.0
        );

        assert!(
            !network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_device().is_infected());

        network.update();       
        
        assert!(
            network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !network.device_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!network.command_device().is_infected());

        wait_for_infection(&mut network);

        assert!(
            network.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.device_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.device_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            network.device_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(network.command_device().is_infected());
    }
}
