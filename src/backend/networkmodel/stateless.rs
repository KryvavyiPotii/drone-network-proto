use std::collections::hash_map::Values;
use std::collections::HashMap;

use crate::backend::ITERATION_TIME;
use crate::backend::connections::{ConnectionGraph, FindSignalLevel, Topology};
use crate::backend::device::{
    IdToDelayMap, Device, DeviceId, IdToDeviceMap, IdToLevelMap, IdToTaskMap, 
    SignalLossResponse, BROADCAST_ID, 
};
use crate::backend::mathphysics::{Megahertz, Millisecond};
use crate::backend::message::{Message, MessageQueue, MessageType, Task};
use crate::backend::signal::{NO_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY};

use super::get_any_device;
use super::attack::{
    AttackerDevice, AttackType, enqueue_malicious_messages, 
    process_electronic_warfare, process_gps_spoofing, process_malware,
    try_multiply_malicious_message_from_receivers,
};
use super::gps::GPS;
use super::msgproc::{
    MessageProcessError, UnicastMessageError, set_delay_map_for_message, 
    try_add_task, try_finish_message, try_preprocess_message
};


fn send_message(
    message: &Message,
    frequency: Megahertz,
    device_map: &mut IdToDeviceMap,
    delay_map: &IdToDelayMap,
    current_time: Millisecond
) -> Vec<DeviceId> {
    let destination_id = message.destination_id();

    if destination_id == BROADCAST_ID {
        broadcast_message(
            message,
            frequency,
            device_map,
            delay_map,
            current_time
        )
    } else {
        let Some(device) = device_map.get_mut(&destination_id) else {
            return Vec::new();
        };
        let delay = delay_map
            .get(&device.id())
            .unwrap_or(&0);

        let unicast_result = unicast_message(
            message, 
            frequency, 
            device, 
            *delay, 
            current_time
        );

        if let Ok(device_id) = unicast_result {
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
    delay_map: &IdToDelayMap,
    current_time: Millisecond
) -> Vec<DeviceId> {
    let mut receiver_ids = Vec::new();

    for device in device_map.devices_mut() {
        let delay = delay_map
            .get(&device.id())
            .unwrap_or(&0);

        let unicast_result = unicast_message(
            message, 
            frequency, 
            device, 
            *delay, 
            current_time
        );

        if let Ok(device_id) = unicast_result {
            receiver_ids.push(device_id);
        }
    }

    receiver_ids
}

fn process_attack(
    device_map: &mut IdToDeviceMap,
    message_queue: &mut MessageQueue,
    attacker_device: &AttackerDevice, 
    current_time: Millisecond,
) {
    match attacker_device.attack_type() {
        AttackType::ElectronicWarfare             => 
            process_electronic_warfare(device_map, attacker_device.device()),
        AttackType::GPSSpoofing(spoofed_position) => 
            process_gps_spoofing(
                device_map, 
                message_queue, 
                attacker_device.device(),
                &spoofed_position,
                current_time,
            ),
        AttackType::MalwareDistribution(malware)  =>
            process_malware(
                device_map, 
                message_queue, 
                attacker_device.device(),
                &malware,
                current_time
            )
    }
}


#[derive(Clone)]
pub struct StatelessModel {
    current_time: Millisecond,
    command_device_id: DeviceId,
    device_map: IdToDeviceMap,
    attacker_devices: Vec<AttackerDevice>,
    gps: GPS,
    connections: ConnectionGraph,
    delay_multiplier: f32,
    message_queue: MessageQueue,
    current_tasks: IdToTaskMap,
    // TODO add control frequency Vec
}

impl StatelessModel {
    #[must_use]
    pub fn new(
        command_device_id: DeviceId,
        device_map: IdToDeviceMap,
        attacker_devices: Vec<AttackerDevice>,
        gps: GPS,
        scenario: &[(Megahertz, Message)],
        topology: Topology,
        delay_multiplier: f32
    ) -> Self {
        let message_queue = MessageQueue::from(scenario);

        let mut stateless_model = Self {
            current_time: 0,
            command_device_id,
            attacker_devices,
            device_map,
            gps,
            connections: ConnectionGraph::new(topology),
            delay_multiplier,
            message_queue,
            current_tasks: HashMap::new(),
        };

        stateless_model.set_initial_state();

        stateless_model
    }
    
    #[must_use]
    pub fn device_tasks(&self) -> IdToTaskMap {
        self.device_map.tasks()
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
        self.connect_gps_to_all_devices();
        self.send_gps_messages();
        self.update_connections_graph();
        self.update_signal_levels();
    }

    pub fn update(&mut self) {
        self.update_connections_graph();
        self.update_signal_levels();
        self.reconnect_devices();
        self.process_attacks();
        self.process_message_queue();
        self.device_map.handle_infection();
        self.device_map.update_states();
        self.send_gps_messages();
        self.connect_gps_to_all_devices();
     
        self.current_time += ITERATION_TIME;
    }

    fn update_connections_graph(&mut self) {
        self.connections.update(
            self.command_device_id, 
            &self.device_map,
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
        if self.message_queue.is_empty() {
            return;
        }

        let mut malicious_messages = Vec::new();

        for (frequency, message, delay_map) in &mut self.message_queue {
            let preprocess_result = try_preprocess_message(
                message, 
                self.current_time
            );

            if let Err(MessageProcessError::TooEarly) = preprocess_result {
                continue;
            }
            if preprocess_result.is_ok() {
                let _ = try_add_task(
                    message,
                    &mut self.current_tasks, 
                    &self.device_map,
                );

                let Some(source_device) = get_any_device(
                    message.source_id(),
                    &self.device_map,
                    &self.attacker_devices,
                    &self.gps,
                ) else {
                    message.finish();
                    continue;
                };
                
                set_delay_map_for_message(
                    message,
                    delay_map,
                    source_device,
                    &self.device_map,
                    &self.connections,
                    self.delay_multiplier
                );
            }
            
            let receiver_ids = send_message(
                message,
                *frequency,
                &mut self.device_map,
                delay_map,
                self.current_time
            );

            let _ = try_finish_message(
                message,
                delay_map,
                self.current_time,
            );

            let _ = try_multiply_malicious_message_from_receivers(
                message,
                *frequency,
                &receiver_ids,
                &self.device_map,
                &self.connections,
                &mut malicious_messages
            );
        }
        self.message_queue.remove_finished_messages();

        enqueue_malicious_messages(
            &malicious_messages,
            &mut self.message_queue,
            &self.device_map,
        );
    }

    fn reconnect_devices(&mut self) {
        self.remove_shut_down_devices();

        for (device_id, device) in &self.device_map {
            let Task::Reconnect(_) = device.task() else {
                continue;
            };

            let current_task = *self.current_tasks
                .get(device_id)
                .unwrap_or(&Task::Undefined);
            let task_message = Message::new(
                self.command_device_id, 
                *device_id, 
                self.current_time, 
                MessageType::SetTask(current_task)
            );

            self.message_queue.add_message(
                task_message,
                WIFI_2_4GHZ_FREQUENCY, 
            );
        }
    }
    
    fn remove_shut_down_devices(&mut self) {
        self.device_map.retain(|device_id, device|
            *device_id == self.command_device_id
            || device.receives_signal(WIFI_2_4GHZ_FREQUENCY)
            || !matches!(
                device.signal_loss_response(), 
                SignalLossResponse::Shutdown
            )
        );
    }
    
    fn process_attacks(&mut self) {
        for attacker_device in &self.attacker_devices {
            process_attack(
                &mut self.device_map, 
                &mut self.message_queue, 
                attacker_device,
                self.current_time,
            ); 
        }
    }

    fn connect_gps_to_all_devices(&mut self) {
        self.gps.connect_gps_to_all_devices(&mut self.device_map);
    }

    fn send_gps_messages(&mut self) {
        self.gps.send_gps_messages(
            &self.device_map, 
            &mut self.message_queue, 
            self.current_time,
        );
    }
}

impl FindSignalLevel for StatelessModel {
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

    use crate::backend::device::{Device, DeviceBuilder, SignalLossResponse};
    use crate::backend::device::systems::{PowerSystem, TRXModule, TRXSystem};
    use crate::backend::mathphysics::{Meter, Point3D, PowerUnit};
    use crate::backend::malware::{Malware, MalwareType};
    use crate::backend::signal::{
        GREEN_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE, GPS_L1_FREQUENCY,
        RED_SIGNAL_LEVEL, SignalArea, SignalLevel, YELLOW_SIGNAL_LEVEL, 
        WIFI_2_4GHZ_FREQUENCY
    };
    
    use super::*;


    const CC_TX_CONTROL_RADIUS: Meter    = 300.0;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const DEFAULT_GPS_POSITION_IN_METERS: Point3D = Point3D { 
        x: 0.0, 
        y: 0.0, 
        z: GPS_TX_RADIUS / 2.0
    };
    const DEVICE_MAX_POWER: PowerUnit   = 10_000;
    const GPS_TX_RADIUS: Meter          = 1_000.0;
    const VERY_BIG_STRENGTH_VALUE: f32  = GREEN_SIGNAL_STRENGTH_VALUE * 1_000.0;

    fn gps_tx_module() -> TRXModule {
        let frequency = GPS_L1_FREQUENCY;
        
        let max_tx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let tx_signal_levels = HashMap::from([(
            frequency, 
            SignalLevel::from_area(
                SignalArea::build(GPS_TX_RADIUS).unwrap(), 
                frequency
            )
        )]);

        TRXModule::build(
            max_tx_signal_levels,
            tx_signal_levels,
        ).unwrap()
    }

    fn default_gps() -> GPS {
        let trx_system = TRXSystem::Strength { 
            tx_module: gps_tx_module(),
            rx_module: TRXModule::default()
        };

        let device = DeviceBuilder::new()
            .set_real_position(DEFAULT_GPS_POSITION_IN_METERS)
            .set_signal_loss_response(SignalLossResponse::Ignore)
            .set_power_system(device_power_system())
            .set_trx_system(trx_system)
            .build();

        GPS::new(device, GPS_L1_FREQUENCY)
    }

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

    fn wait_for_infection(
        stateless_model: &mut StatelessModel, 
        infection_delay: Millisecond
    ) {
        for _ in (0..=infection_delay).step_by(ITERATION_TIME as usize) {
            stateless_model.update();
        }
    }
    
    fn indicator_malware() -> Malware {
        Malware::new(
            ITERATION_TIME * 10, 
            MalwareType::Indicator,
            true,
        )
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
            StatelessModel::try_set_better_signal_level(
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

        let stateless_model1 = StatelessModel::new(
            command_center_id, 
            IdToDeviceMap::from(devices1), 
            Vec::new(), 
            default_gps(),
            &Vec::new(), 
            Topology::Star, 
            0.0
        );

        let mut signal_levels1 = stateless_model1.device_map
            .all_rx_signal_levels(WIFI_2_4GHZ_FREQUENCY);
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

        let stateless_model2 = StatelessModel::new(
            command_center_id, 
            IdToDeviceMap::from(devices2), 
            Vec::new(), 
            default_gps(),
            &Vec::new(), 
            Topology::Mesh, 
            0.0
        );

        let mut signal_levels2 = stateless_model2.device_map
            .all_rx_signal_levels(WIFI_2_4GHZ_FREQUENCY);
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

        let stateless_model = StatelessModel::new(
            command_center_id, 
            IdToDeviceMap::from(devices), 
            Vec::new(), 
            default_gps(),
            &Vec::new(), 
            Topology::Mesh, 
            0.0
        );

        let mut signal_levels = stateless_model.device_map.all_rx_signal_levels(
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
        let indicator_malware = indicator_malware();
        let vulnerable_device_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_vulnerabilities(&[indicator_malware])
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
                MessageType::Malware(indicator_malware)
            ),
        ));
        let device_map = IdToDeviceMap::from(devices);
       
        let mut stateless_model = StatelessModel::new(
            command_center_id, 
            device_map, 
            Vec::new(), 
            default_gps(),
            &scenario, 
            Topology::Star, 
            0.0
        );

        assert!(
            !stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!stateless_model.command_device().is_infected());

        stateless_model.update();
        
        assert!(
            stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!stateless_model.command_device().is_infected());

        wait_for_infection(
            &mut stateless_model, 
            indicator_malware.infection_delay()
        );

        assert!(
            stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(stateless_model.command_device().is_infected());

        wait_for_infection(
            &mut stateless_model, 
            indicator_malware.infection_delay()
        );

        assert!(
            stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            stateless_model.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(stateless_model.command_device().is_infected());
    }

    #[test]
    fn patched_devices_do_not_spread_infection() {
        let indicator_malware = indicator_malware();
        let trx_system = TRXSystem::Strength { 
            tx_module: drone_tx_module(),
            rx_module: drone_rx_module() 
        };

        let vulnerable_device_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_vulnerabilities(&[indicator_malware])
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
                MessageType::Malware(indicator_malware)
            ),
        ));
        let device_map = IdToDeviceMap::from(devices);
       
        let mut stateless_model = StatelessModel::new(
            patched_command_center_id, 
            device_map, 
            Vec::new(), 
            default_gps(),
            &scenario, 
            Topology::Star, 
            0.0
        );

        assert!(
            !stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!stateless_model.command_device().is_infected());

        stateless_model.update();
        
        assert!(
            stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!stateless_model.command_device().is_infected());

        wait_for_infection(
            &mut stateless_model, 
            indicator_malware.infection_delay()
        );

        assert!(
            stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!stateless_model.command_device().is_infected());

        wait_for_infection(
            &mut stateless_model, 
            indicator_malware.infection_delay()
        );

        assert!(
            stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(!stateless_model.command_device().is_infected());
    }


    #[test]
    fn spread_infection_in_mesh() {
        let indicator_malware = indicator_malware();
        let vulnerable_device_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_vulnerabilities(&[indicator_malware])
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
                MessageType::Malware(indicator_malware)
            ),
        ));
        
        let mut stateless_model = StatelessModel::new(
            command_center_id, 
            IdToDeviceMap::from(devices), 
            Vec::new(), 
            default_gps(),
            &scenario, 
            Topology::Mesh, 
            0.0
        );

        assert!(
            !stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!stateless_model.command_device().is_infected());

        stateless_model.update();       
        
        assert!(
            stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            !stateless_model.device_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(!stateless_model.command_device().is_infected());

        wait_for_infection(
            &mut stateless_model, 
            indicator_malware.infection_delay()
        );

        assert!(
            stateless_model.device_map
                .get(&infected_drone_id)
                .unwrap()
                .is_infected()
        );
        assert!(
            stateless_model.device_map
                .get(&vulnerable_drone_id1)
                .unwrap()
                .is_infected()
        );
        assert!(
            stateless_model.device_map
                .get(&vulnerable_drone_id2)
                .unwrap()
                .is_infected()
        );
        assert!(
            stateless_model.device_map
                .get(&vulnerable_drone_id3)
                .unwrap()
                .is_infected()
        );
        assert!(stateless_model.command_device().is_infected());
    }
}
