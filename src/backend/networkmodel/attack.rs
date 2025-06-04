use thiserror::Error;

use crate::backend::CONTROL_FREQUENCY;
use crate::backend::connections::ConnectionGraph;
use crate::backend::device::{Device, DeviceId, IdToDeviceMap};
use crate::backend::malware::Malware;
use crate::backend::mathphysics::{Megahertz, Millisecond, Point3D};
use crate::backend::message::{Message, MessageType, MessageQueue};
use crate::backend::signal::GPS_L1_FREQUENCY;


#[derive(Error, Debug)]
pub enum MalwareSpreadError {
    #[error("Message was not received")]
    DoesNotSpread,
    #[error("Message does not contain malware")]
    NotMalicious,
}


/// # Errors
///
/// Will return `Err` if the message is not malicious or the malware does not
/// spread.
pub fn try_multiply_malicious_message_from_receivers(
    message: &Message,
    frequency: Megahertz,
    receiver_ids: &[DeviceId],
    device_map: &IdToDeviceMap,
    connections: &ConnectionGraph,
    malicious_messages: &mut Vec<(Megahertz, Message)>
) -> Result<(), MalwareSpreadError> {
    let MessageType::Malware(malware) = message.message_type() else {
        return Err(MalwareSpreadError::NotMalicious);
    };

    if !malware.spreads() {
        return Err(MalwareSpreadError::DoesNotSpread);
    }

    for receiver_id in receiver_ids {
        let Some(receiver) = device_map.get(receiver_id) else {
            continue;
        };
        if !receiver.is_infected_with(malware) {
            continue;
        }

        multiply_malicious_message(
            message, 
            frequency, 
            *receiver_id,
            connections, 
            malicious_messages
        );
    }

    Ok(())
}

fn multiply_malicious_message(
    initial_message: &Message,
    frequency: Megahertz,
    infected_device: DeviceId,
    connections: &ConnectionGraph,
    malicious_messages: &mut Vec<(Megahertz, Message)>
) {
    let MessageType::Malware(malware) = initial_message.message_type() else {
        return;
    };

    for neighbor in connections.neighbors(infected_device) {
        let malicious_message = Message::new(
            infected_device,
            neighbor,
            initial_message.time() + malware.infection_delay(),
            *initial_message.message_type()
        );
        
        malicious_messages.push((frequency, malicious_message));
    }
}

pub fn enqueue_malicious_messages(
    malicious_messages: &Vec<(Megahertz, Message)>,
    message_queue: &mut MessageQueue,
    drone_map: &IdToDeviceMap,
) {
    let mut infected_drones = Vec::new();

    for (frequency, malicious_message) in malicious_messages {
        let Some(drone) = drone_map.get(
            &malicious_message.destination_id()
        ) else {
            continue;
        };

        let MessageType::Malware(malware) = malicious_message.message_type() 
            else
        {
            continue;    
        };

        if drone.is_infected_with(malware) 
            || infected_drones.contains(&drone.id()) 
        {
            continue;
        }

        message_queue.add_message(*malicious_message, *frequency);
        
        infected_drones.push(drone.id());
    }
}

pub fn process_attack(
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

fn process_electronic_warfare(
    device_map: &mut IdToDeviceMap, 
    ewd: &Device
) {
    for device in device_map.devices_mut() {
        ewd.suppress_all_signals(device);
    }
}

fn process_gps_spoofing(
    device_map: &mut IdToDeviceMap,
    message_queue: &mut MessageQueue,
    spoofer: &Device,
    spoofed_position: &Point3D,
    current_time: Millisecond,
) {
    for device in device_map.devices() {
        if !spoofer.transmits_to(device, GPS_L1_FREQUENCY) {
            continue;
        }

        let fake_gps_message = Message::new(
            spoofer.id(), 
            device.id(), 
            current_time, 
            MessageType::GPS(*spoofed_position)
        );

        message_queue.add_message(fake_gps_message, GPS_L1_FREQUENCY);
    }
}

fn process_malware(
    device_map: &mut IdToDeviceMap,
    message_queue: &mut MessageQueue,
    malware_distributor: &Device,
    malware: &Malware,
    current_time: Millisecond
) {
    for device in device_map.devices() {
        if !malware_distributor.transmits_to(device, CONTROL_FREQUENCY) {
            continue;
        }

        let malicious_message = Message::new(
            malware_distributor.id(), 
            device.id(), 
            current_time + malware.infection_delay(), 
            MessageType::Malware(*malware)
        );

        message_queue.add_message(
            malicious_message,
            CONTROL_FREQUENCY, 
        );
    }
}


#[derive(Clone, Copy, Debug)]
pub enum AttackType {
    ElectronicWarfare,
    GPSSpoofing(Point3D),
    MalwareDistribution(Malware)
}


#[derive(Clone, Debug)]
pub struct AttackerDevice {
    device: Device,
    attack_type: AttackType
}

impl AttackerDevice {
    #[must_use]
    pub fn new(device: Device, attack_type: AttackType) -> Self {
        Self { device, attack_type }
    }

    #[must_use]
    pub fn device(&self) -> &Device {
        &self.device
    }

    #[must_use]
    pub fn attack_type(&self) -> AttackType {
        self.attack_type
    }
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::backend::connections::{ConnectionGraph, Topology};
    use crate::backend::device::{DeviceBuilder, DeviceId, IdToDeviceMap};
    use crate::backend::device::systems::{
        PowerSystem, TRXModule, TRXSystem, TRXSystemType
    };
    use crate::backend::malware::{Malware, MalwareType};
    use crate::backend::mathphysics::{Meter, PowerUnit};
    use crate::backend::message::MessageType;
    use crate::backend::signal::{
        SignalArea, SignalLevel, GREEN_SIGNAL_STRENGTH_VALUE, NO_SIGNAL_LEVEL, 
    };

    use super::*;

    
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const VERY_BIG_STRENGTH_VALUE: f32   = GREEN_SIGNAL_STRENGTH_VALUE * 1000.0;
    const DEVICE_MAX_POWER: PowerUnit    = 1_000;
    

    fn assert_all_messages_are_malicious(
        messages: &Vec<(Megahertz, Message)>
    ) {
        assert!(
            messages
                .iter()
                .all(|(_, message)| message.is_malware())
        );
    }

    fn correct_source_and_destination_ids(
        malicious_messages: &Vec<(Megahertz, Message)>, 
        source_id: DeviceId, 
        destination_id: DeviceId
    ) -> bool {
        malicious_messages
            .iter()
            .any(|(_, message)| 
                message.source_id() == source_id
                && message.destination_id() == destination_id
            )
    }
   
    fn jamming_malware(jammed_frequency: Megahertz) -> Malware {
        Malware::new(
            0, 
            MalwareType::Jamming(jammed_frequency),
            false,
        )
    }

    fn device_power_system() -> PowerSystem {
        PowerSystem::build(DEVICE_MAX_POWER, DEVICE_MAX_POWER)
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn drone_tx_module() -> TRXModule {
        let frequency = CONTROL_FREQUENCY;
        
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
        let frequency = CONTROL_FREQUENCY;
        
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
   
    fn drone_with_trx_system_set(position: Point3D) -> Device {
        DeviceBuilder::new()
            .set_real_position(position)
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    drone_tx_module(),
                    drone_rx_module() 
                )
            )
            .build()
    }


    #[test]
    fn multiplying_malicious_messages() {
        let frequency = CONTROL_FREQUENCY;
        let malware   = jamming_malware(frequency);

        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    drone_tx_module(),
                    TRXModule::default() 
                )
            )
            .build();
        let command_center_id = command_center.id();
        
        // Network topology:
        //                      D
        //                      |
        //                    (7.0)
        //                      |
        //  A -(7.0)- B -(9.0)- C
        //                      |
        //                    (7.0)
        //                      |
        //                      E
        //
        let devices = [
            command_center,
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, 7.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, -7.0, 0.0)),
        ];
        let drone1_id = devices[1].id();
        let drone2_id = devices[2].id();
        let drone3_id = devices[3].id();
        let drone4_id = devices[4].id();
        let device_map = IdToDeviceMap::from(devices);

        let mut connections = ConnectionGraph::new(Topology::Mesh);
        connections.update(
            command_center_id, 
            &device_map, 
            frequency
        );
        
        let initial_message_from_drone1 = Message::new(
            command_center_id,
            drone1_id,
            0, 
            MessageType::Malware(malware)
        );
        let mut malicious_messages = Vec::new();

        multiply_malicious_message(
            &initial_message_from_drone1, 
            frequency, 
            drone1_id,
            &connections, 
            &mut malicious_messages
        );

        assert_eq!(2, malicious_messages.len());
        assert_all_messages_are_malicious(&malicious_messages);
        // Message from drone1 (B) to the command center (A).
        assert!(
            correct_source_and_destination_ids(
                &malicious_messages, 
                drone1_id, 
                command_center_id
            )
        );
        // Message from drone1 (B) to drone2 (C).
        assert!(
            correct_source_and_destination_ids(
                &malicious_messages, 
                drone1_id, 
                drone2_id
            )
        );
        malicious_messages.clear();

        let initial_message_from_drone2 = Message::new(
            command_center_id,
            drone2_id,
            0, 
            MessageType::Malware(malware)
        );

        multiply_malicious_message(
            &initial_message_from_drone2, 
            frequency, 
            drone2_id,
            &connections, 
            &mut malicious_messages
        );

        assert_eq!(3, malicious_messages.len());
        assert_all_messages_are_malicious(&malicious_messages);
        // Message from drone2 (C) to drone1 (B).
        assert!(
            correct_source_and_destination_ids(
                &malicious_messages, 
                drone2_id, 
                drone1_id
            )
        );
        // Message from drone2 (C) to drone3 (D).
        assert!(
            correct_source_and_destination_ids(
                &malicious_messages, 
                drone2_id, 
                drone3_id
            )
        );
        // Message from drone2 (C) to drone4 (E).
        assert!(
            correct_source_and_destination_ids(
                &malicious_messages, 
                drone2_id, 
                drone4_id
            )
        );
    }
}
