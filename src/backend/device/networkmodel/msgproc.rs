use thiserror::Error;

use crate::backend::device::{
    DeviceId, IdToDeviceMap, IdToTaskMap, BROADCAST_ID, UNKNOWN_ID
};
use crate::backend::mathphysics::{Millisecond, Position};
use crate::backend::message::{Message, MessageType, MessageQueue};
use crate::backend::signal::{GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL};


#[derive(Error, Debug)]
pub enum MessagePreprocessError {
    #[error("Message is already preprocessed")]
    AlreadyPreprocessed,
    #[error("Message execution time is set to be later")]
    TooEarly,
}


#[derive(Error, Debug)]
pub enum UnicastMessageError {
    #[error("Message was not received")]
    NotReceived,
    #[error("Message should be sent later")]
    TooEarly,
}


pub fn try_add_task(
    message: &Message,
    current_tasks: &mut IdToTaskMap,
    device_map: &IdToDeviceMap,
) {
    if !message.is_in_progress() {
        return;
    }
    let MessageType::SetTask(task) = *message.message_type() else {
        return;
    };

    let destination_id = message.destination_id();
    if destination_id != BROADCAST_ID {
        current_tasks.insert(destination_id, task);
        return;
    }

    for device_id in device_map.ids() {
        current_tasks.insert(*device_id, task);
    }    
}

/// # Errors
///
/// Will return `Err` if execution time is greater than current time or
/// message is already in progress or `ConnectionGraph` does not contain
/// source device ID in message.
pub fn try_preprocess_message(
    message: &mut Message,
    current_time: Millisecond,
) -> Result<(), MessagePreprocessError> {
    if message.is_in_progress() {
        return Err(MessagePreprocessError::AlreadyPreprocessed);
    }
    if current_time < message.time() {
        return Err(MessagePreprocessError::TooEarly);
    }

    message.process();

    Ok(())
}

pub fn connect_gps_to_all_devices(device_map: &mut IdToDeviceMap) {
    device_map.set_rx_signal_level(
        &GREEN_SIGNAL_LEVEL,
        GPS_L1_FREQUENCY, 
    );
}

pub fn send_gps_messages(
    device_map: &IdToDeviceMap,
    message_queue: &mut MessageQueue,
    current_time: Millisecond
) {
    for (device_id, device) in device_map {
        if message_queue_already_contains_gps_message_for(
            message_queue, 
            *device_id
        ) {
            continue; 
        }

        let gps_position = device.position();
        let gps_message = Message::new(
            UNKNOWN_ID, 
            *device_id,
            current_time, 
            MessageType::GPS(*gps_position)
        );

        message_queue.add_message(GPS_L1_FREQUENCY, gps_message);
    }
}

#[must_use]
pub fn message_queue_already_contains_gps_message_for(
    message_queue: &MessageQueue, 
    device_id: DeviceId
) -> bool {
    message_queue
        .iter()
        .any(|(_, message, _)| 
            message.destination_id() == device_id && message.is_gps()
        )
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::backend::device::{
        Device, DeviceBuilder, PowerSystem, TRXSystem, BROADCAST_ID
    };
    use crate::backend::device::systems::TRXModule;
    use crate::backend::mathphysics::{Meter, Point3D, PowerUnit};
    use crate::backend::message::{Task, Message};
    use crate::backend::signal::{
        SignalArea, SignalLevel, NO_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE, 
        WIFI_2_4GHZ_FREQUENCY
    };

    use super::*;

    
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const VERY_BIG_STRENGTH_VALUE: f32   = GREEN_SIGNAL_STRENGTH_VALUE * 1000.0;
    const DEVICE_MAX_POWER: PowerUnit    = 1_000;


    fn default_gps_message_for(
        device_id: DeviceId,
        execution_time: Millisecond
    ) -> Message {
        Message::new(
            UNKNOWN_ID, 
            device_id, 
            execution_time, 
            MessageType::GPS(Point3D::default())
        )
    }

    fn message_queue_contains(
        message_queue: &MessageQueue, 
        message_to_find: &Message
    ) -> bool {
        message_queue
            .iter()
            .find(|(_, message, _)| message == message_to_find)
            .is_some()
    }
     
    fn device_power_system() -> PowerSystem {
        PowerSystem::build(DEVICE_MAX_POWER, DEVICE_MAX_POWER)
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


    #[test]
    fn preprocessing_already_preprocessed_message() {
        let current_time = 10;
        let mut message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            current_time, 
            MessageType::SetTask(Task::Undefined)
        );
        message.process();

        assert!(
            matches!(
                try_preprocess_message(&mut message, current_time), 
                Err(MessagePreprocessError::AlreadyPreprocessed)
            )
        );
    }

    #[test]
    fn too_early_message_preprocessing() {
        let current_time = 12;
        let mut message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            50, 
            MessageType::SetTask(Task::Undefined)
        );

        assert!(
            matches!(
                try_preprocess_message(&mut message, current_time), 
                Err(MessagePreprocessError::TooEarly)
            )
        );
    }
    
    #[test]
    fn correct_message_preprocessing() {
        let current_time = 12;
        let mut message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            current_time, 
            MessageType::SetTask(Task::Undefined)
        );

        assert!(
            try_preprocess_message(&mut message, current_time)
                .is_ok()
        );
    }

    #[test]
    fn no_duplicates_when_sending_gps_messages() {
        let not_important_time = 0;
        let frequency          = GPS_L1_FREQUENCY;

        let devices = [
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, 7.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, -7.0, 0.0)),
        ];
        let device1_id = devices[1].id();
        let device2_id = devices[2].id();
        let device_map = IdToDeviceMap::from(devices);

        let mut message_queue = MessageQueue::new();

        let gps_message_for_device1 = default_gps_message_for(
            device1_id,
            not_important_time
        );
        let gps_message_for_device2 = default_gps_message_for(
            device2_id,
            not_important_time
        );

        message_queue.add_message(frequency, gps_message_for_device1);
        message_queue.add_message(frequency, gps_message_for_device2);

        assert_eq!(message_queue.len(), 2);

        send_gps_messages(&device_map, &mut message_queue, not_important_time);
        
        assert_eq!(message_queue.len(), 4);
        assert!(
            message_queue_contains(&message_queue, &gps_message_for_device1)
        );
        assert!(
            message_queue_contains(&message_queue, &gps_message_for_device2)
        );
    }
}
