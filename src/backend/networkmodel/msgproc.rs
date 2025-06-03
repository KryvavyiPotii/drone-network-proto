use thiserror::Error;

use crate::backend::connections::ConnectionGraph;
use crate::backend::device::{
    Device, DeviceId, IdToDelayMap, IdToDeviceMap, IdToTaskMap, BROADCAST_ID
};
use crate::backend::mathphysics::{delay_to, Millisecond, Position};
use crate::backend::message::{Message, MessageType};


#[derive(Error, Debug)]
pub enum TaskAddError {
    #[error("Message is not being processed")]
    MessageNotInProgress,
    #[error("Message type is not setting a new task")]
    WrongMessageType,
}

#[derive(Error, Debug)]
pub enum MessageProcessError {
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


pub fn set_delay_map_for_message(
    message: &Message,
    delay_map: &mut IdToDelayMap,
    source_device: &Device,
    device_map: &IdToDeviceMap,
    connections: &ConnectionGraph,
    delay_multiplier: f32,
) {
    let source_id = source_device.id();

    let message_delay_map = if connections.contains_device(source_id) {
        delay_map_from_device_inside_network(
            message, 
            source_id, 
            connections,
            delay_multiplier
        )
    } else {
        delay_map_from_device_outside_network(
            message, 
            source_device, 
            device_map,
            connections,
            delay_multiplier
        )        
    };
    
    delay_map.clone_from(&message_delay_map);
}

fn delay_map_from_device_inside_network(
    message: &Message,
    source_id: DeviceId,
    connections: &ConnectionGraph,
    delay_multiplier: f32,
) -> IdToDelayMap {
    let destination_id = message.destination_id();

    if destination_id == BROADCAST_ID {
        return connections.delays(source_id, delay_multiplier);
    }

    let delay = connections.delay_to(
        source_id, 
        destination_id, 
        delay_multiplier
    );

    IdToDelayMap::from([(destination_id, delay)])
}

fn delay_map_from_device_outside_network(
    message: &Message,
    source_device: &Device,
    device_map: &IdToDeviceMap,
    connections: &ConnectionGraph,
    delay_multiplier: f32,
) -> IdToDelayMap {
    let destination_id = message.destination_id();

    if destination_id == BROADCAST_ID {
        return connections 
            .devices()
            .filter_map(|device_id| {
                let destination_device = device_map.get(&destination_id)?; 
                
                let delay = delay_to(
                    source_device.distance_to(destination_device), 
                    delay_multiplier
                );

                Some((device_id, delay))
            })
            .collect();
    } 

    let Some(destination_device) = device_map.get(&destination_id) else {
        return IdToDelayMap::new();
    };
    
    let delay = delay_to(
        source_device.distance_to(destination_device), 
        delay_multiplier
    );

    IdToDelayMap::from([(destination_id, delay)])
}

/// # Errors
///
/// Will return `Err` if a message is not being processed and its type is not
/// `SetTask`.
pub fn try_add_task(
    message: &Message,
    current_tasks: &mut IdToTaskMap,
    device_map: &IdToDeviceMap,
) -> Result<(), TaskAddError> {
    if !message.is_in_progress() {
        return Err(TaskAddError::MessageNotInProgress);
    }
    let MessageType::SetTask(task) = *message.message_type() else {
        return Err(TaskAddError::WrongMessageType);
    };

    let destination_id = message.destination_id();
    if destination_id != BROADCAST_ID {
        current_tasks.insert(destination_id, task);
        return Ok(());
    }

    for device_id in device_map.ids() {
        current_tasks.insert(*device_id, task);
    }    

    Ok(())
}

// We assume that the message processing is finished if it was processed by 
// the device with the longest delay.
/// # Errors
///
/// Will return `Err` if message execution time is greater than current time.
pub fn try_finish_message(
    message: &mut Message,
    delay_map: &IdToDelayMap,
    current_time: Millisecond,
) -> Result<(), MessageProcessError>{
    let longest_delay = *delay_map
        .values()
        .max()
        .unwrap_or(&0);

    if current_time < message.time() + longest_delay {
        return Err(MessageProcessError::TooEarly)
    }
    
    message.finish(); 

    Ok(())
}

/// # Errors
///
/// Will return `Err` if message execution time is greater than current time or 
/// message is already being processed or `ConnectionGraph` does not contain 
/// source device ID.
pub fn try_preprocess_message(
    message: &mut Message,
    current_time: Millisecond,
) -> Result<(), MessageProcessError> {
    if message.is_in_progress() {
        return Err(MessageProcessError::AlreadyPreprocessed);
    }
    if current_time < message.time() {
        return Err(MessageProcessError::TooEarly);
    }

    message.process();

    Ok(())
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::backend::device::{Device, DeviceBuilder, BROADCAST_ID};
    use crate::backend::device::systems::{PowerSystem, TRXModule, TRXSystem};
    use crate::backend::malware::{Malware, MalwareType};
    use crate::backend::mathphysics::{Meter, Point3D, PowerUnit};
    use crate::backend::message::{Task, Message};
    use crate::backend::signal::{
        SignalArea, SignalLevel, NO_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE, 
        WIFI_2_4GHZ_FREQUENCY
    };

    use super::*;

    
    const DEVICE_MAX_POWER: PowerUnit    = 1_000;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const UNKNOWN_ID: DeviceId           = 0;
    const VERY_BIG_STRENGTH_VALUE: f32   = GREEN_SIGNAL_STRENGTH_VALUE * 1000.0;


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
    fn adding_task_from_message_not_in_progress() {
        let current_time = 0;
        let mut no_tasks = HashMap::new();
        let no_devices_map = IdToDeviceMap::new();

        let waiting_message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            current_time, 
            MessageType::SetTask(Task::Undefined)
        );

        assert!(
            matches!(
                try_add_task(&waiting_message, &mut no_tasks, &no_devices_map), 
                Err(TaskAddError::MessageNotInProgress)
            )
        );

        let mut finished_message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            current_time, 
            MessageType::SetTask(Task::Undefined)
        );
        finished_message.finish();
        
        assert!(
            matches!(
                try_add_task(&finished_message, &mut no_tasks, &no_devices_map), 
                Err(TaskAddError::MessageNotInProgress)
            )
        );
    }
    
    #[test]
    fn adding_task_from_message_with_wrong_type() {
        let current_time = 0;
        let mut no_tasks = HashMap::new();
        let no_devices_map = IdToDeviceMap::new();

        let mut gps_message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            current_time, 
            MessageType::GPS(Point3D::default())
        );
        gps_message.process();

        assert!(
            matches!(
                try_add_task(&gps_message, &mut no_tasks, &no_devices_map), 
                Err(TaskAddError::WrongMessageType)
            )
        );

        let malware = Malware::new(0, MalwareType::Indicator, false);
        let mut malicious_message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            current_time, 
            MessageType::Malware(malware)
        );
        malicious_message.process();
        
        assert!(
            matches!(
                try_add_task(
                    &malicious_message, 
                    &mut no_tasks, 
                    &no_devices_map
                ), 
                Err(TaskAddError::WrongMessageType)
            )
        );
    }

    #[test]
    fn adding_task_for_single_device() {
        let current_time = 0;
        let mut current_tasks = HashMap::new();

        let devices = [
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
        ];
        let device1_id = devices[0].id();
        let device2_id = devices[1].id();
        let device_map = IdToDeviceMap::from(devices);

        let task = Task::Reposition(Point3D::default());
        let mut message = Message::new(
            UNKNOWN_ID,
            device1_id,
            current_time, 
            MessageType::SetTask(task)
        );
        message.process();

        assert!(
            try_add_task(&message, &mut current_tasks, &device_map)
                .is_ok()
        );
        assert_eq!(*current_tasks.get(&device1_id).unwrap(), task); 
        assert!(!current_tasks.contains_key(&device2_id));
    }

    #[test]
    fn adding_task_for_all_devices() {
        let current_time = 0;
        let mut current_tasks = HashMap::new();

        let devices = [
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
        ];
        let device1_id = devices[0].id();
        let device2_id = devices[1].id();
        let device_map = IdToDeviceMap::from(devices);

        let task = Task::Reposition(Point3D::default());
        let mut message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            current_time, 
            MessageType::SetTask(task)
        );
        message.process();

        assert!(
            try_add_task(&message, &mut current_tasks, &device_map)
                .is_ok()
        );
        assert_eq!(*current_tasks.get(&device1_id).unwrap(), task); 
        assert_eq!(*current_tasks.get(&device2_id).unwrap(), task); 
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
                Err(MessageProcessError::AlreadyPreprocessed)
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
                Err(MessageProcessError::TooEarly)
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
}
