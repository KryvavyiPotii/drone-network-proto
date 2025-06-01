use crate::backend::device::{Device, DeviceId, IdToDeviceMap};
use crate::backend::mathphysics::{Megahertz, Millisecond, Position};
use crate::backend::message::{Message, MessageType, MessageQueue};
use crate::backend::signal::GREEN_SIGNAL_LEVEL;


fn message_queue_already_contains_gps_message_for(
    message_queue: &MessageQueue, 
    device_id: DeviceId
) -> bool {
    message_queue
        .iter()
        .any(|(_, message, _)| 
            message.destination_id() == device_id && message.is_gps()
        )
}


#[derive(Clone, Debug, Default)]
pub struct GPS {
    device: Device,
    frequency: Megahertz
}

impl GPS {
    #[must_use]
    pub fn new(device: Device, frequency: Megahertz) -> Self {
        Self { device, frequency }
    }
    
    #[must_use]
    pub fn device(&self) -> &Device {
        &self.device
    }

    pub fn connect_gps_to_all_devices(&self, device_map: &mut IdToDeviceMap) {
        device_map.set_rx_signal_level(
            // TODO make dependant on `self.device` 
            &GREEN_SIGNAL_LEVEL,
            self.frequency, 
        );
    }

    pub fn send_gps_messages(
        &self,
        device_map: &IdToDeviceMap,
        message_queue: &mut MessageQueue,
        current_time: Millisecond,
    ) {
        for (device_id, device) in device_map {
            // TODO check if device is within reach 
            if message_queue_already_contains_gps_message_for(
                message_queue, 
                *device_id
            ) {
                continue; 
            }

            let gps_message = Message::new(
                self.device.id(), 
                *device_id,
                current_time, 
                MessageType::GPS(*device.position())
            );

            message_queue.add_message(gps_message, self.frequency);
        }
    }
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::backend::device::{Device, DeviceBuilder, SignalLossResponse};
    use crate::backend::device::systems::{PowerSystem, TRXModule, TRXSystem};
    use crate::backend::mathphysics::{Meter, Point3D, PowerUnit};
    use crate::backend::message::Message;
    use crate::backend::signal::{
        SignalArea, SignalLevel, GPS_L1_FREQUENCY, GREEN_SIGNAL_STRENGTH_VALUE, 
        NO_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY
    };

    use super::*;

    
    const DEFAULT_GPS_POSITION_IN_METERS: Point3D = Point3D { 
        x: 0.0, 
        y: 0.0, 
        z: GPS_TX_RADIUS / 2.0
    };
    const DEVICE_MAX_POWER: PowerUnit    = 1_000;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const GPS_MAX_POWER: PowerUnit       = 10_000;
    const GPS_TX_RADIUS: Meter           = 1_000.0;
    const VERY_BIG_STRENGTH_VALUE: f32   = GREEN_SIGNAL_STRENGTH_VALUE * 1000.0;


    fn default_gps_message_for(
        gps_id: DeviceId,
        device_id: DeviceId,
        execution_time: Millisecond
    ) -> Message {
        Message::new(
            gps_id, 
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
        let power_system = PowerSystem::build(
            GPS_MAX_POWER, 
            GPS_MAX_POWER
        ).unwrap();
        let trx_system = TRXSystem::Strength { 
            tx_module: gps_tx_module(),
            rx_module: drone_rx_module()
        };

        let device = DeviceBuilder::new()
            .set_real_position(DEFAULT_GPS_POSITION_IN_METERS)
            .set_signal_loss_response(SignalLossResponse::Ignore)
            .set_power_system(power_system)
            .set_trx_system(trx_system)
            .build();

        GPS::new(device, GPS_L1_FREQUENCY)
    }


    #[test]
    fn no_duplicates_when_sending_gps_messages() {
        let unimportant_time = 0;

        let devices = [
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, 7.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, -7.0, 0.0)),
        ];
        let device1_id = devices[1].id();
        let device2_id = devices[2].id();
        let device_map = IdToDeviceMap::from(devices);
        let gps = default_gps();

        let mut message_queue = MessageQueue::new();

        let gps_message_for_device1 = default_gps_message_for(
            gps.device.id(),
            device1_id,
            unimportant_time
        );
        let gps_message_for_device2 = default_gps_message_for(
            gps.device.id(),
            device2_id,
            unimportant_time
        );

        message_queue.add_message(gps_message_for_device1, gps.frequency);
        message_queue.add_message(gps_message_for_device2, gps.frequency);

        assert_eq!(message_queue.len(), 2);
        
        gps.send_gps_messages(
            &device_map, 
            &mut message_queue, 
            unimportant_time,
        );
        
        assert_eq!(message_queue.len(), 4);
        assert!(
            message_queue_contains(&message_queue, &gps_message_for_device1)
        );
        assert!(
            message_queue_contains(&message_queue, &gps_message_for_device2)
        );
    }
}
