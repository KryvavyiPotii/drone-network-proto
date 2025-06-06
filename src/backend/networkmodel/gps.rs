use crate::backend::device::{Device, IdToDeviceMap};
use crate::backend::mathphysics::{Megahertz, Millisecond, Position};
use crate::backend::message::{Message, MessageType, MessageQueue};


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
        for device in device_map.devices_mut() {
            self.device.propagate_signal(device, self.frequency);
        }
    }

    pub fn add_gps_messages_to_queue(
        &self,
        message_queue: &mut MessageQueue,
        device_map: &IdToDeviceMap,
        current_time: Millisecond,
    ) {
        for (device_id, device) in device_map {
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
