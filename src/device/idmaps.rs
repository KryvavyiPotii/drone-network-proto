use std::collections::HashMap;
use std::collections::hash_map::{Iter, IterMut, Keys, Values, ValuesMut};

use crate::device::{Device, DeviceId};
use crate::mathphysics::Megahertz;
use crate::message::Goal;
use crate::signal::{SignalLevel, NO_SIGNAL_LEVEL};


pub type IdToLevelMap = HashMap<DeviceId, SignalLevel>;
pub type IdToGoalMap  = HashMap<DeviceId, Goal>;


#[derive(Clone, Debug)]
pub struct IdToDeviceMap(HashMap<DeviceId, Device>);

impl IdToDeviceMap {
    #[must_use]
    pub fn get(&self, device_id: &DeviceId) -> Option<&Device> {
        self.0.get(device_id)
    }
    
    #[must_use]
    pub fn get_mut(&mut self, device_id: &DeviceId) -> Option<&mut Device> {
        self.0.get_mut(device_id)
    }

    #[must_use]
    pub fn ids(&self) -> Keys<'_, DeviceId, Device> {
        self.0.keys()
    }

    #[must_use]
    pub fn devices(&self) -> Values<'_, DeviceId, Device> {
        self.0.values()
    }
    
    #[must_use]
    pub fn devices_mut(&mut self) -> ValuesMut<'_, DeviceId, Device> {
        self.0.values_mut()
    }

    #[must_use]
    pub fn iter(&self) -> Iter<'_, DeviceId, Device> {
        self.0.iter()
    }

    pub fn iter_mut(&mut self) -> IterMut<'_, DeviceId, Device> {
        self.0.iter_mut()
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }
    
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    #[must_use]
    pub fn all_rx_signal_levels(&self, frequency: Megahertz) -> IdToLevelMap {
        self.0
            .values()
            .map(|device| (device.id(), *device.rx_signal_level(frequency)))
            .collect()
    }

    #[must_use]
    pub fn goals(&self) -> IdToGoalMap {
        self.0
            .iter()
            .map(|(device_id, device)| (*device_id, *device.goal()))
            .collect()
    }

    #[must_use]
    pub fn remove(&mut self, device_id: &DeviceId) -> Option<Device> {
        self.0.remove(device_id)
    }

    pub fn update_states(&mut self) {
        self.0
            .values_mut()
            .for_each(Device::update_state);
    }

    pub fn handle_infection(&mut self) {
        self.0
            .values_mut()
            .for_each(Device::handle_infection);
    }

    pub fn remove_not_receiving_devices(
        &mut self, 
        command_device_id: &DeviceId, 
        frequency: Megahertz
    ) { 
        // Command device must not be accidentally deleted even if it does not
        // receive control frequency.
        self.0.retain(|device_id, device| 
            device.receives_signal(frequency) || device_id == command_device_id 
        );
    }
    
    pub fn set_tx_signal_level(
        &mut self,
        signal_level: &SignalLevel,
        frequency: Megahertz,
    ) {
        self.0
            .values_mut()
            .for_each(|device| 
                device.set_tx_signal_level(*signal_level, frequency)
            );
    }
    
    pub fn set_tx_signal_levels(
        &mut self,
        signal_levels: &IdToLevelMap,
        frequency: Megahertz
    ) {
        for (id, device) in &mut self.0 {
            let Some(signal_level) = signal_levels.get(id) else {
                continue
            };

            device.set_tx_signal_level(*signal_level, frequency);
        }
    }
    
    pub fn set_rx_signal_level(
        &mut self,
        signal_level: &SignalLevel,
        frequency: Megahertz,
    ) {
        self.0
            .values_mut()
            .for_each(|device| 
                device.set_rx_signal_level(*signal_level, frequency)
            );
    }

    pub fn set_rx_signal_levels(
        &mut self,
        signal_levels: &IdToLevelMap,
        frequency: Megahertz
    ) {
        for (id, device) in &mut self.0 {
            let Some(signal_level) = signal_levels.get(id) else {
                continue
            };

            device.set_rx_signal_level(*signal_level, frequency);
        }
    }
    
    pub fn all_receive_signal_levels(
        &mut self, 
        signal_levels: &IdToLevelMap,
        frequency: Megahertz
    ) {
        for (id, device) in &mut self.0 {
            let Some(signal_level) = signal_levels.get(id) else {
                continue
            };

            device.receive_signal(*signal_level, frequency);
        }
    }

    pub fn clear_rx_signal_levels(&mut self, frequency: Megahertz) {
        self.0
            .values_mut()
            .for_each(|device|
                device.set_rx_signal_level(NO_SIGNAL_LEVEL, frequency)
            );
    }

    pub fn copy_all_rx_signal_levels_to_tx(
        &mut self,
        command_device_id: &DeviceId,
        frequency: Megahertz
    ) {
        let mut rx_signal_levels = self.all_rx_signal_levels(frequency);

        // TX signal level of command device should not be modified in this
        // manner.
        rx_signal_levels.remove(command_device_id);

        self.set_tx_signal_levels(&rx_signal_levels, frequency);
    }
}

impl<'a> IntoIterator for &'a IdToDeviceMap{
    type Item = (&'a DeviceId, &'a Device);
    type IntoIter = Iter<'a, DeviceId, Device>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter()
    }
}

impl<'a> IntoIterator for &'a mut IdToDeviceMap{
    type Item = (&'a DeviceId, &'a mut Device);
    type IntoIter = IterMut<'a, DeviceId, Device>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter_mut()
    }
}

impl From<&[Device]> for IdToDeviceMap {
    fn from(devices: &[Device]) -> Self {
        let hash_map = devices 
            .iter()
            .map(|device| (device.id(), device.clone()))
            .collect();

        Self(hash_map)
    }
}

impl<const N: usize> From<[Device; N]> for IdToDeviceMap {
    fn from(devices: [Device; N]) -> Self {
        let hash_map = devices 
            .iter()
            .map(|device| (device.id(), device.clone()))
            .collect();
        
        Self(hash_map)
    }
}


#[cfg(test)]
mod tests {
    use crate::device::{DeviceBuilder, TRXSystem};
    use crate::device::systems::TRXModule;
    use crate::signal::{GREEN_SIGNAL_LEVEL, RED_SIGNAL_LEVEL};

    use super::*;

    #[test]
    fn removing_not_receiving_devices_and_leaving_command_device() {
        let frequency = 500;
        let trx_system = TRXSystem::Strength { 
            tx_module: TRXModule::default(), 
            rx_module: TRXModule::build(
                HashMap::from([(frequency, GREEN_SIGNAL_LEVEL)]),
                HashMap::from([(frequency, GREEN_SIGNAL_LEVEL)]),
            ).unwrap_or_else(|error| panic!("{}", error)) 
        };

        let command_device = DeviceBuilder::new()
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let not_receiving_device = DeviceBuilder::new()
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let receiving_device = DeviceBuilder::new()
            .set_trx_system(trx_system)
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        
        let command_device_id = command_device.id();
        let not_receiving_device_id = not_receiving_device.id();
        let receiving_device_id = receiving_device.id();

        let mut device_map = IdToDeviceMap::from(
            [command_device, not_receiving_device, receiving_device]
        );

        assert_eq!(device_map.len(), 3);
        assert!(device_map.get(&command_device_id).is_some());
        assert!(device_map.get(&not_receiving_device_id).is_some());
        assert!(device_map.get(&receiving_device_id).is_some());

        device_map.remove_not_receiving_devices(&command_device_id, frequency);

        assert_eq!(device_map.len(), 2);
        assert!(device_map.get(&command_device_id).is_some());
        assert!(device_map.get(&not_receiving_device_id).is_none());
        assert!(device_map.get(&receiving_device_id).is_some());
    }

    #[test]
    fn setting_signal_levels() {
        let frequency1 = 500;
        let frequency2 = 1_500;
        let unknown_frequency = 2_000;

        let trx_module = TRXModule::build(
            HashMap::from([
                (frequency1, GREEN_SIGNAL_LEVEL),
                (frequency2, GREEN_SIGNAL_LEVEL),
            ]),
            HashMap::from([
                (frequency1, GREEN_SIGNAL_LEVEL),
                (frequency2, GREEN_SIGNAL_LEVEL),
            ]),
        ).unwrap_or_else(|error| panic!("{}", error));
        let trx_system = TRXSystem::Strength { 
            tx_module: trx_module.clone(), 
            rx_module: trx_module 
        };
        
        let device1 = DeviceBuilder::new()
            .set_trx_system(trx_system.clone())
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let device2 = DeviceBuilder::new()
            .set_trx_system(trx_system)
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        
        let device1_id = device1.id();
        let device2_id = device2.id();
        let unknown_device_id = device1_id + device2_id;

        let mut device_map = IdToDeviceMap::from([device1, device2]);

        assert_eq!(device_map.len(), 2);

        let device1_ref = device_map
            .get(&device1_id)
            .expect("Missing device");
        let device2_ref = device_map
            .get(&device2_id)
            .expect("Missing device");

        assert!(
            device1_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            device1_ref
                .rx_signal_level(frequency2)
                .is_green()
        );
        assert!(
            device2_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            device2_ref
                .rx_signal_level(frequency2)
                .is_green()
        );

        let new_rx_signal_levels = HashMap::from([
            (device2_id, RED_SIGNAL_LEVEL),
            (unknown_device_id, GREEN_SIGNAL_LEVEL)
        ]);
        
        // `unknown_device_id` must be skipped.
        device_map.set_rx_signal_levels(
            &new_rx_signal_levels, 
            unknown_frequency
        );

        let device1_ref = device_map
            .get(&device1_id)
            .expect("Missing device");
        let device2_ref = device_map
            .get(&device2_id)
            .expect("Missing device");

        // No changes must happen.
        assert!(
            device1_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            device1_ref
                .rx_signal_level(frequency2)
                .is_green()
        );
        assert!(
            device2_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            device2_ref
                .rx_signal_level(frequency2)
                .is_green()
        );

        device_map.set_rx_signal_levels(&new_rx_signal_levels, frequency2);

        let device1_ref = device_map
            .get(&device1_id)
            .expect("Missing device");
        let device2_ref = device_map
            .get(&device2_id)
            .expect("Missing device");

        // Only the rx signal level of `device2` on `frequency2` must be 
        // changed.
        assert!(
            device1_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            device1_ref
                .rx_signal_level(frequency2)
                .is_green()
        );
        assert!(
            device2_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            device2_ref
                .rx_signal_level(frequency2)
                .is_red()
        );
        
        let new_tx_signal_levels = HashMap::from([
            (device1_id, RED_SIGNAL_LEVEL),
            (device2_id, RED_SIGNAL_LEVEL)
        ]);

        device_map.set_tx_signal_levels(&new_tx_signal_levels, frequency1);

        let device1_ref = device_map
            .get(&device1_id)
            .expect("Missing device");
        let device2_ref = device_map
            .get(&device2_id)
            .expect("Missing device");

        assert!(
            device1_ref
                .tx_signal_level(frequency1)
                .is_red()
        );
        assert!(
            device1_ref
                .tx_signal_level(frequency2)
                .is_green()
        );
        assert!(
            device2_ref
                .tx_signal_level(frequency1)
                .is_red()
        );
        assert!(
            device2_ref
                .tx_signal_level(frequency2)
                .is_green()
        );
    }

    #[test]
    fn all_drones_receive_signal_level() {
        let frequency1 = 500;
        let frequency2 = 1_500;
        let unknown_frequency = 2_000;

        let trx_module = TRXModule::build(
            HashMap::from([
                (frequency1, GREEN_SIGNAL_LEVEL),
                (frequency2, GREEN_SIGNAL_LEVEL),
            ]),
            HashMap::from([
                (frequency1, GREEN_SIGNAL_LEVEL),
                (frequency2, GREEN_SIGNAL_LEVEL),
            ]),
        ).unwrap_or_else(|error| panic!("{}", error));
        let trx_system = TRXSystem::Strength { 
            tx_module: trx_module.clone(), 
            rx_module: trx_module 
        };
        
        let rx_device1 = DeviceBuilder::new()
            .set_trx_system(trx_system.clone())
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let rx_device2 = DeviceBuilder::new()
            .set_trx_system(trx_system)
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        
        let rx_device1_id = rx_device1.id();
        let rx_device2_id = rx_device2.id();
        let unknown_rx_device_id = rx_device1_id + rx_device2_id;

        let mut device_map = IdToDeviceMap::from([rx_device1, rx_device2]);

        assert_eq!(device_map.len(), 2);

        let rx_device1_ref = device_map
            .get(&rx_device1_id)
            .expect("Missing device");
        let rx_device2_ref = device_map
            .get(&rx_device2_id)
            .expect("Missing device");

        assert!(
            rx_device1_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            rx_device1_ref
                .rx_signal_level(frequency2)
                .is_green()
        );
        assert!(
            rx_device2_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            rx_device2_ref
                .rx_signal_level(frequency2)
                .is_green()
        );

        let signal_levels_to_receive = HashMap::from([
            (rx_device2_id, RED_SIGNAL_LEVEL),
            (unknown_rx_device_id, GREEN_SIGNAL_LEVEL)
        ]);

        device_map.all_receive_signal_levels(
            &signal_levels_to_receive, 
            unknown_frequency
        );

        let rx_device1_ref = device_map
            .get(&rx_device1_id)
            .expect("Missing rx_device");
        let rx_device2_ref = device_map
            .get(&rx_device2_id)
            .expect("Missing rx_device");

        // No changes must happen.
        assert!(
            rx_device1_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            rx_device1_ref
                .rx_signal_level(frequency2)
                .is_green()
        );
        assert!(
            rx_device2_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            rx_device2_ref
                .rx_signal_level(frequency2)
                .is_green()
        );

        let initial_rx_device2_signal_level = rx_device2_ref
            .rx_signal_level(frequency1)
            .clone();
        
        // `unknown_rx_device_id` must be skipped.
        device_map.all_receive_signal_levels(
            &signal_levels_to_receive, 
            frequency1
        );

        let rx_device1_ref = device_map
            .get(&rx_device1_id)
            .expect("Missing rx_device");
        let rx_device2_ref = device_map
            .get(&rx_device2_id)
            .expect("Missing rx_device");

        // Only the rx signal level of `rx_device2` on `frequency1` must be 
        // changed.
        assert!(
            rx_device1_ref
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            rx_device1_ref
                .rx_signal_level(frequency2)
                .is_green()
        );
        assert_ne!(
            rx_device2_ref.rx_signal_level(frequency1),
            initial_rx_device2_signal_level
        );
        assert!(
            rx_device2_ref
                .rx_signal_level(frequency2)
                .is_green()
        );
    }

    #[test]
    fn clearing_rx_signal_levels() {
        let frequency1 = 500;
        let frequency2 = 1_500;

        let rx_module = TRXModule::build(
            HashMap::from([
                (frequency1, GREEN_SIGNAL_LEVEL),
                (frequency2, GREEN_SIGNAL_LEVEL),
            ]),
            HashMap::from([
                (frequency1, GREEN_SIGNAL_LEVEL),
                (frequency2, GREEN_SIGNAL_LEVEL),
            ]),
        ).unwrap_or_else(|error| panic!("{}", error));
        let trx_system = TRXSystem::Strength { 
            tx_module: TRXModule::default(), 
            rx_module
        };

        let device1 = DeviceBuilder::new()
            .set_trx_system(trx_system.clone())
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let device2 = DeviceBuilder::new()
            .set_trx_system(trx_system)
            .build()
            .unwrap_or_else(|error| panic!("{}", error));

        let device1_id = device1.id();
        let device2_id = device2.id();

        let mut device_map = IdToDeviceMap::from([device1, device2]);

        assert_eq!(device_map.len(), 2);
        assert!(
            device_map
                .get(&device1_id)
                .expect("Missing device")
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            device_map
                .get(&device1_id)
                .expect("Missing device")
                .rx_signal_level(frequency2)
                .is_green()
        );
        assert!(
            device_map
                .get(&device2_id)
                .expect("Missing device")
                .rx_signal_level(frequency1)
                .is_green()
        );
        assert!(
            device_map
                .get(&device2_id)
                .expect("Missing device")
                .rx_signal_level(frequency2)
                .is_green()
        );

        device_map.clear_rx_signal_levels(frequency1);
        
        assert!(
            device_map
                .get(&device1_id)
                .expect("Missing device")
                .rx_signal_level(frequency1)
                .is_black()
        );
        assert!(
            device_map
                .get(&device1_id)
                .expect("Missing device")
                .rx_signal_level(frequency2)
                .is_green()
        );
        assert!(
            device_map
                .get(&device2_id)
                .expect("Missing device")
                .rx_signal_level(frequency1)
                .is_black()
        );
        assert!(
            device_map
                .get(&device2_id)
                .expect("Missing device")
                .rx_signal_level(frequency2)
                .is_green()
        );
    }

    #[test]
    fn copying_rx_signal_levels_to_tx() {
        let frequency = 500;

        let empty_tx_module = TRXModule::build(
            HashMap::from([(frequency, GREEN_SIGNAL_LEVEL)]),
            HashMap::new()
        ).unwrap_or_else(|error| panic!("{}", error));
        let rx_module = TRXModule::build(
            HashMap::from([(frequency, GREEN_SIGNAL_LEVEL)]),
            HashMap::from([(frequency, GREEN_SIGNAL_LEVEL)]),
        ).unwrap_or_else(|error| panic!("{}", error));
        let trx_system = TRXSystem::Strength { 
            tx_module: empty_tx_module, 
            rx_module
        };

        let command_device = DeviceBuilder::new()
            .set_trx_system(trx_system.clone())
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let regular_device = DeviceBuilder::new()
            .set_trx_system(trx_system)
            .build()
            .unwrap_or_else(|error| panic!("{}", error));

        let command_device_id = command_device.id();
        let regular_device_id = regular_device.id();

        let mut device_map = IdToDeviceMap::from(
            [command_device, regular_device]
        );

        assert_eq!(device_map.len(), 2);
        
        device_map.copy_all_rx_signal_levels_to_tx(
            &command_device_id, 
            frequency
        );

        assert!(
            device_map
                .get(&command_device_id)
                .expect("Missing device")
                .tx_signal_level(frequency)
                .is_black()
        );
        assert!(
            device_map
                .get(&regular_device_id)
                .expect("Missing device")
                .tx_signal_level(frequency)
                .is_green()
        );
    }
}
