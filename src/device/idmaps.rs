use std::collections::HashMap;
use std::collections::hash_map::{Iter, IterMut, Keys, Values, ValuesMut};

use crate::device::{
    CommandCenter, Device, DeviceId, Drone, Receiver, Transceiver 
};
use crate::mathphysics::Megahertz;
use crate::signal::{SignalLevel, NO_SIGNAL_LEVEL};


pub type IdToLevelMap = HashMap<DeviceId, SignalLevel>;


#[derive(Clone, Debug)]
pub struct IdToDroneMap(HashMap<DeviceId, Drone>);

impl IdToDroneMap {
    #[must_use]
    pub fn get(&self, drone_id: &DeviceId) -> Option<&Drone> {
        self.0.get(drone_id)
    }
    
    #[must_use]
    pub fn get_mut(&mut self, drone_id: &DeviceId) -> Option<&mut Drone> {
        self.0.get_mut(drone_id)
    }

    #[must_use]
    pub fn ids(&self) -> Keys<'_, DeviceId, Drone> {
        self.0.keys()
    }

    #[must_use]
    pub fn drones(&self) -> Values<'_, DeviceId, Drone> {
        self.0.values()
    }
    
    #[must_use]
    pub fn drones_mut(&mut self) -> ValuesMut<'_, DeviceId, Drone> {
        self.0.values_mut()
    }

    #[must_use]
    pub fn iter(&self) -> Iter<'_, DeviceId, Drone> {
        self.0.iter()
    }

    pub fn iter_mut(&mut self) -> IterMut<'_, DeviceId, Drone> {
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
    pub fn all_rx_signal_levels(
        &self, 
        frequency: Megahertz
    ) -> IdToLevelMap {
        self.0
            .values()
            .map(|drone| (
                drone.id(),
                *drone.rx_signal_level(frequency)
            ))
            .collect()
    }

    pub fn connect_command_center(&mut self, command_center: &CommandCenter) {
        self.0
            .values_mut()
            .for_each(|drone| 
                drone.connect_command_center(command_center)
            );
    }

    pub fn update_states(&mut self) {
        self.0
            .values_mut()
            .for_each(Drone::update_state);
    }

    pub fn handle_infection(&mut self) {
        self.0
            .values_mut()
            .for_each(Drone::handle_infection);
    }

    pub fn remove_uncontrolled_drones(&mut self, frequency: Megahertz) { 
        self.0.retain(|_, drone| drone.receives_signal(frequency));
    }
    
    pub fn set_tx_signal_level(
        &mut self,
        frequency: Megahertz,
        signal_level: &SignalLevel,
    ) {
        self.0
            .values_mut()
            .for_each(|drone| 
                drone.set_tx_signal_level(frequency, *signal_level)
            );
    }
    
    pub fn set_tx_signal_levels(
        &mut self,
        signal_levels: &IdToLevelMap,
        frequency: Megahertz
    ) {
        for (id, drone) in &mut self.0 {
            let Some(signal_level) = signal_levels.get(id) else {
                continue
            };

            drone.set_tx_signal_level(frequency, *signal_level);
        }
    }
    
    pub fn set_rx_signal_level(
        &mut self,
        frequency: Megahertz,
        signal_level: &SignalLevel,
    ) {
        self.0
            .values_mut()
            .for_each(|drone| 
                drone.set_rx_signal_level(frequency, *signal_level)
            );
    }

    pub fn set_rx_signal_levels(
        &mut self,
        signal_levels: &IdToLevelMap,
        frequency: Megahertz
    ) {
        for (id, drone) in &mut self.0 {
            let Some(signal_level) = signal_levels.get(id) else {
                continue
            };

            drone.set_rx_signal_level(frequency, *signal_level);
        }
    }
    
    pub fn all_receive_signals(
        &mut self, 
        signal_levels: &IdToLevelMap,
        frequency: Megahertz
    ) {
        for (id, drone) in &mut self.0 {
            let Some(signal_level) = signal_levels.get(id) else {
                continue
            };

            drone.receive_signal(frequency, *signal_level);
        }
    }

    pub fn clear_rx_signal_levels(&mut self, frequency: Megahertz) {
        self.0
            .values_mut()
            .for_each(|drone|
                drone.set_rx_signal_level(frequency, NO_SIGNAL_LEVEL)
            );
    }

    pub fn all_rx_signals_to_tx(&mut self, frequency: Megahertz) {
        let rx_signal_levels = self.all_rx_signal_levels(frequency);

        self.set_tx_signal_levels(&rx_signal_levels, frequency);
    }
}

impl<'a> IntoIterator for &'a IdToDroneMap{
    type Item = (&'a DeviceId, &'a Drone);
    type IntoIter = Iter<'a, DeviceId, Drone>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter()
    }
}

impl<'a> IntoIterator for &'a mut IdToDroneMap{
    type Item = (&'a DeviceId, &'a mut Drone);
    type IntoIter = IterMut<'a, DeviceId, Drone>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter_mut()
    }
}

impl From<&[Drone]> for IdToDroneMap {
    fn from(drones: &[Drone]) -> Self {
       Self(
           drones 
                .iter()
                .map(|drone| (drone.id(), drone.clone()))
                .collect()
       )
    }
}

impl<const N: usize> From<[Drone; N]> for IdToDroneMap {
    fn from(drones: [Drone; N]) -> Self {
       Self(
           drones 
                .iter()
                .map(|drone| (drone.id(), drone.clone()))
                .collect()
       )
    }
}
