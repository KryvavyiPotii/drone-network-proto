use std::hash::{Hash, Hasher};

use crate::device::{
    Device, DeviceId, Receiver, Transceiver, Transmitter, generate_device_id,
};
use crate::device::systems::{ReceiveMessageError, TRXSystem};
use crate::mathphysics::{Megahertz, Meter, Point3D, Position};
use crate::message::Message;
use crate::signal::{FreqToLevelMap, SignalArea, SignalLevel};


pub struct CommandCenterBuilder {
    id: DeviceId,
    position: Option<Point3D>,
    trx_system: Option<TRXSystem>
}

impl CommandCenterBuilder {
    #[must_use]
    pub fn new() -> Self {
        Self {
            id: generate_device_id(),
            position: None,
            trx_system: None
        }
    }
    
    #[must_use]
    pub fn set_position(mut self, position_in_meters: Point3D) -> Self {
        self.position = Some(position_in_meters);
        self
    }

    #[must_use]
    pub fn set_trx_system(mut self, trx_system: TRXSystem) -> Self {
        self.trx_system = Some(trx_system);
        self
    }

    #[must_use]
    pub fn build(self) -> CommandCenter {
        CommandCenter::new(
            self.id,
            self.position.unwrap_or_default(),
            self.trx_system.unwrap_or_default()
        )
    }
}

impl Default for CommandCenterBuilder {
    fn default() -> Self {
        Self::new()
    }
}


#[derive(Clone)]
pub struct CommandCenter {
    id: DeviceId,
    position_in_meters: Point3D,
    trx_system: TRXSystem
}

impl CommandCenter {
    #[must_use]
    pub fn new(
        id: DeviceId, 
        position_in_meters: Point3D,
        trx_system: TRXSystem
    ) -> Self {
        Self {
            id,
            position_in_meters,
            trx_system
        }
    }
}

impl Default for CommandCenter {
    fn default() -> Self {
        Self {
            id: generate_device_id(),
            position_in_meters: Point3D::default(),
            trx_system: TRXSystem::default()
        }
    }
}

impl PartialEq for CommandCenter {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Hash for CommandCenter {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Position for CommandCenter {
    fn position(&self) -> &Point3D {
        &self.position_in_meters
    }
}

impl Device for CommandCenter {
    fn id(&self) -> DeviceId {
        self.id
    }
}

impl Transmitter for CommandCenter {
    fn signal_levels(&self) -> &FreqToLevelMap {
        self.trx_system.tx_signal_levels()
    }
    
    fn signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        self.trx_system.tx_signal_level(frequency)
    }
    
    fn set_signal_level(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    ) {
        self.trx_system.set_tx_signal_level(frequency, signal_level);
    }

    fn area(&self, frequency: Megahertz) -> SignalArea {
        self.trx_system.area(frequency)
    }

    fn connection_distance<P: Position>(
        &self, 
        object: &P, 
        frequency: Megahertz
    ) -> Option<Meter> {
        self.trx_system.connection_distance(
            self.distance_to(object), 
            frequency
        )
    }
    
    fn propagated_signal_level_at<R: Receiver>(
        &self,
        receiver: &R,
        frequency: Megahertz
    ) -> SignalLevel {
        let distance_to_rx = self.distance_to(receiver);

        self.trx_system.tx_signal_level_at(frequency, distance_to_rx)
    }

    fn propagate_signal<R: Receiver>(
        &self,
        receiver: &mut R,
        frequency: Megahertz
    ) {
        let propagated_signal_level_at_rx = self.propagated_signal_level_at(
            receiver, 
            frequency
        );

        receiver.receive_signal(frequency, propagated_signal_level_at_rx);
    }
}

impl Receiver for CommandCenter {
    fn signal_levels(&self) -> &FreqToLevelMap {
        self.trx_system.rx_signal_levels()
    }
    
    fn signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        self.trx_system.rx_signal_level(frequency)
    }
    
    fn set_signal_level(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    ) {
        self.trx_system.set_rx_signal_level(frequency, signal_level);
    }

    fn receives_signal(&self, frequency: Megahertz) -> bool {
        self.trx_system.receives_signal(frequency)
    }
    
    fn receive_signal(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    ) {
        self.trx_system.receive_signal(frequency, signal_level); 
    }

    fn receive_message(
        &mut self,
        frequency: Megahertz,
        message: &Message
    ) -> Result<(), ReceiveMessageError> {
        self.trx_system.receive_message(frequency, message) 
    }

    fn signal_level_suppression(
        &mut self,
        suppressor_frequency: Megahertz,
        suppressor_signal_level: SignalLevel
    ) {
        self.trx_system.suppress_signal(
            suppressor_frequency, 
            suppressor_signal_level
        );
    }
}

impl Transceiver for CommandCenter {}
