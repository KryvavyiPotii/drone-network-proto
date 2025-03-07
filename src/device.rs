use std::sync::atomic::{AtomicUsize, Ordering};

use crate::communication::{FreqToLevelMap, Message, SignalArea, SignalLevel};
use crate::mathphysics::{Megahertz, Meter, Millisecond, Position};

use self::modules::ReceiveMessageError;


pub use cc::*;
pub use drone::*;
pub use ewd::*;


pub mod cc;
pub mod drone;
pub mod ewd; 
pub mod modules;
pub mod networkmodel;


pub type DeviceId = usize;


pub const STEP_DURATION: Millisecond = 50;
const UNKNOWN_ID: DeviceId = 0;

static FREE_DEVICE_ID: AtomicUsize = AtomicUsize::new(1);


pub trait Device: Position {
    fn id(&self) -> DeviceId;
}

pub trait Transmitter: Device {
    fn signal_levels(&self) -> &FreqToLevelMap;
    fn signal_level(&self, frequency: Megahertz) -> &SignalLevel;
    fn set_signal_level(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    );
    fn area(&self, frequency: Megahertz) -> SignalArea;
    fn connection_distance<P: Position>(
        &self, 
        object: &P, 
        frequency: Megahertz
    ) -> Option<Meter>;
    fn propagated_signal_level_at<R: Receiver>(
        &self,
        receiver: &R,
        frequency: Megahertz
    ) -> SignalLevel;
    fn propagate_signal<R: Receiver>(
        &self,
        receiver: &mut R,
        frequency: Megahertz
    );
}

pub trait Suppressor: Transmitter {
    fn suppress_signal<R: Receiver>(
        &self,
        receiver: &mut R,
        frequency: Megahertz
    );
}

pub trait Receiver: Device {
    fn signal_levels(&self) -> &FreqToLevelMap;
    fn signal_level(&self, frequency: Megahertz) -> &SignalLevel;
    fn set_signal_level(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    );
    fn receives_signal(&self, frequency: Megahertz) -> bool;
    fn receive_signal(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    );
    /// # Errors
    ///
    /// Will return `Err` if message is not received. 
    fn receive_message(
        &mut self,
        frequency: Megahertz,
        message: &Message
    ) -> Result<(), ReceiveMessageError>;
    fn signal_level_suppression(
        &mut self,
        suppressor_frequency: Megahertz,
        suppressor_signal_level: SignalLevel
    );
}

// Trait created to avoid overuse of fully qualified syntax for 
// disambiguation.
pub trait Transceiver: Transmitter + Receiver {
    fn tx_signal_levels(&self) -> &FreqToLevelMap {
        Transmitter::signal_levels(self)
    }
    
    fn tx_signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        Transmitter::signal_level(self, frequency)
    }
    
    fn set_tx_signal_level(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    ) {
        Transmitter::set_signal_level(self, frequency, signal_level);
    }
    
    fn rx_signal_levels(&self) -> &FreqToLevelMap {
        Receiver::signal_levels(self)
    }
    
    fn rx_signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        Receiver::signal_level(self, frequency)
    }
    
    fn set_rx_signal_level(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    ) {
        Receiver::set_signal_level(self, frequency, signal_level);
    }
}


fn generate_device_id() -> DeviceId {
    FREE_DEVICE_ID.fetch_add(1, Ordering::SeqCst)
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::communication::{
        BLACK_SIGNAL_LEVEL, GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, 
        NO_SIGNAL_LEVEL, RED_SIGNAL_LEVEL, SignalArea, WIFI_2_4GHZ_FREQUENCY
    };
    use crate::device::{
        CommandCenterBuilder, DroneBuilder,
        modules::{TRXModule, TRXSystem}
    };
    use crate::mathphysics::{Meter, Point3D};
    
    use super::*;
    

    // These constants were introduced for testing independently from the global
    // constants.
    const CC_TX_CONTROL_RADIUS: Meter    = 300.0;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const EWD_TX_CONTROL_RADIUS: Meter   = 100.0;
    const EWD_TX_GPS_RADIUS: Meter       = 50.0;
    

    fn cc_tx_module() -> TRXModule {
        let max_tx_signal_levels = HashMap::from([(
            WIFI_2_4GHZ_FREQUENCY, 
            SignalLevel::from_area(
                SignalArea::build(CC_TX_CONTROL_RADIUS).unwrap(), 
                WIFI_2_4GHZ_FREQUENCY
            )
        )]);

        TRXModule::build(
            max_tx_signal_levels.clone(), 
            max_tx_signal_levels
        ).unwrap()
    }

    fn drone_tx_module() -> TRXModule {
        let max_tx_signal_levels = HashMap::from([(
            WIFI_2_4GHZ_FREQUENCY, 
            SignalLevel::from_area(
                SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap(), 
                WIFI_2_4GHZ_FREQUENCY
            )
        )]);

        TRXModule::build(
            max_tx_signal_levels.clone(),
            max_tx_signal_levels
        ).unwrap()
    }
    
    fn drone_rx_module() -> TRXModule {
        let max_rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL),
            (WIFI_2_4GHZ_FREQUENCY, GREEN_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, RED_SIGNAL_LEVEL),
            (WIFI_2_4GHZ_FREQUENCY, NO_SIGNAL_LEVEL)
        ]);

        TRXModule::build(
            max_rx_signal_levels,
            rx_signal_levels
        ).unwrap()
    }
    
    fn ewd_signal_levels() -> FreqToLevelMap {
        HashMap::from([
            (
                WIFI_2_4GHZ_FREQUENCY, 
                SignalLevel::from_area(
                    SignalArea::build(EWD_TX_CONTROL_RADIUS).unwrap(), 
                    WIFI_2_4GHZ_FREQUENCY
                )
            ),
            (
                GPS_L1_FREQUENCY, 
                SignalLevel::from_area(
                    SignalArea::build(EWD_TX_GPS_RADIUS).unwrap(),
                    GPS_L1_FREQUENCY
                )
            )
        ])
    }


    #[test]
    fn unique_device_ids() {
        let command_center = CommandCenterBuilder::new().build();
        let electronic_warfare = ElectronicWarfareBuilder::new().build();
        let drone1 = DroneBuilder::new().build();
        let drone2 = DroneBuilder::new().build();
    
        assert_ne!(command_center.id(), electronic_warfare.id());
        assert_ne!(command_center.id(), drone1.id());
        assert_ne!(command_center.id(), drone2.id());
        assert_ne!(electronic_warfare.id(), drone1.id());
        assert_ne!(electronic_warfare.id(), drone2.id());
        assert_ne!(drone1.id(), drone2.id());
    }

    #[test]
    fn cc_connection_to_drone() {
        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Strength {
                    tx_module: cc_tx_module(),
                    rx_module: TRXModule::default()
                }
            )
            .build();
        let mut drone = DroneBuilder::new()
            .set_trx_system(
                TRXSystem::Strength {
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module()
                }
            )
            .build();

        assert!(drone.command_center_id() == UNKNOWN_ID);
        assert!(
            drone
                .rx_signal_level(WIFI_2_4GHZ_FREQUENCY)
                .is_black()
        );

        drone.connect_command_center(&command_center);
        command_center.propagate_signal(&mut drone, WIFI_2_4GHZ_FREQUENCY);
        
        assert!(drone.command_center_id() == command_center.id());
        assert!(
            drone
                .rx_signal_level(WIFI_2_4GHZ_FREQUENCY)
                .is_green()
        );
    }
    
    #[test]
    fn suppress_tranceivers_by_strength() {
        let control_frequency = WIFI_2_4GHZ_FREQUENCY;
        let gps_frequency = GPS_L1_FREQUENCY;
        
        let drone_trx_system = TRXSystem::Strength {
            tx_module: drone_tx_module(), 
            rx_module: drone_rx_module()
        };
        
        let ewd = ElectronicWarfareBuilder::new()
            .set_trx_system(
                TRXSystem::Strength {
                    tx_module: TRXModule::build(
                        ewd_signal_levels(),
                        ewd_signal_levels()
                    ).unwrap(),
                    rx_module: TRXModule::default()
                }
            )
            .build();
        let mut drone_inside = DroneBuilder::new()
            .set_trx_system(drone_trx_system.clone())
            .build();
        let mut drone_outside = DroneBuilder::new()
            .set_global_position(
                Point3D::new(
                    EWD_TX_CONTROL_RADIUS * 20.0,
                    0.0,
                    0.0
                )
            )
            .set_trx_system(drone_trx_system)
            .build();

        ewd.suppress_signal(&mut drone_inside, control_frequency);
        ewd.suppress_signal(&mut drone_inside, gps_frequency);
        ewd.suppress_signal(&mut drone_outside, control_frequency);
        ewd.suppress_signal(&mut drone_outside, gps_frequency);

        assert!(
            *drone_inside
                .tx_signal_level(control_frequency) < BLACK_SIGNAL_LEVEL
        );
        assert!(
            *drone_inside
                .rx_signal_level(gps_frequency) < BLACK_SIGNAL_LEVEL
        );
        assert!(
            *drone_outside
                .tx_signal_level(control_frequency) >= BLACK_SIGNAL_LEVEL
        );
        assert!(
            *drone_outside
                .rx_signal_level(gps_frequency) >= BLACK_SIGNAL_LEVEL
        );
    }

    #[test]
    fn drone_movement_without_gps() {
        let destination_point = Point3D::new(MAX_DRONE_SPEED, 0.0, 0.0);
        
        let mut drone_without_gps = DroneBuilder::new()
            .set_destination(destination_point)
            .build();

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            drone_without_gps.update_state();
        }

        assert_eq!(
            *drone_without_gps.position_without_gps(), 
            Point3D::default()
        );
        assert_eq!(
            *drone_without_gps.position(), 
            destination_point
        );
    }

    #[test]
    fn drone_reach_destination() {
        let destination_point = Point3D::new(MAX_DRONE_SPEED, 0.0, 0.0);
        
        let mut drone = DroneBuilder::new()
            .set_destination(destination_point)
            .build();

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            drone.update_state();
        }

        assert!(drone.reached_destination());
    }
}
