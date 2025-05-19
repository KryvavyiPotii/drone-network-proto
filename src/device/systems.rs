use thiserror::Error;

use crate::mathphysics::{
    Megahertz, Meter, MeterPerSecond, Point3D, PowerUnit, Vector3D
};
use crate::message::Message;
use crate::signal::{
    BLACK_SIGNAL_LEVEL, FreqToLevelMap, NO_SIGNAL_LEVEL, SignalArea, SignalLevel
};


pub use modules::*;


pub mod modules;


#[derive(Error, Debug)]
pub enum MovementSystemBuildError {
    #[error("Maximum speed is negative")]
    NegativeMaxSpeed
}

#[derive(Error, Debug)]
pub enum PowerSystemBuildError {
    #[error("Power is greater than max power")]
    PowerIsGreaterThanMax,
}

#[derive(Error, Debug)]
pub enum PowerSystemError {
    #[error("Provided power is greater than current power")]
    NotEnoughPower,
}

#[derive(Error, Debug)]
pub enum TRXSystemError {
    #[error("Message execution cost is too high")]
    TooExpensiveMessage,
    #[error("Message destination ID does not match device ID")]
    WrongMessageDestination,
}


// By default the system can supply any power, because its maximum power is 0.0.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PowerSystem {
    max_power: PowerUnit,
    power: PowerUnit,
}

impl PowerSystem {
    /// # Errors
    ///
    /// Will return `Err` if provided power is higher than provided max power.
    pub fn build(
        max_power: PowerUnit, 
        power: PowerUnit
    ) -> Result<Self, PowerSystemBuildError> {
        if power > max_power {
            return Err(PowerSystemBuildError::PowerIsGreaterThanMax);
        }

        Ok(Self { max_power, power })
    }

    #[must_use]
    pub fn max_power(&self) -> PowerUnit {
        self.max_power
    }

    #[must_use]
    pub fn power(&self) -> PowerUnit {
        self.power
    }

    pub fn set_power(&mut self, power: PowerUnit) {
        self.power = self.max_power.min(power);
    }

    /// # Errors
    ///
    /// Will return `Err` if device does not have enough power.
    pub fn try_consume_power(
        &mut self, 
        power_to_consume: PowerUnit
    ) -> Result<(), PowerSystemError> {
        let Some(power_left) = self.power.checked_sub(power_to_consume) else {
            return Err(PowerSystemError::NotEnoughPower)
        };

        self.power = power_left;

        Ok(())
    }
}


// By default the system can not move, because its maximum speed is 0.0.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MovementSystem {
    position_in_meters: Point3D,
    max_speed: MeterPerSecond,
    velocity_in_mps: Vector3D,
}

impl MovementSystem {
    /// # Errors
    ///
    /// Will return `Err` if `max_speed` is negative. 
    pub fn build(
        max_speed: MeterPerSecond
    ) -> Result<Self, MovementSystemBuildError> {
        if max_speed < 0.0 {
            return Err(MovementSystemBuildError::NegativeMaxSpeed);
        }

        Ok(
            Self {
                // Upon creation the system does not know its position.
                // The position should be provided by GPS (from TRXSystem).
                position_in_meters: Point3D::default(),
                max_speed,
                velocity_in_mps: Vector3D::default()
            }
        )
    }
    
    #[must_use]
    pub fn position(&self) -> &Point3D {
        &self.position_in_meters
    }
    
    #[must_use]
    pub fn max_speed(&self) -> MeterPerSecond {
        self.max_speed
    }

    #[must_use]
    pub fn velocity(&self) -> &Vector3D {
        &self.velocity_in_mps
    }

    #[must_use]
    pub fn is_disabled(&self) -> bool {
        self.max_speed == 0.0
    }
    
    pub fn set_position(&mut self, position: Point3D) {
        self.position_in_meters = position;
    }
    
    pub fn set_velocity(&mut self, velocity_in_mps: Vector3D) {
        if self.is_disabled() {
            return;
        }

        self.velocity_in_mps = velocity_in_mps;
        self.velocity_in_mps.truncate(self.max_speed);
    }
    
    pub fn update_movement_direction(&mut self, destination: Point3D) {
        if self.is_disabled() {
            return;
        }
        
        self.velocity_in_mps = Vector3D::new(
            self.position_in_meters,
            destination
        );
        
        self.velocity_in_mps.scale_to(self.max_speed);
    }
}


#[derive(Clone, Debug, PartialEq)]
pub enum TRXSystem {
    Color(TRXModule),
    Strength { tx_module: TRXModule, rx_module: TRXModule }
}

impl TRXSystem {
    #[must_use]
    pub fn tx_module(&self) -> &TRXModule {
        match self {
            Self::Color(trx_module)          => trx_module,
            Self::Strength { tx_module, .. } => tx_module 
        }
    }
    
    #[must_use]
    pub fn rx_module(&self) -> &TRXModule {
        match self {
            Self::Color(trx_module)          => trx_module,
            Self::Strength { rx_module, .. } => rx_module 
        }
    }
    
    #[must_use]
    pub fn tx_module_mut(&mut self) -> &mut TRXModule {
        match self {
            Self::Color(trx_module)          => trx_module,
            Self::Strength { tx_module, .. } => tx_module 
        }
    }
    
    #[must_use]
    pub fn rx_module_mut(&mut self) -> &mut TRXModule {
        match self {
            Self::Color(trx_module)          => trx_module,
            Self::Strength { rx_module, .. } => rx_module 
        }
    }

    #[must_use]
    pub fn max_tx_signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        match self {
            Self::Color(trx_module)          => 
                trx_module.max_signal_level(frequency),
            Self::Strength { tx_module, .. } => 
                tx_module.max_signal_level(frequency) 
        }
    }
    
    #[must_use]
    pub fn tx_signal_levels(&self) -> &FreqToLevelMap {
        match self {
            Self::Color(trx_module)          => trx_module.signal_levels(),
            Self::Strength { tx_module, .. } => tx_module.signal_levels() 
        }
    }

    #[must_use]
    pub fn tx_signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        match self {
            Self::Color(trx_module)          => 
                trx_module.signal_level(frequency),
            Self::Strength { tx_module, .. } => 
                tx_module.signal_level(frequency) 
        }
    }

    #[must_use]
    pub fn area(&self, frequency: Megahertz) -> SignalArea {
        SignalArea::from_level(
            *self.tx_signal_level(frequency),
            frequency
        )
    }
    
    pub fn set_tx_signal_levels(&mut self, signal_levels: &FreqToLevelMap) {
        match self {
            Self::Color(trx_module)          => 
                trx_module.set_signal_levels(signal_levels),
            Self::Strength { tx_module, .. } => 
                tx_module.set_signal_levels(signal_levels)
        }
    }
    
    pub fn set_tx_signal_level(
        &mut self, 
        signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        match self {
            Self::Color(trx_module)          => 
                trx_module.set_signal_level(signal_level, frequency),
            Self::Strength { tx_module, .. } => 
                tx_module.set_signal_level(signal_level, frequency)
        }
    }
    
    #[must_use]
    pub fn max_rx_signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        match self {
            Self::Color(trx_module)          => 
                trx_module.max_signal_level(frequency),
            Self::Strength { rx_module, .. } => 
                rx_module.max_signal_level(frequency) 
        }
    }
    
    #[must_use]
    pub fn rx_signal_levels(&self) -> &FreqToLevelMap {
        match self {
            Self::Color(trx_module)          => trx_module.signal_levels(),
            Self::Strength { rx_module, .. } => rx_module.signal_levels() 
        }
    }
    
    #[must_use]
    pub fn rx_signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        match self {
            Self::Color(trx_module)          => 
                trx_module.signal_level(frequency),
            Self::Strength { rx_module, .. } => 
                rx_module.signal_level(frequency) 
        }
    }
    
    pub fn set_rx_signal_levels(&mut self, signal_levels: &FreqToLevelMap) {
        match self {
            Self::Color(trx_module)          => 
                trx_module.set_signal_levels(signal_levels),
            Self::Strength { rx_module, .. } => 
                rx_module.set_signal_levels(signal_levels)
        }
    }
    
    pub fn set_rx_signal_level(
        &mut self, 
        signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        match self {
            Self::Color(trx_module)          => 
                trx_module.set_signal_level(signal_level, frequency),
            Self::Strength { rx_module, .. } => 
                rx_module.set_signal_level(signal_level, frequency)
        }
    }

    #[must_use]
    pub fn transmits_to(
        &self, 
        distance: Meter, 
        frequency: Megahertz
    ) -> bool {
        let tx_signal_level = self.tx_signal_level(frequency);

        // Optimization (if-statement is cheaper than calculating FSPL).
        if tx_signal_level.is_black() {
            return false;
        }

        !self.tx_signal_level_at(frequency, distance).is_black()
    }
    
    #[must_use]
    pub fn tx_signal_level_at(
        &self, 
        frequency: Megahertz,
        distance: Meter
    ) -> SignalLevel {
        match self {
            Self::Color(trx_module)          => trx_module
                .signal_level(frequency)
                .at_by_zone(frequency, distance),
            Self::Strength { tx_module, .. } => tx_module
                .signal_level(frequency)
                .at(frequency, distance)
        }
    }

    pub fn suppress_signal(
        &mut self,
        suppressor_signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        match self {
            Self::Color(trx_module)                 => { 
                let current_signal_level = *trx_module.signal_level(frequency);
                let suppressed_signal_level = suppressor_signal_level
                    .suppress_by_color(current_signal_level);

                trx_module.set_signal_level(suppressed_signal_level, frequency);
            },
            Self::Strength { tx_module, rx_module } => {
                let current_tx_signal_level = *tx_module.signal_level(
                    frequency
                );
                let suppressed_tx_signal_level = suppressor_signal_level
                    .suppress_by_strength(current_tx_signal_level);
                
                let current_rx_signal_level = *rx_module.signal_level(
                    frequency
                );
                let suppressed_rx_signal_level = suppressor_signal_level
                    .suppress_by_strength(current_rx_signal_level);
                
                tx_module.set_signal_level(
                    suppressed_tx_signal_level,
                    frequency, 
                );
                rx_module.set_signal_level(
                    suppressed_rx_signal_level,
                    frequency, 
                );
            }
        }
    }

    #[must_use]
    pub fn receives_signal(&self, frequency: Megahertz) -> bool {
        *self.rx_signal_level(frequency) > BLACK_SIGNAL_LEVEL
    }

    pub fn receive_signals(&mut self, signal_levels: &FreqToLevelMap) {
        signal_levels
            .iter()
            .for_each(|(frequency, signal_level)|
                self.receive_signal(*signal_level, *frequency)
            );
    }

    pub fn receive_signal(
        &mut self,
        tx_signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        match self {
            Self::Color(trx_module)          => {
                let current_signal_level = *trx_module
                    .signal_level(frequency);
                let received_signal_level = current_signal_level
                    .receive_by_color(tx_signal_level);
                
                trx_module.set_signal_level(received_signal_level, frequency);
            },
            Self::Strength { rx_module, .. } => {
                let max_signal_level = *rx_module
                    .max_signal_level(frequency);
                let received_signal_level = max_signal_level
                    .receive_by_strength(tx_signal_level);

                rx_module.set_signal_level(received_signal_level, frequency);
            },
        };
    }

    /// # Errors
    ///
    /// Will return `Err` if message execution cost exceeds RX signal level. 
    pub fn receive_message(
        &mut self,
        frequency: Megahertz,
        message: &Message
    ) -> Result<(), TRXSystemError> {
        let rx_module = match self {
            Self::Color(trx_module)          => trx_module,
            Self::Strength { rx_module, .. } => rx_module
        };
    
        let current_signal_level = rx_module.signal_level(frequency);
        let new_signal_level = current_signal_level - message.cost();
        
        if new_signal_level < NO_SIGNAL_LEVEL {
            return Err(TRXSystemError::TooExpensiveMessage);
        }

        rx_module.set_signal_level(new_signal_level, frequency);

        Ok(())
    }
}

// By default we create a non-functioning strength TRXSystem.
impl Default for TRXSystem {
    fn default() -> Self {
        Self::Strength { 
            tx_module: TRXModule::default(), 
            rx_module: TRXModule::default()
        }
    }
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::device::{BROADCAST_ID, UNKNOWN_ID};
    use crate::message::{Goal, Message, MessageType};
    use crate::signal::{
        SignalStrength, GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, RED_SIGNAL_LEVEL, 
        WIFI_2_4GHZ_FREQUENCY, YELLOW_SIGNAL_LEVEL
    };

    use super::*;


    #[test]
    fn default_power_system_does_not_supply_power() {
        let default_power_system = PowerSystem::default();

        assert_eq!(default_power_system.max_power(), 0);
    }

    #[test]
    fn building_power_system_with_power_greater_than_max_is_impossible() {
        let max_power      = 50;
        let too_high_power = max_power * 2;

        let result = PowerSystem::build(max_power, too_high_power);

        assert!(
            matches!(result, Err(PowerSystemBuildError::PowerIsGreaterThanMax))
        );
    }

    #[test]
    fn setting_higher_power_than_max_is_impossible() {
        let max_power = 50;
        let power     = max_power / 2;

        let mut power_system = PowerSystem::build(max_power, power)
            .unwrap_or_else(|error| panic!("{}", error));

        assert_eq!(power_system.max_power(), max_power);
        assert_eq!(power_system.power(), power);

        let too_high_power = max_power * 2;
        
        power_system.set_power(too_high_power);

        assert_eq!(power_system.power(), max_power);
    }
    
    #[test]
    fn default_movement_system_is_immovable() {
        let default_movement_system = MovementSystem::default();

        assert_eq!(default_movement_system.max_speed(), 0.0);
    }

    #[test]
    fn building_movement_system_with_negative_max_speed() {
        let result = MovementSystem::build(-5.0);

        assert!(
            matches!(result, Err(MovementSystemBuildError::NegativeMaxSpeed))
        );
    }

    #[test]
    fn setting_velocity() {
        let max_speed = 5.0;
        let mut movement_system = MovementSystem::build(max_speed)
            .unwrap();

        assert_eq!(*movement_system.velocity(), Vector3D::default());

        let normal_velocity = Vector3D::new(
            Point3D::default(), 
            Point3D::new(max_speed / 2.0, 0.0, 0.0)
        );
        movement_system.set_velocity(normal_velocity);
        
        assert_eq!(*movement_system.velocity(), normal_velocity);

        let too_high_velocity = Vector3D::new(
            Point3D::default(), 
            Point3D::new(max_speed * 2.0, 0.0, 0.0)
        );
        movement_system.set_velocity(too_high_velocity);

        assert_eq!(
            *movement_system.velocity(), 
            Vector3D::new(
                Point3D::default(),
                Point3D::new(max_speed, 0.0, 0.0)
            )
        );
    }

    #[test]
    fn setting_even_or_too_high_rx_signal_levels() {
        let max_rx_signal_levels = HashMap::from([
            (WIFI_2_4GHZ_FREQUENCY, YELLOW_SIGNAL_LEVEL),
            (GPS_L1_FREQUENCY, YELLOW_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (WIFI_2_4GHZ_FREQUENCY, YELLOW_SIGNAL_LEVEL),
            (GPS_L1_FREQUENCY, RED_SIGNAL_LEVEL)
        ]);

        let mut rx_system = TRXSystem::Strength {
            tx_module: TRXModule::default(),
            rx_module: TRXModule::build(
                    max_rx_signal_levels,
                    rx_signal_levels
                ).unwrap()
        };

        rx_system.set_rx_signal_level(
            GREEN_SIGNAL_LEVEL,
            WIFI_2_4GHZ_FREQUENCY, 
        );
        rx_system.set_rx_signal_level(
            YELLOW_SIGNAL_LEVEL,
            GPS_L1_FREQUENCY, 
        );

        assert!(
            rx_system
                .rx_signal_level(WIFI_2_4GHZ_FREQUENCY)
                .is_yellow()
        );
        assert!(
            rx_system
                .rx_signal_level(GPS_L1_FREQUENCY)
                .is_yellow()
        );
    }

    #[test]
    fn dummy_is_not_receiving_signal() {
        let dummy_trx_system = TRXSystem::default();

        assert!(!dummy_trx_system.receives_signal(GPS_L1_FREQUENCY));
    }
    
    #[test]
    fn is_not_receiving_black_signal() {
        let max_rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, SignalLevel::from(-10.0))
        ]);

        let negative_rx_system = TRXSystem::Strength {
            tx_module: TRXModule::default(),
            rx_module: TRXModule::build(
                    max_rx_signal_levels,
                    rx_signal_levels
                ).unwrap()
        };

        assert!(!negative_rx_system.receives_signal(GPS_L1_FREQUENCY));
    }

    #[test]
    fn is_receiving_signal() {
        let max_rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, YELLOW_SIGNAL_LEVEL)
        ]);

        let rx_system = TRXSystem::Strength {
            tx_module: TRXModule::default(),
            rx_module: TRXModule::build(
                    max_rx_signal_levels,
                    rx_signal_levels
                ).unwrap()
        };

        assert!(rx_system.receives_signal(GPS_L1_FREQUENCY));
    }

    #[test]
    fn receive_message_on_strength_trx_system() {
        let frequency = GPS_L1_FREQUENCY;
        let message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            0, 
            MessageType::SetGoal(Goal::Undefined)
        );

        let barely_green_signal_level = 
            YELLOW_SIGNAL_LEVEL + SignalStrength::new(0.1);

        let mut strength_rx_system = TRXSystem::Strength { 
            tx_module: TRXModule::default(),
            rx_module: TRXModule::build(
                    HashMap::from([(frequency, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(frequency, barely_green_signal_level)])
                ).unwrap()
        };

        assert!(
            strength_rx_system
                .receive_message(frequency, &message)
                .is_ok()
        );
        assert_eq!(
            *strength_rx_system.rx_signal_level(frequency),
            barely_green_signal_level - message.cost()
        );
    }

    #[test]
    fn receive_message_on_color_trx_system() {
        let frequency = GPS_L1_FREQUENCY;
        let message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            0, 
            MessageType::SetGoal(Goal::Undefined)
        );
        
        let barely_green_signal_level = 
            YELLOW_SIGNAL_LEVEL + SignalStrength::new(1.0);
        
        let mut color_rx_system = TRXSystem::Strength { 
            tx_module: TRXModule::default(),
            rx_module: TRXModule::build(
                    HashMap::from([(frequency, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(frequency, barely_green_signal_level)])
                ).unwrap()
        };

        assert!(
            color_rx_system
                .receive_message(frequency, &message)
                .is_ok()
        );
        assert!(
            color_rx_system
                .rx_signal_level(frequency)
                .is_yellow(),
        );
    }
    
    #[test]
    fn not_receive_too_expensive_message() {
        let frequency = GPS_L1_FREQUENCY;
        let message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            0, 
            MessageType::SetGoal(Goal::Undefined)
        );
        let rx_module = TRXModule::build(
            HashMap::from([(frequency, GREEN_SIGNAL_LEVEL)]),
            HashMap::new()
        ).unwrap_or_else(|error| panic!("{}", error));

        let mut strength_rx_system = TRXSystem::Strength {
            tx_module: TRXModule::default(),
            rx_module: rx_module.clone()
        };

        assert!(
            matches!(
                strength_rx_system.receive_message(frequency, &message),
                Err(TRXSystemError::TooExpensiveMessage)
            )
        );

        let mut color_rx_system = TRXSystem::Color(rx_module); 
        
        assert!(
            matches!(
                color_rx_system.receive_message(frequency, &message),
                Err(TRXSystemError::TooExpensiveMessage)
            )
        );
    }
}
