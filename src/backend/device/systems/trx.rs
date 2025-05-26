use thiserror::Error;

use crate::backend::mathphysics::{Megahertz, Meter};
use crate::backend::message::Message;
use crate::backend::signal::{
    BLACK_SIGNAL_LEVEL, FreqToLevelMap, SignalArea, SignalLevel
};

use super::TRXModule;


#[derive(Error, Debug)]
pub enum TRXSystemError {
    #[error("Message execution cost is too high")]
    TooExpensiveMessage,
    #[error("Message destination ID does not match device ID")]
    WrongMessageDestination,
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
        }
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
        if current_signal_level < message.cost() {
            return Err(TRXSystemError::TooExpensiveMessage);
        }
        
        let new_signal_level = current_signal_level - message.cost();
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

    use crate::backend::device::{BROADCAST_ID, UNKNOWN_ID};
    use crate::backend::message::{Task, Message, MessageType};
    use crate::backend::signal::{
        SignalStrength, GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, RED_SIGNAL_LEVEL, 
        WIFI_2_4GHZ_FREQUENCY, YELLOW_SIGNAL_LEVEL
    };

    use super::*;


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
            MessageType::SetTask(Task::Undefined)
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
            MessageType::SetTask(Task::Undefined)
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
            MessageType::SetTask(Task::Undefined)
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
