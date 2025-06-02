use thiserror::Error;

use crate::backend::mathphysics::{Megahertz, Meter};
use crate::backend::signal::{
    FreqToLevelMap, NO_SIGNAL_LEVEL, SignalLevel, min_signal_level
};


fn current_signal_levels_are_not_higher_than_max(
    current_signal_levels: &FreqToLevelMap,
    max_signal_levels: &FreqToLevelMap
) -> bool {
    current_signal_levels
        .iter()
        .all(|(frequency, signal_level)|
            *signal_level <= *max_signal_levels
                                .get(frequency)
                                .unwrap_or(&NO_SIGNAL_LEVEL)
        )
}


#[derive(Error, Debug)]
pub enum TRXModuleBuildError {
    #[error("Signal levels are higher than max signal levels")]
    SignalLevelsHigherThanMax,
}


// By default we create a non-functioning TRXModule.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TRXModule {
    max_signal_levels: FreqToLevelMap,
    signal_levels: FreqToLevelMap
}

impl TRXModule {
    /// # Errors
    ///
    /// Will return `Err` if provided signal levels are higher than provided
    /// maximum signal levels.
    pub fn build(
        max_signal_levels: FreqToLevelMap,
        signal_levels: FreqToLevelMap
    ) -> Result<Self, TRXModuleBuildError> {
        if !current_signal_levels_are_not_higher_than_max(
            &signal_levels, 
            &max_signal_levels
        ) {
            return Err(TRXModuleBuildError::SignalLevelsHigherThanMax);
        }

        Ok(Self { max_signal_levels, signal_levels })
    }

    #[must_use]
    pub fn max_signal_levels(&self) -> &FreqToLevelMap {
        &self.max_signal_levels
    }
    
    #[must_use]
    pub fn max_signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        self.max_signal_levels
            .get(&frequency)
            .unwrap_or(&NO_SIGNAL_LEVEL)
    }
    
    #[must_use]
    pub fn signal_levels(&self) -> &FreqToLevelMap {
        &self.signal_levels
    }

    #[must_use]
    pub fn signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        self.signal_levels
            .get(&frequency)
            .unwrap_or(&NO_SIGNAL_LEVEL)
    }
    
    pub fn set_signal_levels(&mut self, signal_levels: &FreqToLevelMap) {
        signal_levels
            .iter()
            .for_each(|(frequency, signal_level)|
                self.set_signal_level(*signal_level, *frequency)
            );
    }
    
    pub fn set_signal_level(
        &mut self, 
        signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        let max_signal_level = *self.max_signal_level(frequency);

        let new_signal_level = min_signal_level(
            signal_level,
            max_signal_level
        );

        self.signal_levels.insert(frequency, new_signal_level);
    }

    #[must_use]
    pub fn signal_level_at_by_color(
        &self,
        distance: Meter,
        frequency: Megahertz,
    ) -> SignalLevel {
        self
            .signal_level(frequency)
            .at_by_zone(frequency, distance)
    }
    
    #[must_use]
    pub fn signal_level_at_by_strength(
        &self,
        distance: Meter,
        frequency: Megahertz,
    ) -> SignalLevel {
        self
            .signal_level(frequency)
            .at(frequency, distance)
    }

    pub fn receive_signal_by_color(
        &mut self,
        tx_signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        let current_signal_level  = *self.signal_level(frequency);
        let received_signal_level = current_signal_level
            .receive_by_color(tx_signal_level);

        self.set_signal_level(received_signal_level, frequency);
    }

    pub fn receive_signal_by_strength(
        &mut self,
        tx_signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        let max_signal_level      = *self.max_signal_level(frequency);
        let received_signal_level = max_signal_level
            .receive_by_strength(tx_signal_level);

        self.set_signal_level(received_signal_level, frequency);
    }
    
    pub fn suppress_signal_by_color(
        &mut self,
        suppressor_signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        let current_signal_level    = *self.signal_level(frequency);
        let suppressed_signal_level = suppressor_signal_level
            .suppress_by_color(current_signal_level);

        self.set_signal_level(suppressed_signal_level, frequency);
    }

    pub fn suppress_signal_by_strength(
        &mut self,
        suppressor_signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        let current_signal_level    = *self.signal_level(frequency);
        let suppressed_signal_level = suppressor_signal_level
            .suppress_by_strength(current_signal_level);
        
        self.set_signal_level(suppressed_signal_level, frequency);
    }
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::backend::signal::{
        GPS_L1_FREQUENCY, RED_SIGNAL_LEVEL, YELLOW_SIGNAL_LEVEL
    };

    use super::*;


    #[test]
    fn trx_module_signal_level_higher_than_max() {
        let absent_max_signal_levels = HashMap::new();
        let current_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, YELLOW_SIGNAL_LEVEL)
        ]);

        let result = TRXModule::build(
            absent_max_signal_levels, 
            current_signal_levels.clone()
        );

        assert!(
            matches!(
                result,
                Err(TRXModuleBuildError::SignalLevelsHigherThanMax)
            )
        );
        
        let too_low_max_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, RED_SIGNAL_LEVEL)
        ]);
        let result = TRXModule::build(
            too_low_max_signal_levels, 
            current_signal_levels
        );

        assert!(
            matches!(
                result,
                Err(TRXModuleBuildError::SignalLevelsHigherThanMax)
            )
        );
    }

    #[test]
    fn trx_module_successful_build() {
        let correct_max_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, YELLOW_SIGNAL_LEVEL)
        ]);
        let current_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, YELLOW_SIGNAL_LEVEL)
        ]);

        let result = TRXModule::build(
            correct_max_signal_levels, 
            current_signal_levels
        );

        assert!(result.is_ok());
    }
}
