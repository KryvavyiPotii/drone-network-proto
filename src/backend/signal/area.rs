use thiserror::Error;

use crate::backend::mathphysics::{wave_length_in_meters, Megahertz, Meter};

use super::{
    MAX_BLACK_SIGNAL_STRENGTH, SignalLevel, SignalStrength, 
    SIGNAL_STRENGTH_SCALING
};


#[derive(Debug, Error)]
pub enum SignalAreaError {
    #[error("Negative radius is forbidden")]
    NegativeRadius
}


#[derive(Clone, Copy, Debug, Default)]
pub struct SignalArea {
    radius: Meter
}

impl SignalArea {
    /// # Errors
    ///
    /// Will return `Err` if provided radius is a negative number.
    pub fn build(radius: Meter) -> Result<Self, SignalAreaError> {
        if radius < 0.0 {
            return Err(SignalAreaError::NegativeRadius);
        }

        Ok(Self { radius })
    }
   
    // Inverse operation to SignalLevel::from_area()
    #[must_use]
    pub fn from_strength(
        tx_signal_strength: SignalStrength,
        frequency: Megahertz
    ) -> Self {
        let wave_length = wave_length_in_meters(frequency);

        let radius = if tx_signal_strength <= MAX_BLACK_SIGNAL_STRENGTH {
            0.0
        } else {
            // The area radius is a minimal distance from the tx at which 
            // the signal level is black.
            // So, the actual formula is:
            //     radius = wave_length * (
            //         tx_signal_strength / MAX_BLACK_SIGNAL_STRENGTH
            //     ).sqrt()
            // We do not use division by MAX_BLACK_SIGNAL_STRENGTH because it 
            // is equal to 1.0.
            wave_length * (
                tx_signal_strength.value() * SIGNAL_STRENGTH_SCALING
            ).sqrt() 
        } as Meter;
        
        Self { radius }
    }
    
    #[must_use]
    pub fn from_level(
        tx_signal_level: SignalLevel,
        frequency: Megahertz
    ) -> Self {
        Self::from_strength(tx_signal_level.strength(), frequency)
    }

    #[must_use]
    pub fn radius(&self) -> Meter {
        self.radius
    }

    /// # Errors
    ///
    /// Will return `Err` if provided radius is a negative number.
    pub fn set_radius(&mut self, radius: Meter) -> Result<(), SignalAreaError> {
        if radius < 0.0 {
            return Err(SignalAreaError::NegativeRadius);
        }

        self.radius = radius;

        Ok(())
    }
}


#[cfg(test)]
mod tests {
    use crate::backend::signal::GREEN_SIGNAL_LEVEL;

    use super::*;


    #[test]
    fn signal_area_from_black_signal() {
        let negative_strength = SignalStrength::new(-5.0);
        let zero_strength = SignalStrength::default();
        let negligible_strength = SignalStrength::new(0.5);
        let frequency = 5_000;

        assert_eq!(
            0.0, 
            SignalArea::from_strength(negative_strength, frequency)
                .radius()
        );
        assert_eq!(
            0.0, 
            SignalArea::from_strength(zero_strength, frequency)
                .radius()
        );
        assert_eq!(
            0.0, 
            SignalArea::from_strength(negligible_strength, frequency)
                .radius()
        );
    }
    
    #[test]
    fn signal_area_from_nonblack_signal() {
        let frequency = 5_000;

        assert_eq!(
            30.0,
            SignalArea::from_level(GREEN_SIGNAL_LEVEL, frequency)
                .radius()
                .round()
        );
    }
}
