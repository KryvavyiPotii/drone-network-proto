use std::ops;

use impl_ops::{
    _impl_binary_op_borrowed_borrowed, _impl_binary_op_borrowed_owned, 
    _impl_binary_op_internal, _impl_binary_op_owned_borrowed, 
    _impl_binary_op_owned_owned, _parse_binary_op, impl_op, impl_op_ex
};

use crate::mathphysics::{wave_length_in_meters, Megahertz, Meter};
use crate::message::MessageCost;
use crate::signal::{
    GREEN_SIGNAL_STRENGTH, MAX_BLACK_SIGNAL_STRENGTH, MAX_RED_SIGNAL_STRENGTH, 
    MAX_YELLOW_SIGNAL_STRENGTH, NO_SIGNAL_STRENGTH, SignalArea, SignalStrength,
    SIGNAL_STRENGTH_SCALING,
};


pub const RED_SIGNAL_ZONE_COEFFICIENT: f32    = 0.875;
pub const YELLOW_SIGNAL_ZONE_COEFFICIENT: f32 = 0.8;
pub const GREEN_SIGNAL_ZONE_COEFFICIENT: f32  = 0.5;

pub const NO_SIGNAL_LEVEL: SignalLevel     =
    SignalLevel(SignalLevelInner::Black(NO_SIGNAL_STRENGTH));
pub const BLACK_SIGNAL_LEVEL: SignalLevel  = 
    SignalLevel(SignalLevelInner::Black(MAX_BLACK_SIGNAL_STRENGTH));
pub const RED_SIGNAL_LEVEL: SignalLevel    = 
    SignalLevel(SignalLevelInner::Red(MAX_RED_SIGNAL_STRENGTH));
pub const YELLOW_SIGNAL_LEVEL: SignalLevel = 
    SignalLevel(SignalLevelInner::Yellow(MAX_YELLOW_SIGNAL_STRENGTH));
pub const GREEN_SIGNAL_LEVEL: SignalLevel  = 
    SignalLevel(SignalLevelInner::Green(GREEN_SIGNAL_STRENGTH));


#[must_use]
pub fn min_signal_level(
    signal_level1: SignalLevel,
    signal_level2: SignalLevel
) -> SignalLevel {
    if signal_level1 < signal_level2 {
        signal_level1
    } else {
        signal_level2
    }
}


#[derive(Clone, Copy, PartialEq, PartialOrd, Debug, Default)]
pub struct SignalLevel(SignalLevelInner);

impl SignalLevel {
    // Inverse operation to SignalArea::from_level()
    #[must_use]
    pub fn from_area(signal_area: SignalArea, frequency: Megahertz) -> Self {
        let wave_length = wave_length_in_meters(frequency);

        // TX signal strength is such signal strength that grants at least
        // black RX signal strength on the signal area radius.
        // So, the actual formula is:
        //     tx_signal_strength = (
        //         MAX_BLACK_SIGNAL_STRENGTH * radius / wave_length
        //     ).powi()
        // We do not use multiplication by MAX_BLACK_SIGNAL_STRENGTH because it 
        // is equal to 1.0.
        let tx_signal_strength = (
            signal_area.radius() / wave_length
        ).powi(2) / SIGNAL_STRENGTH_SCALING;

        Self(SignalLevelInner::from(tx_signal_strength))
    }
    
    #[must_use]
    pub fn at(&self, frequency: Megahertz, distance: Meter) -> Self {
        if *self <= NO_SIGNAL_LEVEL {
            return NO_SIGNAL_LEVEL;
        }

        let wave_length = wave_length_in_meters(frequency);

        // For now we ignore division by distance, if it is less than a wave
        // length. However, in the future free-space path loss model may 
        // changed for this particular case.
        let rx_signal_strength = if distance <= wave_length {
            wave_length.powi(2)
        } else {
            (wave_length / distance).powi(2)
        } * self.strength().value() * SIGNAL_STRENGTH_SCALING; 

        let signal_level_inner = SignalLevelInner::from(rx_signal_strength);

        Self(signal_level_inner)
    }

    #[must_use]
    pub fn at_by_zone(&self, frequency: Megahertz, distance: Meter) -> Self {
        let radius = SignalArea::from_level(*self, frequency).radius();

        if distance <= radius * GREEN_SIGNAL_ZONE_COEFFICIENT {
            *self
        } else if distance <= radius * YELLOW_SIGNAL_ZONE_COEFFICIENT {
            self.lower_level()
        } else if distance <= radius {
            self.lower_level().lower_level()
        } else {
            BLACK_SIGNAL_LEVEL
        }
    }
    
    // self - SignalLevel of Receiver.
    #[must_use]
    pub fn receive_by_color(&self, tx_signal_level: Self) -> Self {
        if tx_signal_level.is_green() {
            *self
        } else if self.is_green() {
            tx_signal_level
        } else if tx_signal_level.is_yellow() && self.is_yellow() {
            RED_SIGNAL_LEVEL
        } else {
            BLACK_SIGNAL_LEVEL
        }
    }

    // self - maximum SignalLevel of Receiver.
    #[must_use]
    pub fn receive_by_strength(&self, tx_signal_level: Self ) -> Self {
        min_signal_level(*self, tx_signal_level)
    }

    // self - SignalLevel of Suppressor.
    #[must_use]
    pub fn suppress_by_color(&self, rx_signal_level: Self) -> Self {
        if self.is_black() {
            rx_signal_level    
        } else if self.is_red() && rx_signal_level.is_green() {
            YELLOW_SIGNAL_LEVEL
        } else if (self.is_yellow() && rx_signal_level.is_green()) 
            || (self.is_red() && rx_signal_level.is_yellow())
        {
            RED_SIGNAL_LEVEL
        } else {
            BLACK_SIGNAL_LEVEL
        }
    }

    // self - SignalLevel of Suppressor.
    #[must_use]
    pub fn suppress_by_strength(&self, rx_signal_level: Self) -> Self {
        rx_signal_level - self 
    }

    fn lower_level(self) -> Self {
        match self.0 {
            SignalLevelInner::Black(_) | SignalLevelInner::Red(_) => 
                BLACK_SIGNAL_LEVEL,
            SignalLevelInner::Yellow(_) => RED_SIGNAL_LEVEL,
            SignalLevelInner::Green(_)  => YELLOW_SIGNAL_LEVEL
        }
    }

    #[must_use]
    pub fn same_level(&self, other: &Self) -> bool {
        (self.is_black() && other.is_black()) 
            || (self.is_red() && other.is_red()) 
            || (self.is_yellow() && other.is_yellow()) 
            || (self.is_green() && other.is_green()) 
    }

    #[must_use]
    pub fn is_black(&self) -> bool {
        matches!(self.0, SignalLevelInner::Black(_))
    }
    
    #[must_use]
    pub fn is_red(&self) -> bool {
        matches!(self.0, SignalLevelInner::Red(_))
    }
    
    #[must_use]
    pub fn is_yellow(&self) -> bool {
        matches!(self.0, SignalLevelInner::Yellow(_))
    }

    #[must_use]
    pub fn is_green(&self) -> bool {
        matches!(self.0, SignalLevelInner::Green(_))
    }

    #[must_use]
    pub fn strength(&self) -> SignalStrength {
        match self.0 {
            SignalLevelInner::Black(strength)
                | SignalLevelInner::Red(strength)
                | SignalLevelInner::Yellow(strength)
                | SignalLevelInner::Green(strength) => strength
        }
    }
}

impl PartialEq<&SignalLevel> for SignalLevel {
    fn eq(&self, other: &&SignalLevel) -> bool {
        self.0 == other.0
    }
}

impl PartialEq<SignalLevel> for &SignalLevel {
    fn eq(&self, other: &SignalLevel) -> bool {
        self.0 == other.0
    }
}

impl_op_ex!(
    - |a: &SignalLevel, b: &SignalLevel| -> SignalLevel { 
        SignalLevel(SignalLevelInner::from(a.strength() - b.strength()))
    }
);
impl_op_ex!(
    - |a: &SignalLevel, b: &SignalStrength| -> SignalLevel { 
        SignalLevel(SignalLevelInner::from(a.strength() - b))
    }
);
impl_op_ex!(
    - |a: &SignalLevel, b: &MessageCost| -> SignalLevel { 
        SignalLevel(SignalLevelInner::from(a.strength() - b.value()))
    }
);
impl_op_ex!(
    + |a: &SignalLevel, b: &SignalLevel| -> SignalLevel { 
        SignalLevel(SignalLevelInner::from(a.strength() + b.strength()))
    }
);
impl_op_ex!(
    + |a: &SignalLevel, b: &SignalStrength| -> SignalLevel { 
        SignalLevel(SignalLevelInner::from(a.strength() + b))
    }
);
impl_op_ex!(
    + |a: &SignalLevel, b: &MessageCost| -> SignalLevel { 
        SignalLevel(SignalLevelInner::from(a.strength() + b.value()))
    }
);

impl From<f32> for SignalLevel {
    fn from(value: f32) -> Self {
        Self(SignalLevelInner::from(value))
    }
}

impl From<SignalStrength> for SignalLevel {
    fn from(signal_strength: SignalStrength) -> Self {
        Self(SignalLevelInner::from(signal_strength))
    }
}


#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
enum SignalLevelInner {
    Black(SignalStrength),  // (almost) no signal
    Red(SignalStrength),    // signal level is critically low
    Yellow(SignalStrength), // signal level is decent
    Green(SignalStrength)   // signal level is good
}

impl Default for SignalLevelInner {
    fn default() -> Self {
        Self::Black(NO_SIGNAL_STRENGTH)
    }
}

impl From<f32> for SignalLevelInner {
    fn from(value: f32) -> Self {
        Self::from(SignalStrength::new(value))
    }
}

impl From<SignalStrength> for SignalLevelInner {
    fn from(signal_strength: SignalStrength) -> Self {
        if signal_strength > MAX_YELLOW_SIGNAL_STRENGTH {
            Self::Green(signal_strength)
        } else if signal_strength > MAX_RED_SIGNAL_STRENGTH {
            Self::Yellow(signal_strength)
        } else if signal_strength > MAX_BLACK_SIGNAL_STRENGTH {
            Self::Red(signal_strength)
        } else {
            Self::Black(signal_strength)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    

    const SOME_FREQUENCY: Megahertz = 2_000;


    fn rx_signal_level_at_tx(tx_signal_level: &SignalLevel) -> SignalLevel {
        tx_signal_level.at(SOME_FREQUENCY, 0.0)
    }

    fn rx_signal_levels_in_different_zones_by_color(
        tx_signal_level: &SignalLevel
    ) -> (SignalLevel, SignalLevel, SignalLevel, SignalLevel) {
        let radius = SignalArea::from_level(
            tx_signal_level.clone(),
            SOME_FREQUENCY
        ).radius();
        
        let green_zone_rx_signal_level = tx_signal_level.at_by_zone(
            SOME_FREQUENCY,
            radius * GREEN_SIGNAL_ZONE_COEFFICIENT
        );
        let yellow_zone_rx_signal_level = tx_signal_level.at_by_zone(
            SOME_FREQUENCY,
            radius * YELLOW_SIGNAL_ZONE_COEFFICIENT
        );
        let red_zone_rx_signal_level = tx_signal_level.at_by_zone(
            SOME_FREQUENCY,
            radius * RED_SIGNAL_ZONE_COEFFICIENT
        );
        let black_zone_rx_signal_level = tx_signal_level.at_by_zone(
            SOME_FREQUENCY,
            radius + 1.0
        );

        (
            green_zone_rx_signal_level,
            yellow_zone_rx_signal_level,
            red_zone_rx_signal_level,
            black_zone_rx_signal_level
        )
    }

    fn rx_signal_level_is_lower_than_tx_by_color(
        tx_signal_level: &SignalLevel
    ) {
        let (
            green_zone_rx_signal_level,
            yellow_zone_rx_signal_level,
            red_zone_rx_signal_level,
            black_zone_rx_signal_level
        ) = rx_signal_levels_in_different_zones_by_color(tx_signal_level);
       
        assert!(tx_signal_level.same_level(&green_zone_rx_signal_level));
        assert!(
            tx_signal_level
                .lower_level()
                .same_level(&yellow_zone_rx_signal_level)
        );
        assert!(
            tx_signal_level
                .lower_level()
                .lower_level()
                .same_level(&red_zone_rx_signal_level)
        );
        assert!(
            tx_signal_level
                .lower_level()
                .lower_level()
                .lower_level()
                .same_level(&black_zone_rx_signal_level)
        );
    }

    fn rx_signal_level_is_lower_than_tx_by_strength(
        tx_signal_level: &SignalLevel
    ) {
        let radius = SignalArea::from_level(
            tx_signal_level.clone(),
            SOME_FREQUENCY
        ).radius();
        
        let rx_signal_level_at_half = tx_signal_level.at(
            SOME_FREQUENCY,
            radius / 2.0
        );
        let rx_signal_level_outside = tx_signal_level.at(
            SOME_FREQUENCY,
            radius + 1.0
        );
        let no_rx_signal_level = BLACK_SIGNAL_LEVEL.at(
            SOME_FREQUENCY, 
            0.0
        );

        assert!(
            rx_signal_level_at_half <= rx_signal_level_at_tx(tx_signal_level)
        );
        assert!(
            rx_signal_level_outside <= no_rx_signal_level
        )
    }

    
    #[test]
    fn negative_signal_strength_is_allowed() {
        assert_eq!(
            -10.0, 
            SignalLevel::from(-10.0).strength().value()
        );
    }

    #[test]
    fn correct_const_colors() {
        assert!(BLACK_SIGNAL_LEVEL.is_black());
        assert!(RED_SIGNAL_LEVEL.is_red());
        assert!(YELLOW_SIGNAL_LEVEL.is_yellow());
        assert!(GREEN_SIGNAL_LEVEL.is_green());
    }

    #[test]
    fn lowering_signal_level() {
        assert!(
            GREEN_SIGNAL_LEVEL
                .lower_level()
                .is_yellow()
        );
        assert!(
            YELLOW_SIGNAL_LEVEL
                .lower_level()
                .is_red()
        );
        assert!(
            RED_SIGNAL_LEVEL
                .lower_level()
                .is_black()
        );
        assert!(
            BLACK_SIGNAL_LEVEL
                .lower_level()
                .is_black()
        );
    }
    
    #[test]
    fn signal_level_comparison_within_one_color() {
        assert!(SignalLevel::from(5.0) < SignalLevel::from(50.0));
        assert!(SignalLevel::from(10.0) == SignalLevel::from(10.0));
        assert!(SignalLevel::from(10.0) >= SignalLevel::from(10.0));
        assert!(SignalLevel::from(10.0) <= SignalLevel::from(10.0));
        assert!(SignalLevel::from(15.0) > SignalLevel::from(5.0));
    }

    #[test]
    fn signal_level_comparison_within_different_colors() {
        assert!(YELLOW_SIGNAL_LEVEL < GREEN_SIGNAL_LEVEL);
        assert!(RED_SIGNAL_LEVEL >= BLACK_SIGNAL_LEVEL); 
    }

    #[test]
    fn signal_level_color_from_strength() {
        assert_eq!(
            SignalLevelInner::from(-6.0),
            SignalLevelInner::Black(SignalStrength::new(-6.0))
        );
        assert_eq!(
            SignalLevelInner::from(0.1),
            SignalLevelInner::Black(SignalStrength::new(0.1))
        );
        assert_eq!(
            SignalLevelInner::from(MAX_YELLOW_SIGNAL_STRENGTH),
            SignalLevelInner::Yellow(MAX_YELLOW_SIGNAL_STRENGTH)
        );
    }

    #[test]
    fn negative_signal_level_at_by_strength() {
        let some_distance = 5.0;

        assert_eq!(
            NO_SIGNAL_LEVEL,
            SignalLevel::from(-10.0)
                .at(SOME_FREQUENCY, some_distance)
        )
    }

    #[test]
    fn somewhat_realistic_area_radius_by_strength() {
        let frequency = 5_000;
        
        assert!(
            GREEN_SIGNAL_LEVEL
                .at(frequency, 40.0)
                .is_black()
        );
        assert!(
            GREEN_SIGNAL_LEVEL
                .at(frequency, 15.0)
                .is_red()
        );
        assert!(
            GREEN_SIGNAL_LEVEL
                .at(frequency, 5.0)
                .is_yellow()
        );
        assert!(
            GREEN_SIGNAL_LEVEL
                .at(frequency, 3.0)
                .is_green()
        );
    }

    #[test]
    fn correct_signal_level_at_rx_by_color() {
        rx_signal_level_is_lower_than_tx_by_color(&GREEN_SIGNAL_LEVEL); 
        rx_signal_level_is_lower_than_tx_by_color(&YELLOW_SIGNAL_LEVEL); 
        rx_signal_level_is_lower_than_tx_by_color(&RED_SIGNAL_LEVEL); 
        rx_signal_level_is_lower_than_tx_by_color(&BLACK_SIGNAL_LEVEL); 
    }
   
    #[test]
    fn correct_signal_level_at_rx_by_strength() {
        rx_signal_level_is_lower_than_tx_by_strength(&GREEN_SIGNAL_LEVEL); 
        rx_signal_level_is_lower_than_tx_by_strength(&YELLOW_SIGNAL_LEVEL); 
        rx_signal_level_is_lower_than_tx_by_strength(&RED_SIGNAL_LEVEL); 
        rx_signal_level_is_lower_than_tx_by_strength(&BLACK_SIGNAL_LEVEL); 
    }

    #[test]
    fn receive_green_signal_level_with_green_max_by_color() {
        assert!(
            GREEN_SIGNAL_LEVEL.receive_by_color(
                GREEN_SIGNAL_LEVEL
            ).is_green()
        );
    }

    #[test]
    fn receive_higher_signal_level_with_lower_max_by_color() {
        assert!(
            YELLOW_SIGNAL_LEVEL.receive_by_color(
                GREEN_SIGNAL_LEVEL
            ).is_yellow()
        );
    }

    #[test]
    fn receive_lower_signal_level_with_green_max_by_color() {
        assert!(
            GREEN_SIGNAL_LEVEL.receive_by_color(
                RED_SIGNAL_LEVEL
            ).is_red()
        );
    }

    #[test]
    fn receive_yellow_signal_level_with_yellow_max_by_color() {
        assert!(
            YELLOW_SIGNAL_LEVEL.receive_by_color(
                YELLOW_SIGNAL_LEVEL
            ).is_red()
        );
    }

    #[test]
    fn receive_lower_signal_level_with_yellow_max_by_color() {
        assert!(
            YELLOW_SIGNAL_LEVEL.receive_by_color(
                RED_SIGNAL_LEVEL
            ).is_black()
        );
    }

    #[test]
    fn receive_signal_with_even_max_by_strength() {
        let tx_signal_level = SignalLevel::from(5.0);
        let max_rx_signal_level = SignalLevel::from(5.0);

        assert_eq!(
            max_rx_signal_level,
            max_rx_signal_level.receive_by_strength(tx_signal_level)
        );
    }
   
    #[test]
    fn receive_higher_signal_level_with_lower_max_by_strength() {
        let tx_signal_level = SignalLevel::from(15.0);
        let max_rx_signal_level = SignalLevel::from(5.0);

        assert_eq!(
            max_rx_signal_level,
            max_rx_signal_level.receive_by_strength(tx_signal_level)
        );
    }
    
    #[test]
    fn receive_lower_signal_level_with_higher_max_by_strength() {
        let tx_signal_level = SignalLevel::from(5.0);
        let max_rx_signal_level = SignalLevel::from(15.0);

        assert_eq!(
            tx_signal_level,
            max_rx_signal_level.receive_by_strength(tx_signal_level)
        );
    }

    #[test]
    fn not_suppress_signal_by_color() {
        let suppressor_signal_level = BLACK_SIGNAL_LEVEL;
        let rx_signal_level = YELLOW_SIGNAL_LEVEL;
        
        assert_eq!(
            rx_signal_level,
            suppressor_signal_level.suppress_by_color(rx_signal_level)
        );
    }
    
    #[test]
    fn partially_suppress_green_signal_level_with_red_by_color() {
        let suppressor_signal_level = RED_SIGNAL_LEVEL;
        let rx_signal_level = GREEN_SIGNAL_LEVEL;
        
        assert!(
            suppressor_signal_level
                .suppress_by_color(rx_signal_level)
                .is_yellow()
        );
    }
    
    #[test]
    fn partially_suppress_green_signal_level_with_yellow_by_color() {
        let suppressor_signal_level = YELLOW_SIGNAL_LEVEL;
        let rx_signal_level = GREEN_SIGNAL_LEVEL;
        
        assert!(
            suppressor_signal_level
                .suppress_by_color(rx_signal_level)
                .is_red()
        );
    }
    
    #[test]
    fn partially_suppress_yellow_signal_level_with_red_by_color() {
        let suppressor_signal_level = YELLOW_SIGNAL_LEVEL;
        let rx_signal_level = GREEN_SIGNAL_LEVEL;
        
        assert!(
            suppressor_signal_level
                .suppress_by_color(rx_signal_level)
                .is_red()
        );
    }
    
    #[test]
    fn completely_suppress_signal_by_color() {
        let suppressor_signal_level = GREEN_SIGNAL_LEVEL;
        let rx_signal_level = GREEN_SIGNAL_LEVEL;
        
        assert!(
            suppressor_signal_level
                .suppress_by_color(rx_signal_level)
                .is_black()
        );
    }
   
    #[test]
    fn suppress_even_signal_level_by_strength() {
        let suppressor_signal_level = SignalLevel::from(5.0);
        let rx_signal_level = SignalLevel::from(5.0);
        
        assert_eq!(
            SignalLevel::from(0.0),
            suppressor_signal_level.suppress_by_strength(rx_signal_level)
        );
    }
    
    #[test]
    fn suppress_lower_signal_level_by_strength() {
        let suppressor_signal_level = SignalLevel::from(15.0);
        let rx_signal_level = SignalLevel::from(5.0);
        
        assert_eq!(
            SignalLevel::from(-10.0),
            suppressor_signal_level.suppress_by_strength(rx_signal_level)
        );
    }
    
    #[test]
    fn suppress_higher_signal_level_by_strength() {
        let suppressor_signal_level = SignalLevel::from(5.0);
        let rx_signal_level = SignalLevel::from(15.0);
        
        assert_eq!(
            SignalLevel::from(10.0),
            suppressor_signal_level.suppress_by_strength(rx_signal_level)
        );
    }
}
