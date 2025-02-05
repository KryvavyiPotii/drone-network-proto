use std::collections::HashMap;

use derive_more::{Add, Sub, Mul, Div};

use crate::{device::DeviceId, mathphysics::{Meter, MeterPerSecond}};

use super::message::MessageCost;


pub const CC_TX_CONTROL_AREA: SignalArea    = SignalArea { radius: 300.0 };
pub const RWD_TX_CONTROL_AREA: SignalArea   = SignalArea { radius: 25.0 };
pub const RWD_TX_GPS_AREA: SignalArea       = SignalArea { radius: 50.0 };
pub const DRONE_TX_CONTROL_AREA: SignalArea = SignalArea { radius: 50.0 };

pub const SIGNAL_SPEED: MeterPerSecond = 0.0002;

pub const RED_SIGNAL_ZONE_COEFFICIENT: f32    = 0.875;
pub const YELLOW_SIGNAL_ZONE_COEFFICIENT: f32 = 0.7;
pub const GREEN_SIGNAL_ZONE_COEFFICIENT: f32  = 0.5;

const SIGNAL_LEVEL_FROM_AREA_COEFFICIENT: f32        = 1.0;
// Created a separate sqrt const for improving performance. 
// Should be updated with every change of SIGNAL_LEVEL_FROM_AREA_COEFFICIENT!
const SIGNAL_LEVEL_FROM_AREA_COEFFICIENT_SQRT: f32   = 1.0;
pub const BASE_SIGNAL_STRENGTH_VALUE: f32            = 200_000.0;
pub const NO_SIGNAL_STRENGTH: SignalStrength         = SignalStrength(0.0);
pub const MAX_BLACK_SIGNAL_STRENGTH: SignalStrength  = SignalStrength(1.0);
pub const MAX_RED_SIGNAL_STRENGTH: SignalStrength    = SignalStrength(
    BASE_SIGNAL_STRENGTH_VALUE * (1.0 - RED_SIGNAL_ZONE_COEFFICIENT)
);
pub const MAX_YELLOW_SIGNAL_STRENGTH: SignalStrength = SignalStrength(
    BASE_SIGNAL_STRENGTH_VALUE * (1.0 - YELLOW_SIGNAL_ZONE_COEFFICIENT)
);
pub const BASE_SIGNAL_STRENGTH: SignalStrength = SignalStrength(
    BASE_SIGNAL_STRENGTH_VALUE
);

pub const NO_SIGNAL_LEVEL: SignalLevel     =
    SignalLevel(SignalLevelInner::Black(NO_SIGNAL_STRENGTH));
pub const BLACK_SIGNAL_LEVEL: SignalLevel  = 
    SignalLevel(SignalLevelInner::Black(MAX_BLACK_SIGNAL_STRENGTH));
pub const RED_SIGNAL_LEVEL: SignalLevel    = 
    SignalLevel(SignalLevelInner::Red(MAX_RED_SIGNAL_STRENGTH));
pub const YELLOW_SIGNAL_LEVEL: SignalLevel = 
    SignalLevel(SignalLevelInner::Yellow(MAX_YELLOW_SIGNAL_STRENGTH));
pub const GREEN_SIGNAL_LEVEL: SignalLevel  = 
    SignalLevel(SignalLevelInner::Green(BASE_SIGNAL_STRENGTH));


pub fn min_signal_strength(
    signal_strength1: SignalStrength,
    signal_strength2: SignalStrength
) -> SignalStrength {
    if signal_strength1 < signal_strength2 {
        signal_strength1
    } else {
        signal_strength2
    }
}

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

pub fn same_levels_of_id_hashmaps(
    signal_levels1: &HashMap<DeviceId, SignalLevel>,
    signal_levels2: &HashMap<DeviceId, SignalLevel>
) -> bool {
    signal_levels1
        .iter()
        .all(|(id, signal_level1)| 
            match signal_levels2.get(id) {
                Some(signal_level2) => signal_level2.same_level(signal_level1),
                None => signal_level1.same_level(&NO_SIGNAL_LEVEL)
            }
        )
}


#[derive(Clone, Copy, Hash, Eq, PartialEq, Debug)]
pub enum SignalType {
    GPS,
    Control,
    // TODO implement custom frequencies - Custom(f32)
}


#[derive(Clone, Copy, Debug, Default)]
pub struct SignalArea {
    radius: Meter
}

impl SignalArea {
    pub fn build(radius: Meter) -> Result<Self, &'static str> {
        if radius < 0.0 {
            return Err("Negative radius is forbidden");
        }

        Ok(Self { radius })
    }

    pub fn radius(&self) -> Meter {
        self.radius
    }

    pub fn set_radius(&mut self, radius: Meter) -> Result<(), &'static str> {
        if radius < 0.0 {
            return Err("Negative radius is forbidden");
        }

        self.radius = radius;

        Ok(())
    }
}

impl From<SignalStrength> for SignalArea {
    // Formula is:
    //     radius = coef.sqrt() * ((tx_sig_str / black_sig_str).sqrt() - 1)
    fn from(tx_signal_strength: SignalStrength) -> Self {
        let value = tx_signal_strength.0;

        let radius = if value <= 0.0 {
            0.0
        } else {
            SIGNAL_LEVEL_FROM_AREA_COEFFICIENT_SQRT * (value.sqrt() - 1.0) 
        } as Meter;
        
        Self { radius }
    }
}

impl From<&SignalLevel> for SignalArea {
    fn from(tx_signal_level: &SignalLevel) -> Self {
        Self::from(tx_signal_level.strength())
    }
}

impl From<SignalLevel> for SignalArea {
    fn from(tx_signal_level: SignalLevel) -> Self {
        Self::from(tx_signal_level.strength())
    }
}


#[derive(
    Clone, Copy, Debug, Default,
    Add, Mul, PartialEq, PartialOrd
)]
pub struct SignalStrength(f32);

impl SignalStrength {
    pub fn new(value: f32) -> Self {
        Self(value)
    }

    pub fn value(&self) -> f32 {
        self.0
    }
}

impl Sub for SignalStrength {
    type Output = Self;
    
    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Sub<f32> for SignalStrength {
    type Output = Self;
    
    fn sub(self, rhs: f32) -> Self::Output {
        Self(self.0 - rhs)
    }
}

impl Sub<MessageCost> for SignalStrength {
    type Output = Self;
    
    fn sub(self, rhs: MessageCost) -> Self::Output {
        Self(self.0 - rhs.value())
    }
}

impl Div for SignalStrength {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        Self(self.0 / rhs.0)
    }
}

impl Div<f32> for SignalStrength {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self(self.0 / rhs)
    }
}

impl Div<MessageCost> for SignalStrength {
    type Output = Self;

    fn div(self, rhs: MessageCost) -> Self::Output {
        Self(self.0 / rhs.value())
    }
}


#[derive(Clone, Copy, PartialEq, PartialOrd, Debug, Default)]
pub struct SignalLevel(SignalLevelInner);

impl SignalLevel {
    // Formula:
    //     rx_str = tx_str * coef / (distance + coef.sqrt()).pow(2)
    pub fn at(&self, distance: Meter) -> Self {
        if distance == 0.0 {
            return *self;
        }

        let numerator = self.strength() * SIGNAL_LEVEL_FROM_AREA_COEFFICIENT;
        let denumerator = (
            distance + SIGNAL_LEVEL_FROM_AREA_COEFFICIENT_SQRT
        ).powi(2);
        let rx_signal_strength = numerator / denumerator; 

        let signal_level_inner = SignalLevelInner::from(rx_signal_strength);

        Self(signal_level_inner)
    }

    pub fn lower_level(&self) -> Self {
        match self.0 {
            SignalLevelInner::Black(_) | SignalLevelInner::Red(_) => 
                BLACK_SIGNAL_LEVEL,
            SignalLevelInner::Yellow(_) => RED_SIGNAL_LEVEL,
            SignalLevelInner::Green(_) => YELLOW_SIGNAL_LEVEL
        }
    }

    pub fn higher_level(&self) -> Self {
        match self.0 {
            SignalLevelInner::Black(_) => RED_SIGNAL_LEVEL,
            SignalLevelInner::Red(_) => YELLOW_SIGNAL_LEVEL,
            SignalLevelInner::Yellow(_) | SignalLevelInner::Green(_)
                => GREEN_SIGNAL_LEVEL 
        }
    }

    pub fn same_level(&self, other: &Self) -> bool {
        (self.is_black() && other.is_black()) 
            || (self.is_red() && other.is_red()) 
            || (self.is_yellow() && other.is_yellow()) 
            || (self.is_green() && other.is_green()) 
    }

    pub fn is_black(&self) -> bool {
        match self.0 {
            SignalLevelInner::Black(_) => true,
            _ => false
        }
    }
    
    pub fn is_red(&self) -> bool {
        match self.0 {
            SignalLevelInner::Red(_) => true,
            _ => false
        }
    }
    
    pub fn is_yellow(&self) -> bool {
        match self.0 {
            SignalLevelInner::Yellow(_) => true,
            _ => false
        }
    }

    pub fn is_green(&self) -> bool {
        match self.0 {
            SignalLevelInner::Green(_) => true,
            _ => false
        }
    }

    pub fn strength(&self) -> SignalStrength {
        match self.0 {
            SignalLevelInner::Black(strength) => strength,
            SignalLevelInner::Red(strength) => strength,
            SignalLevelInner::Yellow(strength) => strength,
            SignalLevelInner::Green(strength) => strength
        }
    }
}

impl Sub for SignalLevel {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(SignalLevelInner::from(self.strength() - rhs.strength()))
    }
}

impl Sub<SignalStrength> for SignalLevel {
    type Output = Self;

    fn sub(self, rhs: SignalStrength) -> Self::Output {
        Self(SignalLevelInner::from(self.strength() - rhs))
    }
}

impl Add for SignalLevel {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(SignalLevelInner::from(self.strength() + rhs.strength()))
    }
}

impl Add<SignalStrength> for SignalLevel {
    type Output = Self;

    fn add(self, rhs: SignalStrength) -> Self::Output {
        Self(SignalLevelInner::from(self.strength() + rhs))
    }
}

impl Sub<MessageCost> for SignalLevel {
    type Output = Self;

    fn sub(self, rhs: MessageCost) -> Self::Output {
        Self(SignalLevelInner::from(self.strength() - rhs.value()))
    }
}

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

impl From<SignalArea> for SignalLevel {
    fn from(signal_area: SignalArea) -> Self {
        let numerator = (
            signal_area.radius + SIGNAL_LEVEL_FROM_AREA_COEFFICIENT_SQRT
        ).powi(2);
        let signal_strength = numerator / SIGNAL_LEVEL_FROM_AREA_COEFFICIENT;

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
    fn increasing_signal_level() {
        assert!(
            BLACK_SIGNAL_LEVEL
                .higher_level()
                .is_red()
        );
        assert!(
            RED_SIGNAL_LEVEL
                .higher_level()
                .is_yellow()
        );
        assert!(
            YELLOW_SIGNAL_LEVEL
                .higher_level()
                .is_green()
        );
        assert!(
            GREEN_SIGNAL_LEVEL
                .higher_level()
                .is_green()
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
}
