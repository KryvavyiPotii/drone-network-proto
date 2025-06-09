use std::collections::HashMap;

use super::mathphysics::Megahertz;


pub use area::*;
pub use level::*;
pub use strength::*;


pub mod area;
pub mod level;
pub mod strength;


pub const GPS_L1_FREQUENCY: Megahertz = 1_575;
pub const WIFI_2_4GHZ_FREQUENCY: Megahertz = 2_400;

pub const RED_SIGNAL_ZONE_COEFFICIENT: f32    = 1.0;
pub const YELLOW_SIGNAL_ZONE_COEFFICIENT: f32 = 0.5;
pub const GREEN_SIGNAL_ZONE_COEFFICIENT: f32  = 0.2;

// Const for proper signal strength scaling at distance.
const SIGNAL_STRENGTH_SCALING: f32 = 2_500.0; 


pub type FreqToLevelMap = HashMap<Megahertz, SignalLevel>;
