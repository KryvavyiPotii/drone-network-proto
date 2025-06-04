use mathphysics::{Megahertz, Meter, Millisecond};
use signal::WIFI_2_4GHZ_FREQUENCY;


pub mod connections;
pub mod device;
pub mod message;
pub mod malware;
pub mod mathphysics;
pub mod networkmodel;
pub mod signal;


pub const CONTROL_FREQUENCY: Megahertz = WIFI_2_4GHZ_FREQUENCY;
pub const DESTINATION_RADIUS: Meter    = 5.0;
pub const ITERATION_TIME: Millisecond  = 50;
