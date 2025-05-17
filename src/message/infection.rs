use crate::signal::{SignalLevel, YELLOW_SIGNAL_LEVEL};
use crate::mathphysics::{Megahertz, Millisecond};


pub const INFECTION_DELAY: Millisecond      = 500;
pub const JAMMING_SIGNAL_LEVEL: SignalLevel = YELLOW_SIGNAL_LEVEL;


#[derive(Debug, Clone, Copy)]
pub enum InfectionState {
    Vulnerable,
    Infected,
    Patched
}

// Infection parameters:
// * spread to connected devices - yes, limited or no
// * speed of spreading
// * tampering with drone data 
// Ideas:
// * command center spoofing
// * changing destination | goal | global position
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum InfectionType { 
    Indicator,
    Jamming(Megahertz),
}
