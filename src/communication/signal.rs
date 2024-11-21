use crate::device::STEP_DURATION_IN_MILLIS;


#[derive(Clone, Copy, Hash, Eq, PartialEq, Debug)]
pub enum SignalType {
    GPS,
    Control,
}

#[derive(Clone, Copy, Debug)]
pub enum SignalAreaType {
    Dome(f32),
    // TODO Rifle,
}

// For ComplexNetwork it is a boolean value of control connection:
//     Black == false, others == true
// For CellularAutomaton it is a part of a cell's state.
#[derive(Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum SignalLevel {
    Black,  // no signal
    Red,    // signal level is critically low
    Yellow, // signal level is decent
    Green,  // signal level is good
}

impl SignalLevel {
    pub fn propagation(tx_level: &Self, rx_level: &Self) -> Self {
        match (tx_level, rx_level) {
            (Self::Green, _) => rx_level.clone(),
            (_, Self::Green) => tx_level.clone(),
            (Self::Yellow, Self::Yellow) => Self::Red,
            _ => Self::Black,
        }
    }

    pub fn suppression(tx_level: &Self, rx_level: &Self) -> Self {
        match (tx_level, rx_level) {
            (Self::Black, _) => rx_level.clone(),
            (Self::Red, Self::Green) => Self::Yellow,
            (Self::Yellow, Self::Green) | (Self::Red, Self::Yellow) =>
                Self::Red,
            _ => Self::Black,
        }
    }
}
