use std::ops;

use derive_more::Mul;
use impl_ops::{
    _impl_binary_op_borrowed_borrowed, _impl_binary_op_borrowed_owned, 
    _impl_binary_op_internal, _impl_binary_op_owned_borrowed, 
    _impl_binary_op_owned_owned, _parse_binary_op, impl_op, impl_op_ex
};

use crate::message::MessageCost;
use crate::signal::{
    GREEN_SIGNAL_ZONE_COEFFICIENT, YELLOW_SIGNAL_ZONE_COEFFICIENT
};


pub const GREEN_SIGNAL_STRENGTH_VALUE: f32           = 100.0;
pub const NO_SIGNAL_STRENGTH: SignalStrength         = SignalStrength(0.0);
pub const MAX_BLACK_SIGNAL_STRENGTH: SignalStrength  = SignalStrength(1.0);
pub const MAX_RED_SIGNAL_STRENGTH: SignalStrength    = SignalStrength(
    GREEN_SIGNAL_STRENGTH_VALUE * (1.0 - YELLOW_SIGNAL_ZONE_COEFFICIENT)
);
pub const MAX_YELLOW_SIGNAL_STRENGTH: SignalStrength = SignalStrength(
    GREEN_SIGNAL_STRENGTH_VALUE * (1.0 - GREEN_SIGNAL_ZONE_COEFFICIENT)
);
pub const GREEN_SIGNAL_STRENGTH: SignalStrength = SignalStrength(
    GREEN_SIGNAL_STRENGTH_VALUE
);


#[must_use]
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


#[derive(Clone, Copy, Debug, Default, Mul, PartialEq, PartialOrd)]
pub struct SignalStrength(f32);

impl SignalStrength {
    #[must_use]
    pub fn new(value: f32) -> Self {
        Self(value)
    }

    #[must_use]
    pub fn value(&self) -> f32 {
        self.0
    }
}

impl_op_ex!(
    + |a: &SignalStrength, b: &SignalStrength| -> SignalStrength { 
        SignalStrength(a.0 + b.0)
    }
);
impl_op_ex!(
    + |a: &SignalStrength, b: &MessageCost| -> SignalStrength { 
        SignalStrength(a.0 + b.value()) 
    }
);
impl_op_ex!(
    + |a: &SignalStrength, b: &f32| -> SignalStrength { 
        SignalStrength(a.0 + b) 
    }
);
impl_op_ex!(
    - |a: &SignalStrength, b: &SignalStrength| -> SignalStrength { 
        SignalStrength(a.0 - b.0) 
    }
);
impl_op_ex!(
    - |a: &SignalStrength, b: &MessageCost| -> SignalStrength { 
        SignalStrength(a.0 - b.value()) 
    }
);
impl_op_ex!(
    - |a: &SignalStrength, b: &f32| -> SignalStrength { 
        SignalStrength(a.0 - b) 
    }
);
impl_op_ex!(
    / |a: &SignalStrength, b: &SignalStrength| -> SignalStrength { 
        SignalStrength(a.0 / b.0) 
    }
);
impl_op_ex!(
    / |a: &SignalStrength, b: &MessageCost| -> SignalStrength { 
        SignalStrength(a.0 / b.value()) 
    }
);
impl_op_ex!(
    / |a: &SignalStrength, b: &f32| -> SignalStrength { 
        SignalStrength(a.0 / b) 
    }
);
