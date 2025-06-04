pub type Millisecond = i32;
pub type Second = f32;
pub type Meter = f32;
pub type Kilometer = f32;
pub type KilometerPerSecond = f32;
pub type MeterPerMillisecond = f32;
pub type MeterPerSecond = f32;
pub type Megahertz = u32;
pub type PowerUnit = u32;


// Const for conversion from km / (s * MHz) to m / (s * Hz).
const CONVERSION_CONST: f32 = 1_000.0;

pub const SPEED_OF_LIGHT: KilometerPerSecond = 300_000.0;


#[must_use]
pub fn millis_to_secs(millis: Millisecond) -> Second {
    millis as Second / 1_000.0 
}

#[must_use]
pub fn secs_to_millis(secs: Second) -> Millisecond {
    (secs * 1_000.0).round() as Millisecond
}

#[must_use]
pub fn mps_to_mpms(mps: MeterPerSecond) -> MeterPerMillisecond {
    mps as MeterPerMillisecond / 1_000.0 
}

#[must_use]
pub fn kmps_to_mpms(kmps: KilometerPerSecond) -> MeterPerMillisecond {
    kmps as MeterPerMillisecond
}

#[must_use]
pub fn mpms_to_mps(mpms: MeterPerMillisecond) -> MeterPerSecond {
    mpms as MeterPerSecond * 1_000.0
}

#[must_use]
pub fn time_in_secs_from_distance_and_speed(
    distance: Meter, 
    speed: MeterPerSecond
) -> Second {
    distance / speed
}

#[must_use]
pub fn time_in_millis_from_distance_and_speed(
    distance: Meter, 
    speed: MeterPerMillisecond
) -> Millisecond {
    (distance / speed).round() as Millisecond
}

#[must_use]
pub fn wave_length_in_meters(frequency: Megahertz) -> Meter {
    SPEED_OF_LIGHT / (frequency as KilometerPerSecond * CONVERSION_CONST) 
}
