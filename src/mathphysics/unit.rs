pub type Millisecond = u32;
pub type Second = f32;
pub type Meter = f32;
pub type MeterPerMillisecond = f32;
pub type MeterPerSecond = f32;
pub type Megahertz = f32;


pub fn millis_to_secs(millis: Millisecond) -> Second {
    millis as Second / 1000.0 
}

pub fn secs_to_millis(secs: Second) -> Millisecond {
    (secs * 1000.0).round() as Millisecond
}

pub fn mps_to_mpms(mps: MeterPerSecond) -> MeterPerMillisecond {
    mps as MeterPerMillisecond / 1000.0 
}

pub fn mpms_to_mps(mpms: MeterPerMillisecond) -> MeterPerSecond {
    mpms as MeterPerSecond * 1000.0
}

pub fn time_in_secs_from_distance_and_speed(
    distance: Meter, 
    speed: MeterPerSecond
) -> Second {
    distance / speed
}

pub fn time_in_millis_from_distance_and_speed(
    distance: Meter, 
    speed: MeterPerMillisecond
) -> MeterPerMillisecond {
    (distance / speed).round() as MeterPerMillisecond
}
