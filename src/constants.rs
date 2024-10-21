// TODO pub const MAX_HEIGHT_IN_METRES: f32           = 50.0;
pub const MAX_DRONE_SPEED_IN_METRES_PER_S: f32      = 25.0;
pub const DRONE_MAX_BROADCAST_RADIUS_IN_METRES: f32 = 10.0;
pub const MAX_SIGNAL_SPEED_IN_METRES_PER_S: f32     = 200.0;
pub const ID_RANGE: u32                             = 100000;

pub const MAX_TIME_IN_MILLIS: u64      = 15000;
pub const STEP_DURATION_IN_MILLIS: u64 = 100;

pub const CC_CONTROL_RADIUS_IN_METRES: f32  = 200.0;
pub const RWD_CONTROL_RADIUS_IN_METRES: f32 = 25.0;
pub const RWD_GPS_RADIUS_IN_METRES: f32     = 50.0;
pub const DESTINATION_RADIUS_IN_METRES: f32 = 5.0;

pub const INITIAL_DRONE_POSITION: (f32, f32, f32) = (150.3, 90.6, 25.5);
pub const INITIAL_DRONE_DESTINATION: (f32, f32, f32) = (0.0, 0.0, 0.0);
pub const INITIAL_COMMAND_CENTER_POSITION: (f32, f32, f32) = (200.0, 100.0, 0.0);

pub const PLOT_FILE_PATH: &str = "simulation.gif";
pub const PLOT_RESOLUTION_WH_IN_PIXELS: (u32, u32) = (800, 700); 
pub const PLOT_FONT_SIZE: u32 = PLOT_RESOLUTION_WH_IN_PIXELS.0 / 15;

// This value is very rough because it does not consider the perspective.
// TODO change scale calculation method or visualization method.
pub const METRES_TO_PIXELS_SCALE: f32 = PLOT_RESOLUTION_WH_IN_PIXELS.1 as f32
    / 400.0;

pub fn _convert_metres_to_pixels(value_in_metres: f32) -> u32 {
    (value_in_metres * METRES_TO_PIXELS_SCALE) as u32 
}
