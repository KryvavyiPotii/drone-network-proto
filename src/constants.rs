use crate::entities::*;


//pub const MAX_HEIGHT_IN_METRES: f32          = 50.0;
pub const MAX_SPEED_IN_METRES_PER_S: f32     = 25.0;
pub const MAX_TIME_IN_MILLIS: u64            = 15000;
pub const STEP_DURATION_IN_MILLIS: u64       = 100;

pub const RWD_RADIUS_IN_METRES: f32          = 50.0;
pub const DESTINATION_RADIUS_IN_METRES: f32  = 5.0;

pub const INITIAL_DRONE_POSITION: Coordinates3D = Coordinates3D {
    x: 150.3,
    y: 90.6,
    z: 25.5, 
};
pub const INITIAL_DRONE_DESTINATION: Coordinates3D = Coordinates3D {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};
pub const INITIAL_COMMAND_CENTER_POSITION: Coordinates3D = Coordinates3D {
    x: 200.0,
    y: 100.0,
    z: 0.0,
};

pub const PLOT_FILE_PATH: &str = "simulation.gif";
pub const PLOT_RESOLUTION_WH_IN_PIXELS: (u32, u32) = (800, 700); 
pub const PLOT_FONT_SIZE: u32 = PLOT_RESOLUTION_WH_IN_PIXELS.0 / 15;
pub const METRES_TO_PIXELS_SCALE: f32 = PLOT_RESOLUTION_WH_IN_PIXELS.1 as f32
    / 400.0;

pub fn _convert_metres_to_pixels(value_in_metres: f32) -> u32 {
    (value_in_metres * METRES_TO_PIXELS_SCALE) as u32 
}
