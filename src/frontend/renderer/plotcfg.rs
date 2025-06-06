use std::ops::Range;

use crate::backend::mathphysics::{Meter, Point3D};


pub const PLOT_MARGIN: u32 = 20;

const METERS_TO_PIXELS_SCALE_COEF: f64 = 400.0;


pub fn font_size(plot_resolution: PlotResolution) -> u32 {
    plot_resolution.width / 15
}

#[allow(clippy::cast_sign_loss)]
#[allow(clippy::cast_possible_truncation)]
pub fn meters_to_pixels(
    value_in_meters: Meter,
    plot_resolution: PlotResolution,
) -> u32 {
    // This value is very rough because it does not 
    // consider the perspective.
    let coef = f64::from(plot_resolution.height()) 
        / METERS_TO_PIXELS_SCALE_COEF;

    (f64::from(value_in_meters) * coef).round() as u32 
}

pub fn plotters_point_from_point3d(point: &Point3D) -> (f64, f64, f64) {
    (    
        f64::from(point.x), 
        f64::from(point.z), 
        f64::from(point.y), 
    )
}


#[derive(Debug, Clone)]
pub struct Axes3DRanges {
    x: Range<f64>,
    y: Range<f64>,
    z: Range<f64>
}

impl Axes3DRanges {
    #[must_use]
    pub fn new(x: Range<f64>, y: Range<f64>, z: Range<f64>) -> Self {
        Self { x, y, z }
    }

    #[must_use]
    pub fn x(&self) -> Range<f64> {
        self.x.clone()
    }
    
    #[must_use]
    pub fn y(&self) -> Range<f64> {
        self.y.clone()
    }
    
    #[must_use]
    pub fn z(&self) -> Range<f64> {
        self.z.clone()
    }
}

impl Default for Axes3DRanges {
    fn default() -> Self {
        Self {
            x: 0.0..200.0,
            y: 0.0..200.0,
            z: 0.0..200.0,
        }
    }
}


#[derive(Clone, Copy)]
pub struct PlotResolution {
    width: u32,
    height: u32
}

impl PlotResolution {
    #[must_use]
    pub fn new(
        width: u32,
        height: u32
    ) -> Self {
        Self { width, height }
    }

    #[must_use]
    pub fn width(&self) -> u32 {
        self.width
    }
    
    #[must_use]
    pub fn height(&self) -> u32 {
        self.height
    }
}

impl From<PlotResolution> for (u32, u32) {
    fn from(plot_resolution: PlotResolution) -> Self {
        (plot_resolution.width(), plot_resolution.height())
    }
}


#[derive(Clone, Copy)]
pub enum DeviceColoring {
    Infection,
    Signal,
    SingleColor(u8, u8, u8),
}


#[derive(Clone, Copy)]
pub struct CameraAngle {
    pitch: f64,
    yaw: f64
}

impl CameraAngle {
    #[must_use]
    pub fn new(pitch: f64, yaw: f64) -> Self {
        Self { pitch, yaw }
    }
    
    #[must_use]
    pub fn pitch(&self) -> f64 {
        self.pitch
    }
    
    #[must_use]
    pub fn yaw(&self) -> f64 {
        self.yaw
    }
}
