use std::ops::Range;

use crate::backend::mathphysics::{Meter, Point3D};


pub const PLOT_MARGIN: Pixel = 20;

const METERS_TO_PIXELS_SCALE_COEF: PlottersUnit = 400.0;


pub type Pixel         = u32;
pub type PlottersUnit  = f64;


#[must_use]
pub fn font_size(plot_resolution: PlotResolution) -> Pixel {
    plot_resolution.width / 15
}

#[must_use]
#[allow(clippy::cast_sign_loss)]
#[allow(clippy::cast_possible_truncation)]
pub fn meters_to_pixels(
    value_in_meters: Meter,
    plot_resolution: PlotResolution,
) -> Pixel {
    // This value is very rough because it does not 
    // consider the perspective.
    let coef = PlottersUnit::from(plot_resolution.height()) 
        / METERS_TO_PIXELS_SCALE_COEF;

    (PlottersUnit::from(value_in_meters) * coef).round() as Pixel 
}


#[derive(Debug, Clone)]
pub struct Axes3DRanges {
    x: Range<PlottersUnit>,
    y: Range<PlottersUnit>,
    z: Range<PlottersUnit>
}

impl Axes3DRanges {
    #[must_use]
    pub fn new(
        x: Range<PlottersUnit>, 
        y: Range<PlottersUnit>, 
        z: Range<PlottersUnit>
    ) -> Self {
        Self { x, y, z }
    }

    #[must_use]
    pub fn x(&self) -> Range<PlottersUnit> {
        self.x.clone()
    }
    
    #[must_use]
    pub fn y(&self) -> Range<PlottersUnit> {
        self.y.clone()
    }
    
    #[must_use]
    pub fn z(&self) -> Range<PlottersUnit> {
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
pub struct PlottersPoint3D((PlottersUnit, PlottersUnit, PlottersUnit));

impl From<Point3D> for PlottersPoint3D {
    fn from(point3d: Point3D) -> Self {
        let coordinates = (    
            PlottersUnit::from(point3d.x), 
            PlottersUnit::from(point3d.z), 
            PlottersUnit::from(point3d.y), 
        );

        Self(coordinates)
    }
}

impl From<&Point3D> for PlottersPoint3D {
    fn from(point3d: &Point3D) -> Self {
        let coordinates = (    
            PlottersUnit::from(point3d.x), 
            PlottersUnit::from(point3d.z), 
            PlottersUnit::from(point3d.y), 
        );

        Self(coordinates)
    }
}

impl From<PlottersPoint3D> for (PlottersUnit, PlottersUnit, PlottersUnit) {
    fn from(plotters_point3d: PlottersPoint3D) -> Self {
        plotters_point3d.0 
    }
}


#[derive(Clone, Copy)]
pub struct PlotResolution {
    width: Pixel,
    height: Pixel
}

impl PlotResolution {
    #[must_use]
    pub fn new(
        width: Pixel,
        height: Pixel
    ) -> Self {
        Self { width, height }
    }

    #[must_use]
    pub fn width(&self) -> Pixel {
        self.width
    }
    
    #[must_use]
    pub fn height(&self) -> Pixel {
        self.height
    }
}

impl From<PlotResolution> for (Pixel, Pixel) {
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
    pitch: PlottersUnit,
    yaw: PlottersUnit
}

impl CameraAngle {
    #[must_use]
    pub fn new(pitch: PlottersUnit, yaw: PlottersUnit) -> Self {
        Self { pitch, yaw }
    }
    
    #[must_use]
    pub fn pitch(&self) -> PlottersUnit {
        self.pitch
    }
    
    #[must_use]
    pub fn yaw(&self) -> PlottersUnit {
        self.yaw
    }
}
