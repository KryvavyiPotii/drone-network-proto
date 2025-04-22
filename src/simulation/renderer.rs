use std::ops::Range;

use full_palette::{GREY, PINK};
use plotters::coord::{ranged3d::Cartesian3d, types::RangedCoordf64, Shift};
use plotters::prelude::*;
use plotters::style::RGBColor;
use thiserror::Error;

use crate::signal::{
    SignalLevel, GPS_L1_FREQUENCY, GPS_L2_FREQUENCY, WIFI_2_4GHZ_FREQUENCY
};
use crate::device::{
    CommandCenter, DESTINATION_RADIUS, Device, Drone, ElectronicWarfare, 
    STEP_DURATION, Transceiver, Transmitter, 
};
use crate::device::networkmodel::{
    NetworkModel, get_drone_networks_destinations
};
use crate::mathphysics::{Megahertz, Meter, Point3D, Position};


const COMMAND_CENTER_RADIUS: Meter = 5.0;

const METRES_TO_PIXELS_SCALE_CONSTANT: f32 = 400.0;
const PLOT_MARGIN: u32                     = 20;

const PLOTTERS_DESTINATION_COLOR: RGBColor    = YELLOW;
const PLOTTERS_COMMAND_CENTER_COLOR: RGBColor = BLUE;


fn font_size(screen_width: u32) -> u32 {
    screen_width / 15
}

fn get_destination_primitive( 
    destination: &Point3D,
    screen_height: u32,
) -> Circle<(f64, f64, f64), u32> {
    let point = plotters_point_from_point3d(destination);
    let radius = meters_to_pixels(
        DESTINATION_RADIUS,
        screen_height
    );

    Circle::new(point, radius, PLOTTERS_DESTINATION_COLOR)
}

fn get_command_center_primitive(
    command_center: &CommandCenter,
    screen_height: u32,
) -> Circle<(f64, f64, f64), u32> {
    let point = plotters_point_from_point3d(command_center.position());
    let radius = meters_to_pixels(
        COMMAND_CENTER_RADIUS,
        screen_height
    );  
    
    Circle::new(point, radius, PLOTTERS_COMMAND_CENTER_COLOR)
}

fn get_drone_primitive(
    drone: &Drone,
    coloring: DroneColoring
) -> Circle<(f64, f64, f64), u32> {
    let point = plotters_point_from_point3d(drone.position());
    let color = get_drone_color(drone, coloring);

    Circle::new(point, 1, color)
}

fn get_ewd_primitive(
    ewd: &ElectronicWarfare,
    frequency: Megahertz,
    screen_height: u32
) -> Circle<(f64, f64, f64), u32> {
    let radius = ewd
        .area(frequency)
        .radius();

    let point = plotters_point_from_point3d(ewd.position());
    let ewd_coverage = meters_to_pixels(radius, screen_height);
    let area_color = match frequency {
        GPS_L1_FREQUENCY | GPS_L2_FREQUENCY => RED,
        WIFI_2_4GHZ_FREQUENCY => BLUE,
        _ => GREY
    };

    Circle::new(point, ewd_coverage, area_color)
}

fn plotters_point_from_point3d(point: &Point3D) -> (f64, f64, f64) {
    (    
        f64::from(point.x), 
        f64::from(point.z), 
        f64::from(point.y), 
    )
}

fn meters_to_pixels(
    value: Meter,
    screen_height: u32
) -> u32 {
    // This value is very rough because it does not 
    // consider the perspective.
    let coef = screen_height as f32 / METRES_TO_PIXELS_SCALE_CONSTANT;

    (value * coef) as u32 
}

fn color_by_infection(infected: bool) -> RGBColor {
    if infected {
        PINK
    } else {
        BLACK
    }
}

fn color_by_signal(signal_level: SignalLevel) -> RGBColor {
    if signal_level.is_green() {
        GREEN
    } else if signal_level.is_yellow() {
        YELLOW
    } else if signal_level.is_red() {
        RED
    } else {
        BLACK
    }
}

fn get_drone_color(drone: &Drone, coloring: DroneColoring) -> RGBColor {
    match coloring {
        DroneColoring::Infection => color_by_infection(drone.is_infected()),
        DroneColoring::Signal => {
            let signal_level = drone.rx_signal_level(WIFI_2_4GHZ_FREQUENCY);
            
            color_by_signal(*signal_level)
        },
        DroneColoring::SingleColor(r, g, b) => {
            RGBColor(r, g, b)
        }
    }
}


#[derive(Debug, Clone)]
pub struct Axes3DRanges {
    x: Range<f64>,
    y: Range<f64>,
    z: Range<f64>
}

impl Axes3DRanges {
    #[must_use]
    pub fn new(
        x: Range<f64>,
        y: Range<f64>,
        z: Range<f64>
    ) -> Self {
        Self { x, y, z }
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


#[derive(Debug, Error)]
pub enum DrawError {
    #[error("Color count does not match network count")]
    NotMatchingColorNumber
}


#[derive(Clone, Copy)]
pub enum DroneColoring {
    Infection,
    Signal,
    SingleColor(u8, u8, u8),
}


pub struct PlottersRenderer<'a> {
    output_filename: String,
    caption: String,
    screen_resolution: (u32, u32),
    axes_ranges: Axes3DRanges,
    drone_colors: Vec<DroneColoring>,
    area: DrawingArea<BitMapBackend<'a>, Shift>, 
    chart: ChartContext<
        'a, 
        BitMapBackend<'a>, 
        Cartesian3d<RangedCoordf64, RangedCoordf64, RangedCoordf64>
    >,
    pitch: f64,
    yaw: f64
}

impl<'a> PlottersRenderer<'a> {
    /// # Panics
    ///
    /// Will panic if an error occurs during bitmap backend creation. 
    #[must_use]
    pub fn new(
        output_filename: &str,
        caption: &str,
        screen_resolution: (u32, u32),
        axes_ranges: Axes3DRanges,
        drone_colors: &[DroneColoring],
        pitch: f64,
        yaw: f64
    ) -> Self {
        let output_filename = output_filename.to_string();
        let area = BitMapBackend::gif(
            &output_filename, 
            screen_resolution,
            STEP_DURATION
        ).unwrap().into_drawing_area();
        let chart = Self::create_chart_context(
            &area, 
            caption,
            font_size(screen_resolution.0),
            &axes_ranges,
            pitch,
            yaw
        ); 

        Self {
            output_filename,
            caption: caption.to_string(),
            screen_resolution,
            axes_ranges,
            drone_colors: drone_colors.to_vec(),
            area,
            chart,
            pitch,
            yaw
        }
    }

    #[must_use]
    pub fn output_filename(&self) -> String {
        self.output_filename.clone()
    }

    /// # Panics
    ///
    /// Will panic if an error occurs during chart context creation. 
    fn create_chart_context(
        area: &DrawingArea<BitMapBackend<'a>, Shift>, 
        caption: &str,
        font_size: u32,
        axes_ranges: &Axes3DRanges,
        pitch: f64,
        yaw: f64
    ) -> ChartContext<
        'a, 
        BitMapBackend<'a>, 
        Cartesian3d<RangedCoordf64, RangedCoordf64, RangedCoordf64>
    > {
        let mut chart = ChartBuilder::on(area)
            .margin(PLOT_MARGIN)
            .caption(caption, ("sans-serif", font_size))
            .build_cartesian_3d(
                axes_ranges.x.clone(),
                axes_ranges.y.clone(),
                axes_ranges.z.clone(),
            ).unwrap();

        chart.with_projection(|mut p| {
            p.pitch = pitch;
            p.yaw = yaw;
            p.into_matrix()
        });

        chart
    }

    fn reset_chart_context(&mut self) {
        self.chart = Self::create_chart_context(
            &self.area, 
            &self.caption,
            font_size(self.screen_resolution.0), 
            &self.axes_ranges,
            self.pitch,
            self.yaw
        );
    }

    /// # Errors
    ///
    /// Will return Err if the number of networks does not match the number of
    /// colors.
    ///
    /// # Panics
    ///
    /// Will panic, if `draw` returns Err.
    pub fn draw_drone_networks(
        &mut self, 
        drone_networks: &[NetworkModel]
    ) -> Result<(), DrawError> {
        if self.drone_colors.len() != drone_networks.len() {
            return Err(DrawError::NotMatchingColorNumber);
        }
        
        self.reset_chart_context();
        self.chart.configure_axes().draw().unwrap();
        
        let destinations = get_drone_networks_destinations(drone_networks);
        self.draw_destinations(&destinations);
        self.draw_command_centers(drone_networks);
        self.draw_drones(drone_networks);
        for drone_network in drone_networks {
            self.draw_ewds(drone_network.ewds());
        }

        self.area.present().unwrap();
        self.area.fill(&WHITE).unwrap();

        Ok(())
    }

    fn draw_destinations(&mut self, destinations: &[&Point3D]) {
        self.chart.draw_series(
            destinations
                .iter()
                .map(|destination| 
                    get_destination_primitive(
                        destination, 
                        self.screen_resolution.1
                    )
                )
        ).unwrap();
    }
    
    fn draw_command_centers(
        &mut self, 
        drone_networks: &[NetworkModel]
    ) {
        self.chart.draw_series(
            drone_networks
                .iter()
                .map(|drone_network| 
                    get_command_center_primitive(
                        drone_network.command_center(), 
                        self.screen_resolution.1
                    )
                )
        ).unwrap();
    }

    fn draw_drones(
        &mut self, 
        drone_networks: &[NetworkModel]
    ) {
        for (drone_network, coloring) in drone_networks
            .iter()
            .zip(self.drone_colors.iter())
        {
            self.chart.draw_series(
                drone_network
                    .drone_iter()
                    .map(|drone| get_drone_primitive(drone, *coloring))
            ).unwrap();
        }
    }

    fn draw_ewds(&mut self, ewds: &[ElectronicWarfare]) {
        for ewd in ewds {
            self.draw_ewd(ewd, WIFI_2_4GHZ_FREQUENCY);
            self.draw_ewd(ewd, GPS_L1_FREQUENCY);
        }
    }

    fn draw_ewd(&mut self, ewd: &ElectronicWarfare, frequency: Megahertz) {
        self.chart.draw_series([
            get_ewd_primitive(
                ewd, 
                frequency, 
                self.screen_resolution.1
            )
        ]).unwrap();
    }
}
