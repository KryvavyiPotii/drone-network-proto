use std::ops::Range;

use full_palette::{GREEN_400, GREY, PINK, RED_400, YELLOW_700};
use plotters::coord::{ranged3d::Cartesian3d, types::RangedCoordf64, Shift};
use plotters::prelude::*;
use plotters::style::RGBColor;
use thiserror::Error;

use crate::signal::{
    SignalLevel, GPS_L1_FREQUENCY, GPS_L2_FREQUENCY, WIFI_2_4GHZ_FREQUENCY
};
use crate::device::{Device, DESTINATION_RADIUS, STEP_DURATION};
use crate::device::networkmodel::NetworkModel;
use crate::mathphysics::{Megahertz, Meter, Point3D, Position};
use crate::message::Goal;


const COMMAND_CENTER_RADIUS: Meter = 5.0;

const METRES_TO_PIXELS_SCALE_CONSTANT: f32 = 400.0;
const CIRCLE_SIZE_COEFFICIENT: u32         = 400;
const PLOT_MARGIN: u32                     = 20;

const PLOTTERS_DESTINATION_COLOR: RGBColor    = YELLOW;
const PLOTTERS_COMMAND_CENTER_COLOR: RGBColor = BLUE;


type PlottersChartContext<'a> = ChartContext<
    'a, 
    BitMapBackend<'a>, 
    Cartesian3d<RangedCoordf64, RangedCoordf64, RangedCoordf64>
>;
type PlottersCircle = Circle<(f64, f64, f64), u32>; 


fn network_models_destinations(
    network_models: &[NetworkModel]
) -> Vec<Point3D> {
    let mut destinations = Vec::new();

    for network_model in network_models {
        let goal_vec: Vec<Goal> = network_model
            .goals()
            .values()
            .copied()
            .collect();

        goal_vec
            .iter()
            .for_each(|goal|
                match goal {
                    Goal::Attack(destination) | Goal::Reposition(destination) =>
                        destinations.push(*destination),
                    Goal::Undefined => (),
                }
            );
    }

    destinations
}

fn font_size(screen_width: u16) -> u16 {
    screen_width / 15
}

fn destination_primitive( 
    destination: &Point3D,
    screen_height: u16,
) -> PlottersCircle {
    let point = plotters_point_from_point3d(destination);
    let radius = meters_to_pixels(
        DESTINATION_RADIUS,
        screen_height
    );

    Circle::new(point, radius, PLOTTERS_DESTINATION_COLOR)
}

fn command_device_primitive(
    command_device: &Device,
    screen_height: u16,
) -> PlottersCircle {
    let point = plotters_point_from_point3d(command_device.position());
    let radius = meters_to_pixels(
        COMMAND_CENTER_RADIUS,
        screen_height
    );  
    
    Circle::new(point, radius, PLOTTERS_COMMAND_CENTER_COLOR)
}

fn device_primitive(
    drone: &Device,
    coloring: DeviceColoring,
    screen_height: u16
) -> PlottersCircle {
    let screen_height = u32::from(screen_height);

    let point = plotters_point_from_point3d(drone.position());
    let color = get_drone_color(drone, coloring);
    let style = Into::<ShapeStyle>::into(color).filled();
    let size  = if screen_height < CIRCLE_SIZE_COEFFICIENT {
        1  
    } else {
        screen_height / CIRCLE_SIZE_COEFFICIENT
    };

    Circle::new(point, size, style)
}

fn ewd_primitive(
    ewd: &Device,
    frequency: Megahertz,
    screen_height: u16
) -> PlottersCircle {
    let radius = ewd
        .area(frequency)
        .radius();

    let point = plotters_point_from_point3d(ewd.position());
    let ewd_coverage = meters_to_pixels(radius, screen_height);
    let area_color = match frequency {
        GPS_L1_FREQUENCY | GPS_L2_FREQUENCY => RED,
        WIFI_2_4GHZ_FREQUENCY               => BLUE,
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

#[allow(clippy::cast_sign_loss)]
#[allow(clippy::cast_possible_truncation)]
fn meters_to_pixels(
    value: Meter,
    screen_height: u16
) -> u32 {
    // This value is very rough because it does not 
    // consider the perspective.
    let coef = f32::from(screen_height) / METRES_TO_PIXELS_SCALE_CONSTANT;

    (value * coef).round() as u32 
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
        GREEN_400
    } else if signal_level.is_yellow() {
        YELLOW_700
    } else if signal_level.is_red() {
        RED_400
    } else {
        BLACK
    }
}

fn get_drone_color(drone: &Device, coloring: DeviceColoring) -> RGBColor {
    match coloring {
        DeviceColoring::Infection            => {
            color_by_infection(drone.is_infected())
        },
        DeviceColoring::Signal               => {
            let signal_level = drone.rx_signal_level(WIFI_2_4GHZ_FREQUENCY);
            
            color_by_signal(*signal_level)
        },
        DeviceColoring::SingleColor(r, g, b) => {
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
pub enum RenderError {
    #[error("Color count does not match network count")]
    NotMatchingColorNumber
}


#[derive(Clone, Copy)]
pub enum DeviceColoring {
    Infection,
    Signal,
    SingleColor(u8, u8, u8),
}


pub struct PlottersRenderer<'a> {
    output_filename: String,
    caption: String,
    screen_resolution: (u16, u16),
    font_size: u16,
    axes_ranges: Axes3DRanges,
    drone_colors: Vec<DeviceColoring>,
    area: DrawingArea<BitMapBackend<'a>, Shift>, 
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
        screen_resolution: (u16, u16),
        axes_ranges: Axes3DRanges,
        drone_colors: &[DeviceColoring],
        pitch: f64,
        yaw: f64
    ) -> Self {
        let font_size = font_size(screen_resolution.0);
        let output_filename = output_filename.to_string();

        let plotters_screen_resolution = (
            u32::from(screen_resolution.0),
            u32::from(screen_resolution.1),
        );
        let area = BitMapBackend::gif(
            &output_filename, 
            plotters_screen_resolution,
            STEP_DURATION.try_into().expect("Failed to convert i32 to u32")
        )
            .expect("Failed to create `BitMapBackend`")
            .into_drawing_area();

        Self {
            output_filename,
            caption: caption.to_string(),
            screen_resolution,
            font_size,
            axes_ranges,
            drone_colors: drone_colors.to_vec(),
            area,
            pitch,
            yaw
        }
    }

    #[must_use]
    pub fn output_filename(&self) -> String {
        self.output_filename.clone()
    }

    /// # Errors
    ///
    /// Will return `Err` if the number of networks does not match the number of
    /// colors.
    /// 
    /// # Panics
    ///
    /// Will panic if an error occurs during drawing.
    pub fn render(
        &mut self, 
        network_models: &[NetworkModel]
    ) -> Result<(), RenderError> {
        if self.drone_colors.len() != network_models.len() {
            return Err(RenderError::NotMatchingColorNumber);
        }
        self.area.fill(&WHITE).expect("Failed to fill an area");
        
        let mut chart_context = self.chart_context();

        self.draw_chart(&mut chart_context);
        self.draw_network_models(network_models, &mut chart_context);

        self.area.present().expect("Failed to finalize drawing");

        Ok(())
    }
    
    fn chart_context(&self) -> PlottersChartContext<'a> {
        let mut chart = ChartBuilder::on(&self.area)
            .margin(PLOT_MARGIN)
            .caption(&self.caption, ("sans-serif", u32::from(self.font_size)))
            .build_cartesian_3d(
                self.axes_ranges.x.clone(),
                self.axes_ranges.y.clone(),
                self.axes_ranges.z.clone(),
            )
            .expect("Failed to create a chart");

        chart
            .with_projection(|mut p| {
                p.pitch = self.pitch;
                p.yaw = self.yaw;
                p.into_matrix()
            });

        chart
    }

    fn draw_network_models(
        &self,
        network_models: &[NetworkModel],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        let destinations = network_models_destinations(network_models);
        self.draw_destinations(&destinations, chart_context);
        self.draw_command_devices(network_models, chart_context);
        self.draw_devices(network_models, chart_context);
        for network_model in network_models {
            self.draw_ewds(network_model.ewds(), chart_context);
        }
    }

    fn draw_chart(&self, chart_context: &mut PlottersChartContext<'a>) {
        chart_context 
            .configure_axes()
            .axis_panel_style(GREY.mix(0.2))
            .label_style(("sans-serif", self.font_size / 2))
            .draw()
            .expect("Failed to draw a chart");
    }
    
    fn draw_destinations(
        &self, 
        destinations: &[Point3D],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        let destination_primitives = destinations
            .iter()
            .map(|destination| 
                destination_primitive(
                    destination, 
                    self.screen_resolution.1
                )
            );

        chart_context
            .draw_series(destination_primitives)
            .expect("Failed to draw destination points");
    }
    
    fn draw_command_devices(
        &self, 
        network_models: &[NetworkModel],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        let network_model_primitives = network_models
            .iter()
            .map(|network_model| 
                command_device_primitive(
                    network_model.command_device(), 
                    self.screen_resolution.1
                )
            );

        chart_context
            .draw_series(network_model_primitives)
            .expect("Failed to draw destination points");
    }

    fn draw_devices(
        &self, 
        network_models: &[NetworkModel],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        for (network_model, coloring) in network_models
            .iter()
            .zip(self.drone_colors.iter())
        {
            let device_primitives = network_model
                .device_iter()
                .map(|device| 
                    device_primitive(
                        device, 
                        *coloring, 
                        self.screen_resolution.0
                    )
                );

            chart_context.draw_series(device_primitives).unwrap();
        }
    }

    fn draw_ewds(
        &self, 
        ewds: &[Device],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        let control_ewd_primitives = ewds
            .iter()
            .map(|ewd| {
                ewd_primitive(
                    ewd, 
                    WIFI_2_4GHZ_FREQUENCY, 
                    self.screen_resolution.1
                )
            });
        let gps_ewd_primitives = ewds
            .iter()
            .map(|ewd| {
                ewd_primitive(
                    ewd, 
                    GPS_L1_FREQUENCY, 
                    self.screen_resolution.1
                )
            });

        chart_context
            .draw_series(control_ewd_primitives)
            .expect("Failed to draw control EWDs");
        chart_context
            .draw_series(gps_ewd_primitives)
            .expect("Failed to draw GPS EWDs");
    }
}
