use std::{
    ops::Range,
    sync::{Arc, atomic::{AtomicBool, Ordering}}
};

use full_palette::GREY;
use log::{info, trace};
use plotters::{
    coord::{ranged3d::Cartesian3d, types::RangedCoordf64, Shift}, 
    prelude::*,
    style::RGBColor
};
use thiserror::Error;

use crate::communication::{
    GPS_L1_FREQUENCY, GPS_L2_FREQUENCY, WIFI_2_4GHZ_FREQUENCY
};
use crate::device::{
    CommandCenter, DESTINATION_RADIUS, Drone, ElectronicWarfare, STEP_DURATION, 
    Transceiver, Transmitter, 
    networkmodel::{NetworkModel, get_drone_networks_destinations}
};
use crate::mathphysics::{Megahertz, Meter, Millisecond, Point3D, Position};


pub const START_TIME: Millisecond = 0;
pub const END_TIME: Millisecond   = 15000;

const COMMAND_CENTER_RADIUS: Meter = 5.0;

const METRES_TO_PIXELS_SCALE_CONSTANT: f32 = 400.0;
const PLOT_MARGIN: u32                     = 20;

const PLOTTERS_DESTINATION_COLOR: RGBColor    = YELLOW;
const PLOTTERS_COMMAND_CENTER_COLOR: RGBColor = BLUE;


fn set_ctrlc_handler() -> Arc<AtomicBool> {
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc_async::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    running
}

fn log_drone_count(drone_networks: &[NetworkModel]) -> String {
    let mut output = String::new();

    for (i, network) in drone_networks.iter().enumerate() {
        let entry = format!("Drone count {}: {}, ", i, network.drone_count());
        
        output.push_str(&entry); 
    }

    // Remove trailing comma and space.
    output.truncate(output.len() - 2);

    output
}

fn font_size(screen_width: u32) -> u32 {
    screen_width / 15
}

fn get_destination_primitive( 
    destination: &Point3D,
    screen_height: u32,
) -> Circle<(f64, f64, f64), u32> {
    let point = plotters_point_from_point3d(destination);
    let radius = convert_meters_to_pixels(
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
    let radius = convert_meters_to_pixels(
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
    let point = plotters_point_from_point3d(ewd.position());
    let ewd_coverage = convert_meters_to_pixels(
        ewd
            .area(frequency)
            .radius(), 
        screen_height
    );
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

fn convert_meters_to_pixels(
    value: Meter,
    screen_height: u32
) -> u32 {
    // This value is very rough because it does not 
    // consider the perspective.
    let coef = screen_height as f32 / METRES_TO_PIXELS_SCALE_CONSTANT;

    (value * coef) as u32 
}

fn get_drone_color(drone: &Drone, coloring: DroneColoring) -> RGBColor {
    match coloring {
        DroneColoring::Signal => {
            let signal_level = drone.rx_signal_level(WIFI_2_4GHZ_FREQUENCY);

            if signal_level.is_green() {
                GREEN
            } else if signal_level.is_yellow() {
                YELLOW
            } else if signal_level.is_red() {
                RED
            } else {
                BLACK
            }
        },
        DroneColoring::SingleColor(r, g, b) => {
            RGBColor(r, g, b)
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
    SingleColor(u8, u8, u8),
    Signal,
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


pub struct PlottersRenderer<'a> {
    caption: String,
    screen_resolution: (u32, u32),
    axes_ranges: Axes3DRanges,
    drone_colors: Vec<DroneColoring>,
    area: DrawingArea<BitMapBackend<'a>, Shift>, 
    chart: ChartContext<
        'a, 
        BitMapBackend<'a>, 
        Cartesian3d<RangedCoordf64, RangedCoordf64, RangedCoordf64>
    >
}

impl<'a> PlottersRenderer<'a> {
    /// # Panics
    ///
    /// Will panic if an error occurs during bitmap backend creation. 
    #[must_use]
    pub fn new(
        output_path: &str,
        caption: &str,
        screen_resolution: (u32, u32),
        axes_ranges: Axes3DRanges,
        drone_colors: &[DroneColoring],
    ) -> Self {
        let output_path = output_path.to_string();
        let area = BitMapBackend::gif(
            &output_path, 
            screen_resolution,
            STEP_DURATION
        ).unwrap().into_drawing_area();
        let chart = Self::create_chart_context(
            &area, 
            caption,
            font_size(screen_resolution.0),
            &axes_ranges
        ); 

        Self {
            caption: caption.to_string(),
            screen_resolution,
            axes_ranges,
            drone_colors: drone_colors.to_vec(),
            area,
            chart
        }
    }

    /// # Panics
    ///
    /// Will panic if an error occurs during chart context creation. 
    fn create_chart_context(
        area: &DrawingArea<BitMapBackend<'a>, Shift>, 
        caption: &str,
        font_size: u32,
        axes_ranges: &Axes3DRanges
    ) -> ChartContext<
        'a, 
        BitMapBackend<'a>, 
        Cartesian3d<RangedCoordf64, RangedCoordf64, RangedCoordf64>
    > {
        ChartBuilder::on(area)
            .margin(PLOT_MARGIN)
            .caption(caption, ("sans-serif", font_size))
            .build_cartesian_3d(
                axes_ranges.x.clone(),
                axes_ranges.y.clone(),
                axes_ranges.z.clone(),
            ).unwrap()
    }

    fn reset_chart_context(&mut self) {
        self.chart = Self::create_chart_context(
            &self.area, 
            &self.caption,
            font_size(self.screen_resolution.0), 
            &self.axes_ranges
        );
    }

    fn draw_drone_networks(
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


pub struct Simulation<'a> {
    current_time_in_millis: u32,
    end_time_in_millis: u32,
    drone_networks: Vec<NetworkModel>,
    renderer: PlottersRenderer<'a>
}

impl<'a> Simulation<'a> {
    #[must_use]
    pub fn new(
        end_time_in_millis: u32,
        drone_networks: Vec<NetworkModel>,
        renderer: PlottersRenderer<'a>
    ) -> Self {
        Self {
            current_time_in_millis: START_TIME,
            end_time_in_millis,
            drone_networks,
            renderer
        }
    }

    /// # Panics
    ///
    /// Will panic if an error occurs during rendering. 
    pub fn run(&mut self) {
        let running = set_ctrlc_handler(); 

        let begin = self.current_time_in_millis;
        let end = self.end_time_in_millis;

        for _ in (begin..end).step_by(STEP_DURATION as usize) {
            if !running.load(std::sync::atomic::Ordering::SeqCst) { 
                info!(
                    "TERM, Simulation, Time: {}, {}", 
                    self.current_time_in_millis,
                    log_drone_count(&self.drone_networks)
                );
                return;
            }

            self.drone_networks
                .iter_mut()
                .for_each(NetworkModel::update);
            
            self.renderer
                .draw_drone_networks(&self.drone_networks)
                .unwrap();
                        
            trace!(
                "Current time: {}, {}", 
                self.current_time_in_millis,
                log_drone_count(&self.drone_networks)
            );
            self.current_time_in_millis += STEP_DURATION;
        }

        info!(
            "Simulation finished at {} millis",
            self.current_time_in_millis
        );
    }
}
