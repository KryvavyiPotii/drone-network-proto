use std::{
    ops::Range,
    sync::{Arc, atomic::{AtomicBool, Ordering}}
};

use log::{info, trace};
use modules::Receiver;
use plotters::{
    coord::{ranged3d::Cartesian3d, types::RangedCoordf64, Shift}, 
    prelude::*,
    style::RGBColor
};

use crate::communication::signal::*;
use crate::device::{*, networkmodel::*};
use crate::mathphysics::*;


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
        let entry = format!("Drone count {}: {},", i, network.drone_count());
        
        output.push_str(&entry); 
    }

    // Remove a trailing comma.
    output.truncate(output.len() - 1);

    output
}

fn font_size(screen_width: u32) -> u32 {
    screen_width / 15
}

fn get_destination_primitive( 
    destination: &Point3D,
    screen_height: u32,
) -> Circle<(f64, f64, f64), u32> {
    let point = plotters_point_from_point3d(&destination);
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
    coloring: &DroneColoring
) -> Circle<(f64, f64, f64), u32> {
    let point = plotters_point_from_point3d(drone.position());
    let color = get_drone_color(drone, coloring);

    Circle::new(point, 1, color)
}

fn get_rwd_primitive(
    rwd: &RWDType,
    signal_type: &SignalType,
    screen_height: u32
) -> Circle<(f64, f64, f64), u32> {
    let point = plotters_point_from_point3d(rwd.position());
    let rwd_coverage = convert_meters_to_pixels(
        rwd
            .area(signal_type)
            .radius(), 
        screen_height
    );
    let area_color = match signal_type {
        SignalType::GPS => RED,
        SignalType::Control => BLUE
    };

    Circle::new(point, rwd_coverage, area_color)
}

fn plotters_point_from_point3d(point: &Point3D) -> (f64, f64, f64) {
    (point.x as f64, point.z as f64, point.y as f64,)
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

fn get_drone_color(drone: &Drone, coloring: &DroneColoring) -> RGBColor {
    match coloring {
        DroneColoring::Signal => {
            let signal_level = drone.rx_signal_level(&SignalType::Control);

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
            RGBColor(*r, *g, *b)
        }
    }
}


#[derive(Clone, Copy)]
pub enum DroneColoring {
    SingleColor(u8, u8, u8),
    Signal,
}


pub struct Axes3DRanges {
    x: Range<f64>,
    y: Range<f64>,
    z: Range<f64>
}

impl Axes3DRanges {
    pub fn new(
        x: Range<f64>,
        y: Range<f64>,
        z: Range<f64>
    ) -> Self {
        Self { x, y, z }
    }
}


pub struct PlottersRenderer<'a> {
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
    pub fn new(
        output_path: &str,
        screen_resolution: (u32, u32),
        axes_ranges: Axes3DRanges,
        drone_colors: &[DroneColoring],
    ) -> Self {
        let output_path = output_path.to_string();
        let area = BitMapBackend::gif(
            &output_path, 
            screen_resolution,
            STEP_DURATION as u32
        ).unwrap().into_drawing_area();
        let chart = Self::create_chart_context(
            &area, 
            font_size(screen_resolution.0),
            &axes_ranges
        ); 

        Self {
            screen_resolution,
            axes_ranges,
            drone_colors: drone_colors.to_vec(),
            area,
            chart
        }
    }

    fn create_chart_context(
        area: &DrawingArea<BitMapBackend<'a>, Shift>, 
        font_size: u32,
        axes_ranges: &Axes3DRanges
    ) -> ChartContext<
        'a, 
        BitMapBackend<'a>, 
        Cartesian3d<RangedCoordf64, RangedCoordf64, RangedCoordf64>
    > {
        ChartBuilder::on(&area)
            .margin(PLOT_MARGIN)
            .caption("Drone network", ("sans-serif", font_size))
            .build_cartesian_3d(
                axes_ranges.x.clone(),
                axes_ranges.y.clone(),
                axes_ranges.z.clone(),
            ).unwrap()
    }

    fn reset_chart_context(&mut self) {
        self.chart = Self::create_chart_context(
            &self.area, 
            font_size(self.screen_resolution.0), 
            &self.axes_ranges
        );
    }

    fn draw_drone_networks(
        &mut self, 
        drone_networks: &[NetworkModel]
    // TODO improve error handling (with thiserror crate)
    ) -> Result<(), &'static str> {
        if self.drone_colors.len() != drone_networks.len() {
            return Err("Colors count does not match network count");
        }
        
        self.reset_chart_context();

        let destinations = get_drone_networks_destinations(drone_networks);
        let rwds = drone_networks
            .iter()
            .fold(Vec::new(), |mut acc, network| { 
                acc.extend(network.rwds());
                acc
            });

        self.chart.configure_axes().draw().unwrap();
        
        self.draw_destinations(&destinations);
        self.draw_command_centers(drone_networks);
        self.draw_drones(drone_networks);
        self.draw_rwds(&rwds);

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
                    .map(|drone| get_drone_primitive(drone, coloring))
            ).unwrap();
        }
    }

    fn draw_rwds(&mut self, rwds: &[RWDType]) {
        for rwd in rwds {
            self.draw_rwd(rwd, &SignalType::Control);
            self.draw_rwd(rwd, &SignalType::GPS);
        }
    }

    fn draw_rwd(&mut self, rwd: &RWDType, signal_type: &SignalType) {
        self.chart.draw_series([
            get_rwd_primitive(
                rwd, 
                signal_type, 
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
                .for_each(|drone_network| drone_network.update());
            
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
