use std::sync::{Arc, atomic::AtomicBool};

use plotters::prelude::*;
use plotters::style::RGBColor;

use crate::communication::signal::*;
use crate::device::{*, network::*};
use crate::math_physics::*;


pub const START_TIME_IN_MILLIS: u64 = 0;
pub const END_TIME_IN_MILLIS: u64   = 15000;

const COMMAND_CENTER_RADIUS_IN_METRES: f32 = 5.0;


fn font_size(screen_width: u32) -> u32 {
    screen_width / 15
}

fn convert_metres_to_pixels(
    value_in_metres: f32,
    screen_height: u32
    ) -> u32 {
    // This value is very rough because it does not consider the perspective.
    // TODO change scale calculation method or visualization method.
    (value_in_metres * screen_height as f32 / 400.0) as u32 
}


pub enum DroneColoring {
    SingleColor(u8, u8, u8),
    Signal,
}


pub struct Simulation {
    pub current_time_in_millis: u64,
    pub end_time_in_millis: u64,
    pub drone_networks: Vec<DroneNetwork>,
    pub output_path: String,
    pub screen_resolution: (u32, u32),
    pub x_axis_range: std::ops::Range<f64>,
    pub y_axis_range: std::ops::Range<f64>,
    pub z_axis_range: std::ops::Range<f64>,
    pub drone_colors: Vec<DroneColoring>,
}

impl Simulation {
    pub fn build(
        end_time_in_millis: u64,
        drone_networks: Vec<DroneNetwork>,
        output_path: String,
        screen_resolution: (u32, u32),
        x_axis_range: std::ops::Range<f64>,
        y_axis_range: std::ops::Range<f64>,
        z_axis_range: std::ops::Range<f64>,
        drone_colors: Vec<DroneColoring>,
    ) -> Result<Self, &'static str> {
        let drone_network_count = &drone_networks.len();
        
        if drone_colors.len() != *drone_network_count {
            return Err("Colors count does not match network count");
        }

        Ok(Self {
            current_time_in_millis: START_TIME_IN_MILLIS,
            end_time_in_millis,
            drone_networks,
            output_path,
            screen_resolution,
            x_axis_range,
            y_axis_range,
            z_axis_range,
            drone_colors,
        })
    }

    pub fn run(&mut self) {
        // Handle SIGINT to stop rendering.
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        ctrlc_async::set_handler(move || {
            r.store(false, std::sync::atomic::Ordering::SeqCst);
        }).expect("Error setting Ctrl-C handler");

        let area = BitMapBackend::gif(
            &self.output_path, 
            self.screen_resolution,
            STEP_DURATION_IN_MILLIS as u32
        ).unwrap().into_drawing_area();

        let begin = self.current_time_in_millis;
        let end = self.end_time_in_millis;
        let font_size = font_size(self.screen_resolution.1);

        let mut rwds: Vec<RadarWarfareDeviceType> = Vec::new();

        for drone_network in &self.drone_networks {
            rwds.extend(drone_network.radar_warfare_devices());
        }

        for _ in (begin..end).step_by(STEP_DURATION_IN_MILLIS as usize) {
            if !running.load(std::sync::atomic::Ordering::SeqCst) { 
                break;
            }

            self.drone_networks
                .iter_mut()
                .for_each(|drone_network| drone_network.update_network());
            
            let mut chart = ChartBuilder::on(&area)
                .margin(20)
                .caption("Drone network", ("sans-serif", font_size))
                .build_cartesian_3d(
                    self.x_axis_range.clone(),
                    self.y_axis_range.clone(),
                    self.z_axis_range.clone()
                )
                .unwrap();
            chart.configure_axes().draw().unwrap();
            
            // Draw destination points.
            let destinations: Vec<&Coordinates3D> = self.drone_networks
                .iter()
                .map(|drone_network| drone_network.destination())
                .collect();

            chart.draw_series(
                destinations
                    .iter()
                    .map(|position_in_metres| {
                        let point = (
                            position_in_metres.x as f64,
                            position_in_metres.z as f64,
                            position_in_metres.y as f64,
                        );

                        let radius = convert_metres_to_pixels(
                                DESTINATION_RADIUS_IN_METRES,
                                self.screen_resolution.1
                            );

                        Circle::new(point, radius, YELLOW)
                    }),
            ).unwrap();
            
            // Draw command centers.
            chart.draw_series(
                self.drone_networks
                    .iter()
                    .map(|drone_network| {
                        let command_center = drone_network.command_center();
                        
                        let position = command_center.position(); 

                        let point = (
                            position.x as f64,
                            position.z as f64,
                            position.y as f64,
                        );

                        let radius = convert_metres_to_pixels(
                            COMMAND_CENTER_RADIUS_IN_METRES,
                            self.screen_resolution.1
                        );  
                        
                        Circle::new(point, radius, GREEN)
                    }),
            ).unwrap();
           
            // Draw drones.
            for (drone_network, coloring) in self.drone_networks
                .iter()
                .zip(self.drone_colors.iter())
            {
                chart.draw_series(
                    drone_network
                        .drone_iter()
                        .map(|drone| {
                            let position = drone.position();

                            let point = (
                                position.x as f64,
                                position.z as f64,
                                position.y as f64,
                            );

                            let color = match coloring {
                                DroneColoring::Signal => {
                                    match drone.rx_signal_level(
                                        &SignalType::Control
                                    ).unwrap() {
                                        SignalLevel::Black => BLACK,
                                        SignalLevel::Yellow => YELLOW,
                                        SignalLevel::Red => RED,
                                        SignalLevel::Green => GREEN,
                                    }
                                },

                                DroneColoring::SingleColor(r, g, b) => {
                                    RGBColor(*r, *g, *b)
                                }
                            };

                            Circle::new(point, 1, color)
                        })
                ).unwrap();
            }

            // Draw radar warfare devices.
            for rwd in &rwds {
                chart.draw_series(
                    rwd.tx_signal_levels() 
                        .keys()
                        .map(|signal_type| {
                            let position = rwd.position();

                            let point = (
                                position.x as f64,
                                position.z as f64,
                                position.y as f64,
                            );

                            let rwd_coverage = match rwd.area() {
                                SignalAreaType::Dome(radius_in_metres) =>
                                    convert_metres_to_pixels(
                                        *radius_in_metres,
                                        self.screen_resolution.1
                                    ),
                            };
                            
                            let area_color = match signal_type {
                                SignalType::GPS => RED,
                                SignalType::Control => BLUE
                            };

                            Circle::new(point, rwd_coverage, area_color)
                        }),
                ).unwrap();
            }

            area.present().unwrap();
            area.fill(&WHITE).unwrap();

            println!("Current time: {}", self.current_time_in_millis);
            self.current_time_in_millis += STEP_DURATION_IN_MILLIS;
        }

        println!(
            "[INFO] Simulation finished at {} millis",
            self.current_time_in_millis
        );
    }
}
