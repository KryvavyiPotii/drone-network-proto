use std::io::Write;

use chrono;
use clap::{Command, Arg};
use env_logger::{Builder, Target};

use device::{modules::AntennaType, networkmodel::Topology};


pub mod communication;
pub mod device;
pub mod examples;
pub mod mathphysics;
pub mod simulation;


fn configure_logging() {
    Builder::new()
        .format(|buf, record| 
            writeln!(
                buf,
                "{} {} - {}", 
                chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
                record.level(), 
                record.args()
            )
        )
        .filter(None, log::LevelFilter::Trace)
        .target(Target::Stdout)
        .init();
}

fn cli() {
    let matches = Command::new("drone_network")
        .version("0.1.0")
        .about("Models drone networks")
        .arg(
            Arg::new("example")
                .short('e')
                .long("example")
                .value_parser(clap::value_parser!(u8))
        )
        .get_matches();

    if let Some(num) = matches.get_one::<u8>("example") {
        match num {
            1 => examples::gps_only(
                &Config::new(
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Star
                )
            ),
            2 => examples::gps_and_control(
                &Config::new(
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Star
                )
            ),
            3 => examples::command_delay(
                &Config::new(
                    true,
                    NetworkModelType::ComplexNetwork(1.0),
                    Topology::Star
                )
            ),
            4 => examples::signal_color(
                &Config::new(
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Star
                )
            ),
            5 => examples::gps_only(
                &Config::new(
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Mesh
                )
            ),
            6 => examples::gps_and_control(
                &Config::new(
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Mesh
                )
            ),
            7 => examples::command_delay(
                &Config::new(
                    true,
                    NetworkModelType::ComplexNetwork(1.0),
                    Topology::Mesh
                )
            ),
            8 => examples::signal_color(
                &Config::new(
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Mesh
                )
            ),
            9 => examples::gps_only(
                &Config::new(
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Star
                )
            ),
            10 => examples::gps_and_control(
                &Config::new(
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Star
                )
            ),
            11 => examples::signal_color(
                &Config::new(
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Star
                )
            ),
            12 => examples::gps_only(
                &Config::new(
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Mesh
                )
            ),
            13 => examples::gps_and_control(
                &Config::new(
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Mesh
                )
            ),
            14 => examples::signal_color(
                &Config::new(
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Mesh
                )
            ),
            _ => ()
        }
    }
}


pub enum NetworkModelType {
    CellularAutomaton,
    ComplexNetwork(f32) // delay
}

pub struct Config {
    display_delayless_network: bool, 
    network_model: NetworkModelType,
    topology: Topology,
}

impl Config {
    pub fn new(
        display_delayless_network: bool,
        network_model: NetworkModelType,
        topology: Topology
    ) -> Self {
        Self { 
            display_delayless_network,
            network_model,
            topology
        }
    }

    pub fn antenna(&self) -> AntennaType {
        match self.network_model {
            NetworkModelType::CellularAutomaton => AntennaType::Color,
            NetworkModelType::ComplexNetwork(_) => AntennaType::Strength,
        }
    }

    pub fn delay_multiplier(&self) -> f32 {
        match self.network_model {
            NetworkModelType::ComplexNetwork(delay_multiplier) =>
                delay_multiplier,
            _ => 0.0
        }
    }
}


fn main() {
    configure_logging();
    cli();
}
