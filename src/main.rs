use std::io::Write;

use clap::{Command, Arg};
use env_logger::{Builder, Target};

use device::networkmodel::{Topology, NetworkModelType};


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
        .version("0.3.1")
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
                    "Complex network (Star, GPS EWD)",
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Star
                )
            ),
            2 => examples::gps_and_control(
                &Config::new(
                    "Complex network (Star, GPS and Control EWD)",
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Star
                )
            ),
            3 => examples::command_delay(
                &Config::new(
                    "Complex network (Star, delays)",
                    true,
                    NetworkModelType::ComplexNetwork(1.0),
                    Topology::Star
                )
            ),
            4 => examples::signal_color(
                &Config::new(
                    "Complex network (Star, signal color)",
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Star
                )
            ),
            5 => examples::gps_only(
                &Config::new(
                    "Complex network (Mesh, GPS EWD)",
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Mesh
                )
            ),
            6 => examples::gps_and_control(
                &Config::new(
                    "Complex network (Mesh, GPS and Control EWD)",
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Mesh
                )
            ),
            7 => examples::command_delay(
                &Config::new(
                    "Complex network (Mesh, delays)",
                    true,
                    NetworkModelType::ComplexNetwork(1.0),
                    Topology::Mesh
                )
            ),
            8 => examples::signal_color(
                &Config::new(
                    "Complex network (Mesh, signal color)",
                    false,
                    NetworkModelType::ComplexNetwork(0.0),
                    Topology::Mesh
                )
            ),
            9 => examples::gps_only(
                &Config::new(
                    "Cellular automaton (Star, GPS EWD)",
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Star
                )
            ),
            10 => examples::gps_and_control(
                &Config::new(
                    "Cellular automaton (Star, GPS and Control EWD)",
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Star
                )
            ),
            11 => examples::signal_color(
                &Config::new(
                    "Cellular automaton (Star, signal color)",
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Star
                )
            ),
            12 => examples::gps_only(
                &Config::new(
                    "Cellular automaton (Mesh, GPS EWD)",
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Mesh
                )
            ),
            13 => examples::gps_and_control(
                &Config::new(
                    "Cellular automaton (Mesh, GPS and Control EWD)",
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Mesh
                )
            ),
            14 => examples::signal_color(
                &Config::new(
                    "Cellular automaton (Mesh, signal color)",
                    false,
                    NetworkModelType::CellularAutomaton,
                    Topology::Mesh
                )
            ),
            _ => ()
        }
    }
}


pub enum AntennaType {
    Color,
    Strength
}


pub struct Config {
    plot_caption: String,
    display_delayless_network: bool, 
    network_model: NetworkModelType,
    topology: Topology,
}

impl Config {
    #[must_use]
    pub fn new(
        plot_caption: &str,
        display_delayless_network: bool,
        network_model: NetworkModelType,
        topology: Topology
    ) -> Self {
        Self {
            plot_caption: plot_caption.to_string(),
            display_delayless_network,
            network_model,
            topology
        }
    }

    #[must_use]
    pub fn antenna(&self) -> AntennaType {
        match self.network_model {
            NetworkModelType::CellularAutomaton => AntennaType::Color,
            NetworkModelType::ComplexNetwork(_) => AntennaType::Strength,
        }
    }

    #[must_use]
    pub fn delay_multiplier(&self) -> f32 {
        match self.network_model {
            NetworkModelType::ComplexNetwork(delay_multiplier) =>
                delay_multiplier,
            NetworkModelType::CellularAutomaton => 0.0
        }
    }
}


fn main() {
    configure_logging();
    cli();
}
