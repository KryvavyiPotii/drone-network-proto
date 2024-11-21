use clap::{Command, Arg, ArgAction};


pub mod communication;
pub mod device;
pub mod examples;
pub mod math_physics;
pub mod simulation;


pub enum DroneNetworkTypeConfig {
    CellularAutomaton,
    ComplexNetwork(f32) // delay
}

pub struct Config {
    display_delayless_network: bool, 
    drone_network_type: DroneNetworkTypeConfig,
    topology: device::network::Topology,
}

impl Config {
    pub fn new(
        display_delayless_network: bool,
        drone_network_type: DroneNetworkTypeConfig,
        topology: device::network::Topology
    ) -> Self {
        Self { 
            display_delayless_network,
            drone_network_type,
            topology
        }
    }
}


fn main() {
    let matches = Command::new("drone_network")
        .version("0.1.0")
        .about("Models drone networks.")
        .arg(
            Arg::new("example")
                .short('e')
                .long("example")
                .value_parser(clap::value_parser!(u8))
        )
        .arg(
            Arg::new("verbose")
                .short('v')
                .long("verbose")
                .action(ArgAction::SetTrue)
        )
        .get_matches();

    if let Some(num) = matches.get_one::<u8>("example") {
        match num {
            1 => examples::gps_only(Config::new(
                false,
                DroneNetworkTypeConfig::ComplexNetwork(0.0),
                device::network::Topology::Star
            )),
            2 => examples::gps_and_control(Config::new(
                false,
                DroneNetworkTypeConfig::ComplexNetwork(0.0),
                device::network::Topology::Star
            )),
            3 => examples::command_delay(Config::new(
                true,
                DroneNetworkTypeConfig::ComplexNetwork(1.0),
                device::network::Topology::Star
            )),
            4 => examples::signal_color(Config::new(
                false,
                DroneNetworkTypeConfig::ComplexNetwork(0.0),
                device::network::Topology::Star
            )),
            5 => examples::gps_only(Config::new(
                false,
                DroneNetworkTypeConfig::ComplexNetwork(0.0),
                device::network::Topology::Mesh
            )),
            6 => examples::gps_and_control(Config::new(
                false,
                DroneNetworkTypeConfig::ComplexNetwork(0.0),
                device::network::Topology::Mesh
            )),
            7 => examples::command_delay(Config::new(
                true,
                DroneNetworkTypeConfig::ComplexNetwork(1.0),
                device::network::Topology::Mesh
            )),
            8 => examples::signal_color(Config::new(
                false,
                DroneNetworkTypeConfig::ComplexNetwork(0.0),
                device::network::Topology::Mesh
            )),
            9 => examples::gps_only(Config::new(
                false,
                DroneNetworkTypeConfig::CellularAutomaton,
                device::network::Topology::Star
            )),
            10 => examples::gps_and_control(Config::new(
                false,
                DroneNetworkTypeConfig::CellularAutomaton,
                device::network::Topology::Star
            )),
            11 => examples::signal_color(Config::new(
                false,
                DroneNetworkTypeConfig::CellularAutomaton,
                device::network::Topology::Star
            )),
            12 => examples::gps_only(Config::new(
                false,
                DroneNetworkTypeConfig::CellularAutomaton,
                device::network::Topology::Mesh
            )),
            13 => examples::gps_and_control(Config::new(
                false,
                DroneNetworkTypeConfig::CellularAutomaton,
                device::network::Topology::Mesh
            )),
            14 => examples::signal_color(Config::new(
                false,
                DroneNetworkTypeConfig::CellularAutomaton,
                device::network::Topology::Mesh
            )),
            _ => ()
        }
    }
}
