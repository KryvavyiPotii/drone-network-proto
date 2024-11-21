use std::collections::HashMap;

use rand::prelude::*;

use super::{DroneNetworkTypeConfig, Config};
use crate::communication::{message::*, signal::*};
use crate::device::{*, network::*};
use crate::device::network::cellularautomaton::CellularAutomaton;
use crate::device::network::complexnetwork::ComplexNetwork;
use crate::math_physics::Coordinates3D;
use crate::simulation::*;


const DRONE_NETWORK_POSITION: (f32, f32, f32)  = (150.3, 90.6, 25.5);
const DRONE_DESTINATION: (f32, f32, f32)       = (0.0, 0.0, 0.0);
const COMMAND_CENTER_POSITION: (f32, f32, f32) = (200.0, 100.0, 0.0);

const PLOT_RESOLUTION: (u32, u32) = (800, 600);


fn derive_filename(
    drone_network_type: &DroneNetworkTypeConfig,
    text: &str,
    topology: &Topology
) -> String {
    let mut filename = String::new();

    match drone_network_type {
        DroneNetworkTypeConfig::ComplexNetwork(_) => filename.push_str("cn_"),
        DroneNetworkTypeConfig::CellularAutomaton => filename.push_str("ca_"),
    }

    filename.push_str(text);

    match topology {
        Topology::Mesh => filename.push_str("_mesh.gif"),
        Topology::Star => filename.push_str("_star.gif"),
    }

    filename
}

fn init_drone_vec(drone_count: u32) -> Vec<Drone> {
    let mut rng = thread_rng();

    (1..=drone_count)
        .map(|_| {
            let random_x_offset = rng.gen_range(-40.0..40.0);
            let random_y_offset = rng.gen_range(-40.0..40.0);
            let random_z_offset = rng.gen_range(-20.0..20.0);
            
            let position = Coordinates3D::new(
                DRONE_NETWORK_POSITION.0 + random_x_offset,
                DRONE_NETWORK_POSITION.1 + random_y_offset,
                DRONE_NETWORK_POSITION.2 + random_z_offset
            );

            Drone::new(position)
        })
        .collect()
}


pub fn gps_only(config: Config) {
    let drone_network_type = config.drone_network_type;
    let topology = config.topology;

    let command_center = CommandCenter::new(
        Coordinates3D::from(COMMAND_CENTER_POSITION),
        SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
    );
    
    let drones = init_drone_vec(100); 
    

    let scenario: Vec<Message> = vec![
        Message::new(0, MessageType::SetDestination(
            Some(Coordinates3D::from(DRONE_DESTINATION)),
            Goal::Attack
        )),
    ];

    let drone_network = match drone_network_type {
        DroneNetworkTypeConfig::ComplexNetwork(delay_multiplier) => {
            let rwd_gps = complexnetwork::RadarWarfareDevice::new(
                Coordinates3D::new(0.0, 5.0, 2.0),
                HashMap::from([(SignalType::GPS, SignalLevel::Green)]),
                SignalAreaType::Dome(RWD_GPS_RADIUS_IN_METRES)
            );

            DroneNetwork::ComplexNetwork(
                complexnetwork::ComplexNetwork::new(
                    command_center,
                    &drones,
                    vec![rwd_gps],
                    &scenario,
                    topology,
                    delay_multiplier
            ))
        },

        DroneNetworkTypeConfig::CellularAutomaton => {
            let rwd_gps = cellularautomaton::RadarWarfareDevice::new(
                Coordinates3D::new(0.0, 5.0, 2.0),
                HashMap::from([(SignalType::GPS, SignalLevel::Green)]),
                SignalAreaType::Dome(RWD_GPS_RADIUS_IN_METRES)
            );

            DroneNetwork::CellularAutomaton(
                CellularAutomaton::new(
                    command_center,
                    &drones,
                    vec![rwd_gps],
                    &scenario,
                    topology
                )
            )
        }
    };

    let mut simulation = Simulation::build(
        END_TIME_IN_MILLIS,
        vec![drone_network],
        derive_filename(&drone_network_type, "gps_only", &topology),
        PLOT_RESOLUTION,
        0.0..200.0,
        0.0..200.0,
        0.0..200.0,
        vec![DroneColoring::SingleColor(0, 0, 0)] 
    ).unwrap_or_else(|err| {
        println!("Problem parsing arguments: {err}");
        std::process::exit(1);
    });

    simulation.run();
}

pub fn gps_and_control(config: Config) {
    let drone_network_type = config.drone_network_type;
    let topology = config.topology;

    let command_center = CommandCenter::new(
        Coordinates3D::from(COMMAND_CENTER_POSITION),
        SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
    );
    
    let drones = init_drone_vec(100); 

    let scenario: Vec<Message> = vec![
        Message::new(0, MessageType::SetDestination(
            Some(Coordinates3D::from(DRONE_DESTINATION)),
            Goal::Attack
        )),
    ];

    let drone_network = match drone_network_type {
        DroneNetworkTypeConfig::ComplexNetwork(delay_multiplier) => {
            let rwd_control = complexnetwork::RadarWarfareDevice::new(
                Coordinates3D::new(-10.0, 2.0, 0.0),
                HashMap::from([(SignalType::Control, SignalLevel::Green)]),
                SignalAreaType::Dome(RWD_CONTROL_RADIUS_IN_METRES)
            );

            let rwd_gps = complexnetwork::RadarWarfareDevice::new(
                Coordinates3D::new(0.0, 5.0, 2.0),
                HashMap::from([(SignalType::GPS, SignalLevel::Green)]),
                SignalAreaType::Dome(RWD_GPS_RADIUS_IN_METRES)
            );

            DroneNetwork::ComplexNetwork(
                complexnetwork::ComplexNetwork::new(
                    command_center,
                    &drones,
                    vec![rwd_control, rwd_gps],
                    &scenario,
                    topology,
                    delay_multiplier
            ))
        },

        DroneNetworkTypeConfig::CellularAutomaton => {
            let rwd_control = cellularautomaton::RadarWarfareDevice::new(
                Coordinates3D::new(-10.0, 2.0, 0.0),
                HashMap::from([(SignalType::Control, SignalLevel::Green)]),
                SignalAreaType::Dome(RWD_CONTROL_RADIUS_IN_METRES)
            );

            let rwd_gps = cellularautomaton::RadarWarfareDevice::new(
                Coordinates3D::new(0.0, 5.0, 2.0),
                HashMap::from([(SignalType::GPS, SignalLevel::Green)]),
                SignalAreaType::Dome(RWD_GPS_RADIUS_IN_METRES)
            );

            DroneNetwork::CellularAutomaton(
                CellularAutomaton::new(
                    command_center,
                    &drones,
                    vec![rwd_control, rwd_gps],
                    &scenario,
                    topology
                )
            )
        }
    };
 
    let mut simulation = Simulation::build(
        10000,
        vec![drone_network],
        derive_filename(&drone_network_type, "gps_and_control", &topology),
        PLOT_RESOLUTION,
        0.0..200.0,
        0.0..200.0,
        0.0..200.0,
        vec![DroneColoring::SingleColor(0, 0, 0)] 
    ).unwrap_or_else(|err| {
        println!("Problem parsing arguments: {err}");
        std::process::exit(1);
    });

    simulation.run();
}

pub fn command_delay(config: Config) {
    let display_delayless_network = config.display_delayless_network;
    let drone_network_type = DroneNetworkTypeConfig::ComplexNetwork(0.0);
    let topology = config.topology;

    let command_center = CommandCenter::new(
        Coordinates3D::from(COMMAND_CENTER_POSITION),
        SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
    );

    let drones = init_drone_vec(100); 

    let scenario: Vec<Message> = vec![
        Message::new(0, MessageType::SetDestination(
            Some(Coordinates3D::from(DRONE_DESTINATION)),
            Goal::Attack
        )),
        Message::new(250, MessageType::SetDestination(
            Some(Coordinates3D::from((0.0, 0.0, 150.0))),
            Goal::Reposition
        )),
        Message::new(4000, MessageType::SetDestination(
            Some(Coordinates3D::from((0.0, 150.0, 150.0))),
            Goal::Reposition
        )),
        Message::new(6000, MessageType::SetDestination(
            Some(Coordinates3D::from(DRONE_DESTINATION)),
            Goal::Attack
        )),
    ];

    let mut drone_networks = Vec::new();

    if display_delayless_network {
        let delayless_drone_network = DroneNetwork::ComplexNetwork(
            ComplexNetwork::new(
                command_center.clone(),
                &drones,
                Vec::new(),
                &scenario,
                topology,
                0.0
        ));

        drone_networks.push(delayless_drone_network);
    }

    let drone_network = DroneNetwork::ComplexNetwork(
        ComplexNetwork::new(
            command_center,
            &drones,
            Vec::new(),
            &scenario, 
            topology,
            1.0
    ));
    
    drone_networks.insert(0, drone_network);

    let mut simulation = Simulation::build(
        END_TIME_IN_MILLIS,
        drone_networks,
        derive_filename(&drone_network_type, "command_delay", &topology),
        PLOT_RESOLUTION,
        0.0..200.0,
        0.0..200.0,
        0.0..200.0,
        vec![
            DroneColoring::SingleColor(0, 0, 0),
            DroneColoring::SingleColor(255, 0, 0)
        ] 
    ).unwrap_or_else(|err| {
        println!("Problem parsing arguments: {err}");
        std::process::exit(1);
    });

    simulation.run();
}

pub fn signal_color(config: Config) {
    let drone_network_type = config.drone_network_type;
    let topology = config.topology;

    let command_center = CommandCenter::new(
        Coordinates3D::from(COMMAND_CENTER_POSITION),
        SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
    );

    let drones = init_drone_vec(100); 

    let scenario: Vec<Message> = vec![
        Message::new(0, MessageType::SetDestination(
            Some(Coordinates3D::from(DRONE_DESTINATION)),
            Goal::Attack
        )),
        Message::new(250, MessageType::SetDestination(
            Some(Coordinates3D::from((0.0, 0.0, 150.0))),
            Goal::Reposition
        )),
        Message::new(4000, MessageType::SetDestination(
            Some(Coordinates3D::from((0.0, 150.0, 150.0))),
            Goal::Reposition
        )),
        Message::new(6000, MessageType::SetDestination(
            Some(Coordinates3D::from(DRONE_DESTINATION)),
            Goal::Attack
        )),
    ];

    let drone_network = match drone_network_type {
        DroneNetworkTypeConfig::ComplexNetwork(delay_multiplier) => {
            DroneNetwork::ComplexNetwork(
                complexnetwork::ComplexNetwork::new(
                    command_center,
                    &drones,
                    Vec::new(),
                    &scenario,
                    topology,
                    delay_multiplier
            ))
        },

        DroneNetworkTypeConfig::CellularAutomaton => {
            DroneNetwork::CellularAutomaton(
                CellularAutomaton::new(
                    command_center,
                    &drones,
                    Vec::new(),
                    &scenario,
                    topology
                )
            )
        }
    };

    let mut simulation = Simulation::build(
        END_TIME_IN_MILLIS,
        vec![drone_network],
        derive_filename(&drone_network_type, "signal_color", &topology),
        PLOT_RESOLUTION,
        0.0..200.0,
        0.0..200.0,
        0.0..200.0,
        vec![DroneColoring::Signal]
    ).unwrap_or_else(|err| {
        println!("Problem parsing arguments: {err}");
        std::process::exit(1);
    });

    simulation.run();
}
