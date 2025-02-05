use std::collections::HashMap;

use rand::prelude::*;

use crate::communication::{message::*, signal::*};
use crate::device::{
    *, 
    networkmodel::*,
    modules::{AntennaType, TRXModule}
};
use crate::mathphysics::Point3D;
use crate::simulation::*;

use super::{NetworkModelType, Config};


const DRONE_NETWORK_POSITION: Point3D  = Point3D { x: 150.3, y: 90.6, z: 25.5 };
const DRONE_DESTINATION: Point3D       = Point3D { x: 0.0, y: 0.0, z: 0.0 };
const COMMAND_CENTER_POSITION: Point3D = Point3D { x: 200.0, y: 100.0, z: 0.0 };

const PLOT_RESOLUTION: (u32, u32) = (800, 600);


fn cc_tx_module(antenna: AntennaType) -> TRXModule {
    TRXModule::build(
        antenna,
        HashMap::from([(SignalType::Control, GREEN_SIGNAL_LEVEL)]),
        HashMap::from([(
            SignalType::Control,
            SignalLevel::from(CC_TX_CONTROL_AREA)
        )])
    ).unwrap()
}

fn drone_tx_module(antenna: AntennaType) -> TRXModule {
    TRXModule::build(
        antenna,
        HashMap::from([
            (SignalType::Control, SignalLevel::from(DRONE_TX_CONTROL_AREA))
        ]),
        HashMap::new()
    ).unwrap()
}

fn drone_rx_module(antenna: AntennaType) -> TRXModule {
    TRXModule::build(
        antenna,
        HashMap::from([
            (SignalType::Control, GREEN_SIGNAL_LEVEL),
            (SignalType::GPS, GREEN_SIGNAL_LEVEL)
        ]),
        HashMap::new()
    ).unwrap()
}

fn gps_rwd_tx_module(antenna: AntennaType) -> TRXModule {
    let tx_signal_levels = HashMap::from([(
        SignalType::GPS, 
        SignalLevel::from(RWD_TX_GPS_AREA)
    )]);

    TRXModule::build(
        antenna,
        tx_signal_levels.clone(),
        tx_signal_levels
    ).unwrap()
}

fn control_rwd_tx_module(antenna: AntennaType) -> TRXModule {
    let tx_signal_levels = HashMap::from([(
        SignalType::Control, 
        SignalLevel::from(RWD_TX_CONTROL_AREA)
    )]);

    TRXModule::build(
        antenna,
        tx_signal_levels.clone(),
        tx_signal_levels
    ).unwrap()
}

fn default_movement_scenario() -> Scenario {
    Scenario::from([
        (
            SignalType::Control,
            Message::new(
                0, 
                MessageType::SetDestination(
                    Point3D::from(DRONE_DESTINATION),
                    Goal::Attack
                )
            )
        ),
        (
            SignalType::Control,
            Message::new(
                250, 
                MessageType::SetDestination(
                    Point3D::new(0.0, 0.0, 150.0),
                    Goal::Reposition
                )
            )
        ),
        (
            SignalType::Control,
            Message::new(
                4000, 
                MessageType::SetDestination(
                    Point3D::new(0.0, 150.0, 150.0),
                    Goal::Reposition
                )
            )
        ),
        (
            SignalType::Control,
            Message::new(
                6000, 
                MessageType::SetDestination(
                    DRONE_DESTINATION,
                    Goal::Attack
                )
            )
        ),
    ])
}

fn derive_filename(config: &Config, text: &str) -> String {
    let mut filename = String::new();

    match config.network_model {
        NetworkModelType::ComplexNetwork(_) => filename.push_str("cn_"),
        NetworkModelType::CellularAutomaton => filename.push_str("ca_"),
    }

    filename.push_str(text);

    match config.topology {
        Topology::Mesh => filename.push_str("_mesh.gif"),
        Topology::Star => filename.push_str("_star.gif"),
    }

    filename
}

fn init_drone_vec(drone_count: u32, antenna: AntennaType) -> Vec<Drone> {
    let mut rng = thread_rng();

    let tx_module = drone_tx_module(antenna);
    let rx_module = drone_rx_module(antenna);

    (1..=drone_count)
        .map(|_| {
            let random_offset = Point3D::new(
                rng.gen_range(-40.0..40.0),
                rng.gen_range(-40.0..40.0),
                rng.gen_range(-20.0..20.0)
            );
            
            let position = DRONE_NETWORK_POSITION + random_offset;

            DroneBuilder::new()
                .set_global_position(position)
                .set_tx_module(tx_module.clone())
                .set_rx_module(rx_module.clone())
                .build()
        })
        .collect()
}

fn create_renderer<'a>(
    output_path: &'a str, 
    drone_colorings: &[DroneColoring]
) -> PlottersRenderer<'a> {
    PlottersRenderer::new(
        output_path,
        PLOT_RESOLUTION,
        Axes3DRanges::new(
            0.0..200.0,
            0.0..200.0,
            0.0..200.0,
        ),
        drone_colorings
    )
}


pub fn gps_only(config: &Config) {
    let antenna = config.antenna();

    let command_center = CommandCenterBuilder::new()
        .set_position(COMMAND_CENTER_POSITION)
        .set_tx_module(cc_tx_module(antenna))
        .build();

    let drones = init_drone_vec(100, antenna); 
    
    let scenario = Scenario::from([(
        SignalType::Control,
        Message::new(
            0, 
            MessageType::SetDestination(
                Point3D::from(DRONE_DESTINATION),
                Goal::Attack
            )
        ),
    )]);

    let drone_network = match config.network_model {
        NetworkModelType::CellularAutomaton => {
            let rwd_gps = CellularAutomatonRWDBuilder::new()
                .set_position(Point3D::new(0.0, 5.0, 2.0))
                .set_tx_module(gps_rwd_tx_module(AntennaType::Color))
                .build();

            NetworkModel::CellularAutomaton(
                CellularAutomatonBuilder::new()
                    .set_command_center(command_center)
                    .set_drones(&drones)
                    .set_radar_warfare_devices(&[rwd_gps])
                    .set_topology(config.topology)
                    .set_scenario(scenario)
                    .build()
            )
        },
        NetworkModelType::ComplexNetwork(delay_multiplier) => {
            let rwd_gps = ComplexNetworkRWDBuilder::new()
                .set_position(Point3D::new(0.0, 5.0, 2.0))
                .set_tx_module(gps_rwd_tx_module(AntennaType::Strength))
                .build();

            NetworkModel::ComplexNetwork(
                ComplexNetworkBuilder::new()
                    .set_command_center(command_center)
                    .set_drones(&drones)
                    .set_radar_warfare_devices(&vec![rwd_gps])
                    .set_topology(config.topology)
                    .set_scenario(scenario)
                    .set_delay_multiplier(delay_multiplier)
                    .build()
            )
        }
    };

    let output_path = derive_filename(config, "gps_only");
    let drone_colorings = vec![DroneColoring::SingleColor(0, 0, 0)]; 
    let renderer = create_renderer(&output_path, &drone_colorings);

    let mut simulation = Simulation::new(
        END_TIME,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn gps_and_control(config: &Config) {
    let antenna = config.antenna();

    let command_center = CommandCenterBuilder::new()
        .set_position(COMMAND_CENTER_POSITION)
        .set_tx_module(cc_tx_module(antenna))
        .build();
    
    let drones = init_drone_vec(100, antenna); 

    let scenario = Scenario::from([(
        SignalType::Control,
        Message::new(
            0, 
            MessageType::SetDestination(
                Point3D::from(DRONE_DESTINATION),
                Goal::Attack
            )
        )
    )]);

    let drone_network = match config.network_model {
        NetworkModelType::CellularAutomaton => {
            let rwd_control = CellularAutomatonRWDBuilder::new()
                .set_position(Point3D::new(-10.0, 2.0, 0.0))
                .set_tx_module(control_rwd_tx_module(AntennaType::Color))
                .build();
            let rwd_gps = CellularAutomatonRWDBuilder::new()
                .set_position(Point3D::new(0.0, 5.0, 2.0))
                .set_tx_module(gps_rwd_tx_module(AntennaType::Color))
                .build();

            NetworkModel::CellularAutomaton(
                CellularAutomatonBuilder::new()
                    .set_command_center(command_center)
                    .set_drones(&drones)
                    .set_radar_warfare_devices(&vec![rwd_control, rwd_gps])
                    .set_topology(config.topology)
                    .set_scenario(scenario)
                    .build()
            )
        },
        NetworkModelType::ComplexNetwork(delay_multiplier) => {
            let rwd_control = ComplexNetworkRWDBuilder::new()
                .set_position(Point3D::new(-10.0, 2.0, 0.0))
                .set_tx_module(control_rwd_tx_module(AntennaType::Strength))
                .build();
            let rwd_gps = ComplexNetworkRWDBuilder::new()
                .set_position(Point3D::new(0.0, 5.0, 2.0))
                .set_tx_module(gps_rwd_tx_module(AntennaType::Strength))
                .build();

            NetworkModel::ComplexNetwork(
                ComplexNetworkBuilder::new()
                    .set_command_center(command_center)
                    .set_drones(&drones)
                    .set_radar_warfare_devices(&vec![rwd_control, rwd_gps])
                    .set_topology(config.topology)
                    .set_scenario(scenario)
                    .set_delay_multiplier(delay_multiplier)
                    .build()
            )
        }
    };
 
    let output_path = derive_filename(config, "gps_and_control");
    let drone_colorings = vec![DroneColoring::SingleColor(0, 0, 0)]; 
    let renderer = create_renderer(&output_path, &drone_colorings);
    
    let mut simulation = Simulation::new(
        10000,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn command_delay(config: &Config) {
    let delay_multiplier = config.delay_multiplier();
    let antenna = config.antenna();

    let command_center = CommandCenterBuilder::new()
        .set_position(COMMAND_CENTER_POSITION)
        .set_tx_module(cc_tx_module(antenna))
        .build();

    let drones = init_drone_vec(100, antenna); 

    let scenario = default_movement_scenario(); 

    let mut drone_networks = Vec::new();

    if config.display_delayless_network {
        let delayless_drone_network = NetworkModel::ComplexNetwork(
            ComplexNetworkBuilder::new()
                .set_command_center(command_center.clone())
                .set_drones(&drones)
                .set_topology(config.topology)
                .set_scenario(scenario.clone())
                .build()
        );

        drone_networks.push(delayless_drone_network);
    }

    let drone_network = NetworkModel::ComplexNetwork(
        ComplexNetworkBuilder::new()
            .set_command_center(command_center)
            .set_drones(&drones)
            .set_topology(config.topology)
            .set_scenario(scenario)
            .set_delay_multiplier(delay_multiplier)
            .build()
    );
    
    drone_networks.insert(0, drone_network);

    let output_path = derive_filename(config, "command_delay");
    let drone_colorings = vec![
        DroneColoring::SingleColor(0, 0, 0),
        DroneColoring::SingleColor(255, 0, 0)
    ];
    let renderer = create_renderer(&output_path, &drone_colorings);

    let mut simulation = Simulation::new(
        END_TIME,
        drone_networks,
        renderer
    );

    simulation.run();
}

pub fn signal_color(config: &Config) {
    let antenna = config.antenna();

    let command_center = CommandCenterBuilder::new()
        .set_position(COMMAND_CENTER_POSITION)
        .set_tx_module(cc_tx_module(antenna))
        .build();

    let drones = init_drone_vec(100, antenna); 

    let scenario = default_movement_scenario(); 

    let drone_network = match config.network_model {
        NetworkModelType::CellularAutomaton => {
            NetworkModel::CellularAutomaton(
                CellularAutomatonBuilder::new()
                    .set_command_center(command_center)
                    .set_drones(&drones)
                    .set_topology(config.topology)
                    .set_scenario(scenario)
                    .build()
            )
        },
        NetworkModelType::ComplexNetwork(delay_multiplier) => {
            NetworkModel::ComplexNetwork(
                ComplexNetworkBuilder::new()
                    .set_command_center(command_center)
                    .set_drones(&drones)
                    .set_topology(config.topology)
                    .set_scenario(scenario)
                    .set_delay_multiplier(delay_multiplier)
                    .build()
            )
        }
    };

    let output_path = derive_filename(config, "signal_color");
    let drone_colorings = vec![DroneColoring::Signal];
    let renderer = create_renderer(&output_path, &drone_colorings);
    
    let mut simulation = Simulation::new(
        END_TIME,
        vec![drone_network],
        renderer
    );

    simulation.run();
}
