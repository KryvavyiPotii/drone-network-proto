use std::collections::HashMap;
use std::ops::Range;

use rand::prelude::*;

use crate::device::{
    CommandCenterBuilder, Device, DeviceId, Drone, DroneBuilder, 
    ElectronicWarfareBuilder, Topology, UNKNOWN_ID
};
use crate::device::networkmodel::{NetworkModelBuilder, NetworkModelType}; 
use crate::device::systems::{TRXModule, TRXSystem};
use crate::infection::InfectionType;
use crate::mathphysics::{Megahertz, Meter, Point3D};
use crate::message::{Goal, Message, MessageType};
use crate::signal::{
    SignalArea, SignalLevel, GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, 
    WIFI_2_4GHZ_FREQUENCY
};
use crate::simulation::{END_TIME, Simulation};
use crate::simulation::renderer::{
    Axes3DRanges, DroneColoring, PlottersRenderer
};

use super::{AntennaType, Config};


const BIG_SIGNAL_STRENGTH_VALUE: f32 = 100_000.0;

const DRONE_NETWORK_POSITION: Point3D  = Point3D { x: 150.3, y: 90.6, z: 25.5 };
const DRONE_DESTINATION: Point3D       = Point3D { x: 0.0, y: 0.0, z: 0.0 };
const COMMAND_CENTER_POSITION: Point3D = Point3D { x: 200.0, y: 100.0, z: 0.0 };

const PLOT_RESOLUTION: (u32, u32) = (400, 300);

const VULNERABILITY_PROBABILITY: f64 = 1.0;


fn cc_trx_system(
    antenna: &AntennaType, 
    tx_control_area_radius: Meter
) -> TRXSystem {
    let tx_control_area = SignalArea::build(tx_control_area_radius)
        .unwrap();

    let max_tx_signal_levels = HashMap::from([(
        WIFI_2_4GHZ_FREQUENCY, 
        SignalLevel::from(BIG_SIGNAL_STRENGTH_VALUE)
    )]);
    let tx_signal_levels = HashMap::from([(
        WIFI_2_4GHZ_FREQUENCY,
        SignalLevel::from_area(tx_control_area, WIFI_2_4GHZ_FREQUENCY)
    )]);

    let tx_module = TRXModule::build(
        max_tx_signal_levels,
        tx_signal_levels
    ).unwrap();

    match antenna {
        AntennaType::Color => TRXSystem::Color(tx_module),
        AntennaType::Strength => TRXSystem::Strength { 
            tx_module, 
            rx_module: TRXModule::default()
        }
    }
}

fn drone_trx_system(
    antenna: &AntennaType, 
    tx_control_area_radius: Meter
) -> TRXSystem {
    let tx_control_area = SignalArea::build(tx_control_area_radius)
        .unwrap();
    let max_tx_control_signal_level = SignalLevel::from_area(
        tx_control_area, 
        WIFI_2_4GHZ_FREQUENCY
    );

    match antenna {
        AntennaType::Color => {
            let max_signal_levels = HashMap::from([
                (WIFI_2_4GHZ_FREQUENCY, max_tx_control_signal_level),
                (GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL)
            ]);

            let trx_module = TRXModule::build(
                max_signal_levels,
                HashMap::new()
            ).unwrap();

            TRXSystem::Color(trx_module)
        },
        AntennaType::Strength => {
            let max_tx_signal_levels = HashMap::from([
                (WIFI_2_4GHZ_FREQUENCY, max_tx_control_signal_level),
            ]);
            let max_rx_signal_levels = HashMap::from([
                (WIFI_2_4GHZ_FREQUENCY, GREEN_SIGNAL_LEVEL),
                (GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL)
            ]);

            let tx_module = TRXModule::build(
                max_tx_signal_levels,
                HashMap::new()
            ).unwrap();
            let rx_module = TRXModule::build(
                max_rx_signal_levels,
                HashMap::new()
            ).unwrap();

            TRXSystem::Strength { tx_module, rx_module }
        }
    }
}

fn ewd_trx_system(
    antenna: &AntennaType,
    frequency: Megahertz,
    suppression_area_radius: Meter
) -> TRXSystem {
    let suppression_area = SignalArea::build(suppression_area_radius)
        .unwrap();
    let suppression_signal_level = SignalLevel::from_area(
        suppression_area, 
        frequency
    );

    let max_tx_signal_levels = HashMap::from([
        (frequency, suppression_signal_level)
    ]);
    let tx_signal_levels = max_tx_signal_levels.clone();

    let tx_module = TRXModule::build(
        max_tx_signal_levels,
        tx_signal_levels
    ).unwrap();

    match antenna {
        AntennaType::Color => TRXSystem::Color(tx_module),
        AntennaType::Strength => TRXSystem::Strength { 
            tx_module, 
            rx_module: TRXModule::default()
        }
    }
}

fn default_movement_scenario(
    command_center_id: DeviceId
) -> Vec<(Megahertz, Message)> {
    let goal1 = Goal::Attack(DRONE_DESTINATION);
    let goal2 = Goal::Attack(Point3D::new(0.0, 0.0, 150.0));
    let goal3 = Goal::Attack(Point3D::new(0.0, 150.0, 150.0));
    let goal4 = goal1;

    vec!(
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                UNKNOWN_ID,
                0, 
                MessageType::SetGoal(goal1)
            )
        ),
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                UNKNOWN_ID,
                250, 
                MessageType::SetGoal(goal2) 
            )
        ),
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                UNKNOWN_ID,
                4000, 
                MessageType::SetGoal(goal3) 
            )
        ),
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                UNKNOWN_ID,
                6000, 
                MessageType::SetGoal(goal4) 
            )
        ),
    )
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

fn create_drone_vec(
    network_position: &Point3D,
    x_offset_range: Range<f32>,
    y_offset_range: Range<f32>,
    z_offset_range: Range<f32>,
    drone_count: u32, 
    antenna: &AntennaType,
    tx_control_area_radius: Meter,
    vulnerabilities: &[InfectionType]
) -> Vec<Drone> {
    let mut rng = rand::rng();

    let trx_system = drone_trx_system(antenna, tx_control_area_radius);

    (1..=drone_count)
        .map(|_| {
            let random_offset = Point3D::new(
                rng.random_range(x_offset_range.clone()),
                rng.random_range(y_offset_range.clone()),
                rng.random_range(z_offset_range.clone())
            );
            
            let position = network_position + random_offset;

            let drone_builder = DroneBuilder::new()
                .set_global_position(position)
                .set_trx_system(trx_system.clone());

            if rand::random_bool(VULNERABILITY_PROBABILITY) {
                drone_builder
                    .set_vulnerabilities(vulnerabilities)
                    .build()
            } else {
                drone_builder.build()
            }
        })
        .collect()
}


pub fn gps_only(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius = 300.0;
    let drone_tx_control_area_radius = 50.0;
    let ewd_suppression_area_radius = 100.0; 
        
    let command_center = CommandCenterBuilder::new()
        .set_position(COMMAND_CENTER_POSITION)
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let drones = create_drone_vec(
        &DRONE_NETWORK_POSITION,
        -40.0..40.0,
        -40.0..40.0,
        -20.0..20.0,
        100, 
        &antenna,
        drone_tx_control_area_radius, 
        &[]
    ); 
    let scenario = vec!(
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center.id(),
                UNKNOWN_ID,
                0, 
                MessageType::SetGoal(Goal::Attack(DRONE_DESTINATION))
            )
        ),
    );
    let ewd_gps = ElectronicWarfareBuilder::new()
        .set_position(Point3D::new(0.0, 5.0, 2.0))
        .set_trx_system(
            ewd_trx_system(
                &antenna, 
                GPS_L1_FREQUENCY, 
                ewd_suppression_area_radius
            )
        )
        .build();

    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center(command_center)
        .set_drones(&drones)
        .set_electronic_warfare_devices(&[ewd_gps])
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();

    let output_filename = derive_filename(config, "gps_only");
    let drone_colorings = vec![DroneColoring::SingleColor(0, 0, 0)]; 
    let renderer = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        PLOT_RESOLUTION,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );

    let mut simulation = Simulation::new(
        END_TIME,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn gps_and_control(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius = 300.0;
    let drone_tx_control_area_radius = 50.0;
    let gps_ewd_suppression_area_radius = 100.0; 
    let control_ewd_suppression_area_radius = 50.0;

    let command_center = CommandCenterBuilder::new()
        .set_position(COMMAND_CENTER_POSITION)
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let drones = create_drone_vec(
        &DRONE_NETWORK_POSITION,
        -40.0..40.0,
        -40.0..40.0,
        -20.0..20.0,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        &[]
    ); 
    let scenario = vec!(
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center.id(),
                UNKNOWN_ID,
                0, 
                MessageType::SetGoal(Goal::Attack(DRONE_DESTINATION))
            )
        ),
    );
    let ewd_control = ElectronicWarfareBuilder::new()
        .set_position(Point3D::new(-10.0, 2.0, 0.0))
        .set_trx_system(
            ewd_trx_system(
                &antenna,
                WIFI_2_4GHZ_FREQUENCY,
                control_ewd_suppression_area_radius
            )
        )
        .build();
    let ewd_gps = ElectronicWarfareBuilder::new()
        .set_position(Point3D::new(0.0, 5.0, 2.0))
        .set_trx_system(
            ewd_trx_system(
                &antenna,
                GPS_L1_FREQUENCY,
                gps_ewd_suppression_area_radius
            )
        )
        .build();

    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center(command_center)
        .set_drones(&drones)
        .set_electronic_warfare_devices(&vec![ewd_control, ewd_gps])
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();
 
    let output_filename = derive_filename(config, "gps_and_control");
    let drone_colorings = vec![DroneColoring::SingleColor(0, 0, 0)]; 
    let renderer = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        PLOT_RESOLUTION,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );
    
    let mut simulation = Simulation::new(
        10000,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn command_delay(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius = 300.0;
    let drone_tx_control_area_radius = 50.0;

    let command_center = CommandCenterBuilder::new()
        .set_position(COMMAND_CENTER_POSITION)
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let drones = create_drone_vec(
        &DRONE_NETWORK_POSITION,
        -40.0..40.0,
        -40.0..40.0,
        -20.0..20.0,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        &[]
    ); 
    let scenario = default_movement_scenario(command_center.id()); 

    let mut drone_networks = Vec::new();

    if config.display_delayless_network {
        let delayless_drone_network = NetworkModelBuilder::new(
                NetworkModelType::ComplexNetwork(0.0)
            )
                .set_command_center(command_center.clone())
                .set_drones(&drones)
                .set_topology(config.topology)
                .set_scenario(scenario.clone())
                .build();

        drone_networks.push(delayless_drone_network);
    }
    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center(command_center)
        .set_drones(&drones)
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();
    
    drone_networks.insert(0, drone_network);

    let output_filename = derive_filename(config, "command_delay");
    let drone_colorings = vec![
        DroneColoring::SingleColor(0, 0, 0),
        DroneColoring::SingleColor(255, 0, 0)
    ];
    let renderer = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        PLOT_RESOLUTION,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );

    let mut simulation = Simulation::new(
        END_TIME,
        drone_networks,
        renderer
    );

    simulation.run();
}

pub fn signal_color(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius = match antenna {
        AntennaType::Color => 200.0,
        AntennaType::Strength => 300.0
    };
    let drone_tx_control_area_radius = 50.0;

    let command_center = CommandCenterBuilder::new()
        .set_position(COMMAND_CENTER_POSITION)
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let drones = create_drone_vec(
        &DRONE_NETWORK_POSITION,
        -40.0..40.0,
        -40.0..40.0,
        -20.0..20.0,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        &[]
    ); 
    let scenario = default_movement_scenario(command_center.id()); 

    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center(command_center)
        .set_drones(&drones)
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();

    let output_filename = derive_filename(config, "signal_color");
    let drone_colorings = vec![DroneColoring::Signal];
    let renderer = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        PLOT_RESOLUTION,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );
    
    let mut simulation = Simulation::new(
        END_TIME,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn infection(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius = match antenna {
        AntennaType::Color => 200.0,
        AntennaType::Strength => 300.0
    };
    let drone_tx_control_area_radius = match config.network_model {
        NetworkModelType::CellularAutomaton => 20.0,
        NetworkModelType::ComplexNetwork(_) => 12.5
    };

    let command_center = CommandCenterBuilder::new()
        .set_position(Point3D::new(100.0, 50.0, 0.0))
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let drones = create_drone_vec(
        &Point3D::new(50.0, 50.0, 0.0),
        -40.0..40.0,
        -40.0..40.0,
        0.0..10.0,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        &[InfectionType::Indicator]
    ); 
    let infected_drone_id = drones[0].id();
    let scenario = vec!((
        WIFI_2_4GHZ_FREQUENCY,
        Message::new(
            command_center.id(),
            infected_drone_id,
            0, 
            MessageType::Infection(InfectionType::Indicator)
        ),
    ));

    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center(command_center)
        .set_drones(&drones)
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();

    let output_filename = derive_filename(config, "infection");
    let drone_colorings = vec![DroneColoring::Infection]; 
    let axes_ranges = Axes3DRanges::new(0.0..100.0, 0.0..0.0, 0.0..100.0);
    let renderer = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        PLOT_RESOLUTION,
        axes_ranges,
        &drone_colorings,
        1.57,
        1.57
    );

    let mut simulation = Simulation::new(
        END_TIME,
        vec![drone_network],
        renderer
    );

    simulation.run();
}
