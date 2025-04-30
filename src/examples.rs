use std::collections::HashMap;
use std::ops::Range;

use rand::prelude::*;

use crate::device::{
    BROADCAST_ID, Device, DeviceBuilder, DeviceId, Topology, MAX_DRONE_SPEED, 
};
use crate::device::networkmodel::{NetworkModelBuilder, NetworkModelType}; 
use crate::device::systems::{MovementSystem, TRXModule, TRXSystem};
use crate::infection::InfectionType;
use crate::mathphysics::{Megahertz, Meter, Millisecond, Point3D};
use crate::message::{Goal, Message, MessageType};
use crate::signal::{
    SignalArea, SignalLevel, GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, 
    RED_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY
};
use crate::simulation::Simulation;
use crate::simulation::renderer::{
    Axes3DRanges, DeviceColoring, PlottersRenderer
};

use super::{AntennaType, Config};


const BIG_SIGNAL_STRENGTH_VALUE: f32 = 100_000.0;

const NETWORK_ORIGIN: Point3D          = Point3D { x: 150.3, y: 90.6, z: 25.5 };
const DRONE_DESTINATION: Point3D       = Point3D { x: 0.0, y: 0.0, z: 0.0 };
const COMMAND_CENTER_POSITION: Point3D = Point3D { x: 200.0, y: 100.0, z: 0.0 };

const START_TIME: Millisecond = 0;

const PLOT_RESOLUTION: (u16, u16) = (400, 300);

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
        AntennaType::Color    => TRXSystem::Color(tx_module),
        AntennaType::Strength => TRXSystem::Strength { 
            tx_module, 
            rx_module: TRXModule::default()
        }
    }
}

fn drone_trx_system(
    antenna: &AntennaType, 
    tx_control_area_radius: Meter,
    max_gps_rx_signal_level: SignalLevel
) -> TRXSystem {
    let tx_control_area = SignalArea::build(tx_control_area_radius)
        .unwrap();
    let max_tx_control_signal_level = SignalLevel::from_area(
        tx_control_area, 
        WIFI_2_4GHZ_FREQUENCY
    );

    match antenna {
        AntennaType::Color    => {
            let max_signal_levels =HashMap::from([
                (WIFI_2_4GHZ_FREQUENCY, max_tx_control_signal_level),
                (GPS_L1_FREQUENCY, max_gps_rx_signal_level)
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
                (GPS_L1_FREQUENCY, max_gps_rx_signal_level)
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
        AntennaType::Color    => TRXSystem::Color(tx_module),
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
                BROADCAST_ID,
                0, 
                MessageType::SetGoal(goal1)
            )
        ),
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                250, 
                MessageType::SetGoal(goal2) 
            )
        ),
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                4000, 
                MessageType::SetGoal(goal3) 
            )
        ),
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
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

fn create_device_vec(
    command_device: Device,
    network_position: &NetworkPosition,
    device_count: u32, 
    antenna: &AntennaType,
    tx_control_area_radius: Meter,
    max_gps_rx_signal_level: SignalLevel,
    vulnerabilities: &[InfectionType]
) -> Vec<Device> {
    let mut rng = rand::rng();

    let movement_system = MovementSystem::build(MAX_DRONE_SPEED)
        .unwrap_or_else(|error| panic!("{}", error));
    let trx_system = drone_trx_system(
        antenna, 
        tx_control_area_radius,
        max_gps_rx_signal_level
    );

    let mut devices: Vec<Device> = (1..device_count)
        .map(|_| {
            let random_offset = Point3D::new(
                rng.random_range(network_position.x_offset_range.clone()),
                rng.random_range(network_position.y_offset_range.clone()),
                rng.random_range(network_position.z_offset_range.clone())
            );
            
            let position = network_position.origin + random_offset;

            let drone_builder = DeviceBuilder::new()
                .set_real_position(position)
                .set_movement_system(movement_system.clone())
                .set_trx_system(trx_system.clone());

            let drone_builder = if rand::random_bool(
                VULNERABILITY_PROBABILITY
            ) {
                drone_builder
                    .set_vulnerabilities(vulnerabilities)
            } else {
                drone_builder
            };

            drone_builder.build()
        })
        .collect();

    devices.insert(0, command_device);

    devices
}


struct NetworkPosition {
    origin: Point3D,
    x_offset_range: Range<f32>,
    y_offset_range: Range<f32>,
    z_offset_range: Range<f32>,
}

impl NetworkPosition {
    #[must_use]
    fn new(
        origin: Point3D,
        x_offset_range: Range<f32>,
        y_offset_range: Range<f32>,
        z_offset_range: Range<f32>,
    ) -> Self {
        Self { 
            origin, 
            x_offset_range,
            y_offset_range,
            z_offset_range
        }
    }
}


pub fn gps_only(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius    = 300.0;
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = RED_SIGNAL_LEVEL; 
    let ewd_suppression_area_radius  = 100.0; 
        
    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let command_center_id = command_center.id();

    let network_position = NetworkPosition::new(
        NETWORK_ORIGIN,
        -40.0..40.0,
        -40.0..40.0,
        -20.0..20.0,
    );
    let devices = create_device_vec(
        command_center,
        &network_position,
        100,
        &antenna,
        drone_tx_control_area_radius, 
        drone_gps_rx_signal_level, 
        &[]
    ); 
    let scenario = vec!(
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                0, 
                MessageType::SetGoal(Goal::Attack(DRONE_DESTINATION))
            )
        ),
    );
    let ewd_gps = DeviceBuilder::new()
        .set_real_position(Point3D::new(0.0, 5.0, 2.0))
        .set_trx_system(
            ewd_trx_system(
                &antenna, 
                GPS_L1_FREQUENCY, 
                ewd_suppression_area_radius
            )
        )
        .build();

    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_electronic_warfare_devices(&[ewd_gps])
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();

    let end_time        = 15_000;

    let output_filename = derive_filename(config, "gps_only");
    let drone_colorings = vec![DeviceColoring::SingleColor(0, 0, 0)]; 
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        PLOT_RESOLUTION,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );

    let mut simulation = Simulation::new(
        START_TIME,
        end_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn gps_and_control(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius           = 300.0;
    let drone_tx_control_area_radius        = 50.0;
    let drone_gps_rx_signal_level           = RED_SIGNAL_LEVEL; 
    let gps_ewd_suppression_area_radius     = 100.0; 
    let control_ewd_suppression_area_radius = 50.0;

    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let command_center_id = command_center.id();
    
    let network_position = NetworkPosition::new(
        NETWORK_ORIGIN,
        -40.0..40.0,
        -40.0..40.0,
        -20.0..20.0,
    );
    let devices = create_device_vec(
        command_center,
        &network_position,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[]
    ); 
    let scenario = vec!(
        (
            WIFI_2_4GHZ_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                0, 
                MessageType::SetGoal(Goal::Attack(DRONE_DESTINATION))
            )
        ),
    );
    let ewd_control = DeviceBuilder::new()
        .set_real_position(Point3D::new(-10.0, 2.0, 0.0))
        .set_trx_system(
            ewd_trx_system(
                &antenna,
                WIFI_2_4GHZ_FREQUENCY,
                control_ewd_suppression_area_radius
            )
        )
        .build();
    let ewd_gps = DeviceBuilder::new()
        .set_real_position(Point3D::new(0.0, 5.0, 2.0))
        .set_trx_system(
            ewd_trx_system(
                &antenna,
                GPS_L1_FREQUENCY,
                gps_ewd_suppression_area_radius
            )
        )
        .build();

    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_electronic_warfare_devices(&vec![ewd_control, ewd_gps])
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();
 
    let end_time        = 10_000;

    let output_filename = derive_filename(config, "gps_and_control"); 
    let drone_colorings = vec![DeviceColoring::SingleColor(0, 0, 0)]; 
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        PLOT_RESOLUTION,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );
    
    let mut simulation = Simulation::new(
        START_TIME,
        end_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn command_delay(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius    = 300.0;
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 

    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let command_center_id = command_center.id();

    let network_position = NetworkPosition::new(
        NETWORK_ORIGIN,
        -40.0..40.0,
        -40.0..40.0,
        -20.0..20.0,
    );
    let devices = create_device_vec(
        command_center,
        &network_position,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[]
    ); 
    let scenario = default_movement_scenario(command_center_id); 

    let mut drone_networks = Vec::new();

    if config.display_delayless_network {
        let delayless_drone_network = NetworkModelBuilder::new(
                NetworkModelType::ComplexNetwork(0.0)
            )
            .set_command_center_id(command_center_id)
            .set_devices(&devices)
            .set_topology(config.topology)
            .set_scenario(scenario.clone())
            .build();

        drone_networks.push(delayless_drone_network);
    }
    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();
    
    drone_networks.insert(0, drone_network);

    let end_time        = 15_000;

    let output_filename = derive_filename(config, "command_delay");
    let drone_colorings = vec![
        DeviceColoring::SingleColor(0, 0, 0),
        DeviceColoring::SingleColor(255, 0, 0)
    ];
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        PLOT_RESOLUTION,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );

    let mut simulation = Simulation::new(
        START_TIME,
        end_time,
        drone_networks,
        renderer
    );

    simulation.run();
}

pub fn signal_color(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius    = match antenna {
        AntennaType::Color    => 100.0,
        AntennaType::Strength => 300.0
    };
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 

    let command_center = DeviceBuilder::new()
        .set_real_position(Point3D::new(100.0, 50.0, 0.0))
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let command_center_id = command_center.id();

    let network_position = NetworkPosition::new(
        Point3D::new(50.0, 50.0, 0.0),
        -40.0..40.0,
        -40.0..40.0,
        0.0..10.0,
    );
    let devices = create_device_vec(
        command_center,
        &network_position,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[]
    ); 
    let scenario = Vec::new(); 

    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();

    let end_time        = 50;
    
    let output_filename = derive_filename(config, "signal_color");
    let plot_resolution = (800, 800);
    let drone_colorings = vec![DeviceColoring::Signal];
    let axes_ranges     = Axes3DRanges::new(0.0..100.0, 0.0..0.0, 0.0..100.0);
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        plot_resolution,
        axes_ranges,
        &drone_colorings,
        1.57,
        1.57
    );
    
    let mut simulation = Simulation::new(
        START_TIME,
        end_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn signal_color_dynamic(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius    = match antenna {
        AntennaType::Color    => 200.0,
        AntennaType::Strength => 600.0
    };
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 

    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let command_center_id = command_center.id();

    let network_position = NetworkPosition::new(
        NETWORK_ORIGIN,
        -40.0..40.0,
        -40.0..40.0,
        -20.0..20.0,
    );
    let devices = create_device_vec(
        command_center,
        &network_position,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[]
    ); 
    let scenario = default_movement_scenario(command_center_id); 

    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();

    let end_time        = 15_000;

    let output_filename = derive_filename(config, "signal_color_dynamic");
    let drone_colorings = vec![DeviceColoring::Signal];
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        PLOT_RESOLUTION,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );
    
    let mut simulation = Simulation::new(
        START_TIME,
        end_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn infection(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius    = match antenna {
        AntennaType::Color    => 200.0,
        AntennaType::Strength => 300.0
    };
    let drone_tx_control_area_radius = match config.network_model {
        NetworkModelType::CellularAutomaton => 20.0,
        NetworkModelType::ComplexNetwork(_) => 12.5
    };
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 

    let command_center = DeviceBuilder::new()
        .set_real_position(Point3D::new(100.0, 50.0, 0.0))
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let command_center_id = command_center.id();

    let network_position = NetworkPosition::new(
        Point3D::new(50.0, 50.0, 0.0),
        -40.0..40.0,
        -40.0..40.0,
        0.0..10.0,
    );
    let devices = create_device_vec(
        command_center,
        &network_position,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[InfectionType::Indicator]
    ); 
    let infected_drone_id = devices[1].id();
    let scenario = vec!((
        WIFI_2_4GHZ_FREQUENCY,
        Message::new(
            command_center_id,
            infected_drone_id,
            0, 
            MessageType::Infection(InfectionType::Indicator)
        ),
    ));

    let drone_network = NetworkModelBuilder::new(config.network_model)
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_topology(config.topology)
        .set_scenario(scenario)
        .build();

    let end_time        = 15_000;
    
    let output_filename = derive_filename(config, "infection");
    let plot_resolution = (800, 800);
    let drone_colorings = vec![DeviceColoring::Infection]; 
    let axes_ranges     = Axes3DRanges::new(0.0..100.0, 0.0..0.0, 0.0..100.0);
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        plot_resolution,
        axes_ranges,
        &drone_colorings,
        1.57,
        1.57
    );

    let mut simulation = Simulation::new(
        START_TIME,
        end_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn jamming_infection(config: &Config) {
    let antenna = config.antenna();
    let cc_tx_control_area_radius    = match antenna {
        AntennaType::Color    => 200.0,
        AntennaType::Strength => 300.0
    };
    let drone_tx_control_area_radius = match config.network_model {
        NetworkModelType::CellularAutomaton => 20.0,
        NetworkModelType::ComplexNetwork(_) => 12.5
    };
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 

    let command_center = DeviceBuilder::new()
        .set_real_position(Point3D::new(100.0, 50.0, 0.0))
        .set_trx_system(cc_trx_system(&antenna, cc_tx_control_area_radius))
        .build();
    let command_center_id = command_center.id();

    let network_position = NetworkPosition::new(
        Point3D::new(50.0, 50.0, 0.0),
        -40.0..40.0,
        -40.0..40.0,
        0.0..10.0,
    );
    let vulnerabilities = [
        InfectionType::Indicator, 
        InfectionType::Jamming(WIFI_2_4GHZ_FREQUENCY)
    ];
    let devices = create_device_vec(
        command_center,
        &network_position,
        100, 
        &antenna,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &vulnerabilities
    ); 
    let infected_drone_id = devices[1].id();
    let indicator_scenario = vec!((
        WIFI_2_4GHZ_FREQUENCY,
        Message::new(
            command_center_id,
            infected_drone_id,
            0, 
            MessageType::Infection(InfectionType::Indicator)
        ),
    ));
    let jamming_scenario = vec!((
        WIFI_2_4GHZ_FREQUENCY,
        Message::new(
            command_center_id,
            infected_drone_id,
            0, 
            MessageType::Infection(
                InfectionType::Jamming(WIFI_2_4GHZ_FREQUENCY)
            )
        ),
    ));

    let drone_network_indicator = NetworkModelBuilder::new(config.network_model)
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_topology(config.topology)
        .set_scenario(indicator_scenario)
        .build();
    let drone_network_jamming = NetworkModelBuilder::new(config.network_model)
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_topology(config.topology)
        .set_scenario(jamming_scenario)
        .build();

    let end_time        = 15_000;
    
    let output_filename = derive_filename(config, "infection_indicator");
    let plot_resolution = (800, 800);
    let drone_colorings = vec![DeviceColoring::Infection]; 
    let axes_ranges     = Axes3DRanges::new(0.0..100.0, 0.0..0.0, 0.0..100.0);
    let indicator_renderer = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        plot_resolution,
        axes_ranges.clone(),
        &drone_colorings,
        1.57,
        1.57
    );
    
    let output_filename = derive_filename(config, "infection_jamming");
    let drone_colorings = vec![DeviceColoring::Signal]; 
    let jamming_renderer = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        plot_resolution,
        axes_ranges,
        &drone_colorings,
        1.57,
        1.57
    );

    let mut indicator_simulation = Simulation::new(
        START_TIME,
        end_time,
        vec![drone_network_indicator],
        indicator_renderer
    );
    let mut jamming_simulation = Simulation::new(
        START_TIME,
        end_time,
        vec![drone_network_jamming],
        jamming_renderer
    );

    // CTRL+C handler in `simulation` should be disabled in order for that 
    // example to work.
    indicator_simulation.run();
    jamming_simulation.run();
}
