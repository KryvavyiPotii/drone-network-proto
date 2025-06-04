use std::collections::HashMap;
use std::ops::Range;

use rand::prelude::*;

use crate::backend::CONTROL_FREQUENCY;
use crate::backend::connections::Topology;
use crate::backend::device::{
    Device, DeviceBuilder, DeviceId, SignalLossResponse, BROADCAST_ID, 
    MAX_DRONE_SPEED 
};
use crate::backend::device::systems::{
    MovementSystem, PowerSystem, TRXModule, TRXSystem, TRXSystemType
};
use crate::backend::malware::{Malware, MalwareType};
use crate::backend::mathphysics::{Megahertz, Meter, Point3D, PowerUnit};
use crate::backend::message::{Task, Message, MessageType};
use crate::backend::networkmodel::NetworkModelBuilder; 
use crate::backend::networkmodel::attack::{AttackType, AttackerDevice};
use crate::backend::networkmodel::gps::GPS;
use crate::backend::signal::{
    SignalArea, SignalLevel, GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, 
    RED_SIGNAL_LEVEL
};

use super::cli::Config;
use super::simulation::Simulation;
use super::simulation::renderer::{
    Axes3DRanges, DeviceColoring, PlottersRenderer
};


const BIG_SIGNAL_STRENGTH_VALUE: f32 = 100_000.0;

const DEVICE_MAX_POWER: PowerUnit = 10_000;
const GPS_TX_RADIUS: Meter        = 1_000.0;

const DEFAULT_GPS_POSITION_IN_METERS: Point3D = Point3D { 
    x: 0.0, 
    y: 0.0, 
    z: 200.0
};

const NETWORK_ORIGIN: Point3D          = Point3D { x: 150.3, y: 90.6, z: 25.5 };
const DRONE_DESTINATION: Point3D       = Point3D { x: 0.0, y: 0.0, z: 0.0 };
const COMMAND_CENTER_POSITION: Point3D = Point3D { x: 200.0, y: 100.0, z: 0.0 };

const VULNERABILITY_PROBABILITY: f64 = 1.0;


fn cc_trx_system(
    trx_system_type: TRXSystemType, 
    tx_control_area_radius: Meter
) -> TRXSystem {
    let tx_control_area = SignalArea::build(tx_control_area_radius)
        .unwrap();

    let max_tx_signal_levels = HashMap::from([(
        CONTROL_FREQUENCY, 
        SignalLevel::from(BIG_SIGNAL_STRENGTH_VALUE)
    )]);
    let tx_signal_levels = HashMap::from([(
        CONTROL_FREQUENCY,
        SignalLevel::from_area(tx_control_area, CONTROL_FREQUENCY)
    )]);

    let tx_module = TRXModule::build(
        max_tx_signal_levels,
        tx_signal_levels
    ).unwrap();
    let rx_module = tx_module.clone();

    TRXSystem::new( 
        trx_system_type,
        tx_module, 
        rx_module
    )
}

fn drone_trx_system(
    trx_system_type: TRXSystemType, 
    tx_control_area_radius: Meter,
    max_gps_rx_signal_level: SignalLevel
) -> TRXSystem {
    let tx_control_area = SignalArea::build(tx_control_area_radius)
        .unwrap();
    let max_tx_control_signal_level = SignalLevel::from_area(
        tx_control_area, 
        CONTROL_FREQUENCY
    );

    let max_tx_signal_levels = HashMap::from([
        (CONTROL_FREQUENCY, max_tx_control_signal_level),
    ]);
    let max_rx_signal_levels = HashMap::from([
        (CONTROL_FREQUENCY, GREEN_SIGNAL_LEVEL),
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

    TRXSystem::new(
        trx_system_type,
        tx_module, 
        rx_module
    )
}

fn ewd_trx_system(
    trx_system_type: TRXSystemType,
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

    TRXSystem::new( 
        trx_system_type,
        tx_module, 
        TRXModule::default()
    )
}

fn gps_tx_module() -> TRXModule {
    let frequency = GPS_L1_FREQUENCY;
    
    let max_tx_signal_levels = HashMap::from([
        (frequency, SignalLevel::from(BIG_SIGNAL_STRENGTH_VALUE))
    ]);
    let tx_signal_levels = HashMap::from([(
        frequency, 
        SignalLevel::from_area(
            SignalArea::build(GPS_TX_RADIUS).unwrap(), 
            frequency
        )
    )]);

    TRXModule::build(
        max_tx_signal_levels,
        tx_signal_levels,
    ).unwrap()
}

fn default_gps(trx_system_type: TRXSystemType) -> GPS {
    let trx_system = TRXSystem::new( 
        trx_system_type,
        gps_tx_module(),
        TRXModule::default()
    );

    let device = DeviceBuilder::new()
        .set_real_position(DEFAULT_GPS_POSITION_IN_METERS)
        .set_signal_loss_response(SignalLossResponse::Ignore)
        .set_power_system(device_power_system())
        .set_trx_system(trx_system)
        .build();

    GPS::new(device, GPS_L1_FREQUENCY)
}

fn device_power_system() -> PowerSystem {
    PowerSystem::build(DEVICE_MAX_POWER, DEVICE_MAX_POWER)
        .unwrap_or_else(|error| panic!("{}", error))
}

fn device_movement_system() -> MovementSystem {
    MovementSystem::build(MAX_DRONE_SPEED)
        .unwrap_or_else(|error| panic!("{}", error))
}

fn default_movement_scenario(
    command_center_id: DeviceId
) -> Vec<(Megahertz, Message)> {
    let task1 = Task::Attack(DRONE_DESTINATION);
    let task2 = Task::Attack(Point3D::new(0.0, 0.0, 150.0));
    let task3 = Task::Attack(Point3D::new(0.0, 150.0, 150.0));
    let task4 = task1;

    vec!(
        (
            CONTROL_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                0, 
                MessageType::SetTask(task1)
            )
        ),
        (
            CONTROL_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                250, 
                MessageType::SetTask(task2) 
            )
        ),
        (
            CONTROL_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                4000, 
                MessageType::SetTask(task3) 
            )
        ),
        (
            CONTROL_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                6000, 
                MessageType::SetTask(task4) 
            )
        ),
    )
}

fn derive_filename(config: &Config, text: &str) -> String {
    let trx_system_part = match config.trx_system_type {
        TRXSystemType::Color    => "col",
        TRXSystemType::Strength => "str",
    };
    let topology_part = match config.topology {
        Topology::Mesh => "mesh",
        Topology::Star => "star",
    };

    format!("{trx_system_part}_{text}_{topology_part}.gif")
}

fn create_device_vec(
    command_device: Device,
    network_position: &NetworkPosition,
    device_count: u32, 
    trx_system_type: TRXSystemType,
    tx_control_area_radius: Meter,
    max_gps_rx_signal_level: SignalLevel,
    vulnerabilities: &[Malware]
) -> Vec<Device> {
    let mut rng = rand::rng();

    let power_system    = device_power_system();
    let movement_system = device_movement_system();
    let trx_system      = drone_trx_system(
        trx_system_type, 
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
                .set_power_system(power_system.clone())
                .set_movement_system(movement_system.clone())
                .set_trx_system(trx_system.clone())
                .set_signal_loss_response(SignalLossResponse::Shutdown);

            let drone_builder = if rand::random_bool(
                VULNERABILITY_PROBABILITY
            ) {
                drone_builder.set_vulnerabilities(vulnerabilities)
            } else {
                drone_builder
            };

            drone_builder.build()
        })
        .collect();

    devices.insert(0, command_device);

    devices
}

fn indicator_malware() -> Malware {
    Malware::new(
        500, 
        MalwareType::Indicator,
        true,
    )
}

fn jamming_malware(jammed_frequency: Megahertz) -> Malware {
    Malware::new(
        500, 
        MalwareType::Jamming(jammed_frequency),
        true,
    )
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
    let cc_tx_control_area_radius    = 300.0;
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = RED_SIGNAL_LEVEL; 
    let ewd_suppression_area_radius  = 100.0; 
        
    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
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
        config.trx_system_type,
        drone_tx_control_area_radius, 
        drone_gps_rx_signal_level, 
        &[]
    ); 
    let scenario = vec!(
        (
            CONTROL_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                0, 
                MessageType::SetTask(Task::Attack(DRONE_DESTINATION))
            )
        ),
    );
    let ewd_gps = DeviceBuilder::new()
        .set_real_position(Point3D::new(0.0, 5.0, 2.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            ewd_trx_system(
                config.trx_system_type, 
                GPS_L1_FREQUENCY, 
                ewd_suppression_area_radius
            )
        )
        .build();
    let attacker_devices = [
        AttackerDevice::new(ewd_gps, AttackType::ElectronicWarfare)
    ];

    let drone_network = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_attacker_devices(&attacker_devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();

    let output_filename = derive_filename(config, "gps_only");
    let drone_colorings = vec![DeviceColoring::SingleColor(0, 0, 0)]; 
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );

    let mut simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn gps_and_control(config: &Config) {
    let cc_tx_control_area_radius           = 300.0;
    let drone_tx_control_area_radius        = 50.0;
    let drone_gps_rx_signal_level           = RED_SIGNAL_LEVEL; 
    let gps_ewd_suppression_area_radius     = 100.0; 
    let control_ewd_suppression_area_radius = 50.0;

    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
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
        config.trx_system_type,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[]
    ); 
    let scenario = vec!(
        (
            CONTROL_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                0, 
                MessageType::SetTask(Task::Attack(DRONE_DESTINATION))
            )
        ),
    );
    let ewd_control = DeviceBuilder::new()
        .set_real_position(Point3D::new(-10.0, 2.0, 0.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            ewd_trx_system(
                config.trx_system_type,
                CONTROL_FREQUENCY,
                control_ewd_suppression_area_radius
            )
        )
        .build();
    let ewd_gps = DeviceBuilder::new()
        .set_real_position(Point3D::new(0.0, 5.0, 2.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            ewd_trx_system(
                config.trx_system_type,
                GPS_L1_FREQUENCY,
                gps_ewd_suppression_area_radius
            )
        )
        .build();
    let attacker_devices = [
        AttackerDevice::new(ewd_control, AttackType::ElectronicWarfare),
        AttackerDevice::new(ewd_gps, AttackType::ElectronicWarfare)
    ];

    let drone_network = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_attacker_devices(&attacker_devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();
 
    let output_filename = derive_filename(config, "gps_and_control"); 
    let drone_colorings = vec![DeviceColoring::SingleColor(0, 0, 0)]; 
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );
    
    let mut simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn command_delay(config: &Config) {
    let cc_tx_control_area_radius    = 1000.0;
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 

    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
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
        config.trx_system_type,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[]
    ); 
    let scenario = default_movement_scenario(command_center_id); 

    let mut drone_networks = Vec::new();

    if config.display_delayless_network {
        let delayless_drone_network = NetworkModelBuilder::new()
            .set_command_center_id(command_center_id)
            .set_devices(&devices)
            .set_gps(default_gps(config.trx_system_type))
            .set_topology(config.topology)
            .set_scenario(scenario.clone())
            .build();

        drone_networks.push(delayless_drone_network);
    }
    let drone_network = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();
    
    drone_networks.insert(0, drone_network);

    let output_filename = derive_filename(config, "command_delay");
    let drone_colorings = if config.display_delayless_network {
        vec![
            DeviceColoring::SingleColor(0, 0, 0),
            DeviceColoring::SingleColor(255, 0, 0)
        ]
    } else {
        vec![DeviceColoring::SingleColor(0, 0, 0)]
    };
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );

    let mut simulation = Simulation::new(
        config.simulation_time,
        drone_networks,
        renderer
    );

    simulation.run();
}

pub fn signal_color(config: &Config) {
    let cc_tx_control_area_radius    = match config.trx_system_type {
        TRXSystemType::Color    => 100.0,
        TRXSystemType::Strength => 300.0
    };
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 

    let command_center = DeviceBuilder::new()
        .set_real_position(Point3D::new(100.0, 50.0, 0.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
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
        config.trx_system_type,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[]
    ); 
    let scenario = Vec::new(); 

    let drone_network = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();

    let output_filename = derive_filename(config, "signal_color");
    let drone_colorings = vec![DeviceColoring::Signal];
    let axes_ranges     = Axes3DRanges::new(0.0..100.0, 0.0..0.0, 0.0..100.0);
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
        axes_ranges,
        &drone_colorings,
        1.57,
        1.57
    );
    
    let mut simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn signal_color_dynamic(config: &Config) {
    let cc_tx_control_area_radius    = match config.trx_system_type {
        TRXSystemType::Color    => 200.0,
        TRXSystemType::Strength => 600.0
    };
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 

    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
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
        config.trx_system_type,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[]
    ); 
    let scenario = default_movement_scenario(command_center_id); 

    let drone_network = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();

    let output_filename = derive_filename(config, "signal_color_dynamic");
    let drone_colorings = vec![DeviceColoring::Signal];
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );
    
    let mut simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn infection(config: &Config) {
    let cc_tx_control_area_radius    = match config.trx_system_type {
        TRXSystemType::Color    => 200.0,
        TRXSystemType::Strength => 300.0
    };
    let drone_tx_control_area_radius = 25.0;
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 
    let indicator_malware = indicator_malware();

    let command_center = DeviceBuilder::new()
        .set_real_position(Point3D::new(100.0, 50.0, 0.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
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
        config.trx_system_type,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[indicator_malware]
    ); 
    let infected_drone_id = devices[1].id();
    let scenario = vec!((
        CONTROL_FREQUENCY,
        Message::new(
            command_center_id,
            infected_drone_id,
            0, 
            MessageType::Malware(indicator_malware)
        ),
    ));

    let drone_network = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();

    let output_filename = derive_filename(config, "infection");
    let drone_colorings = vec![DeviceColoring::Infection]; 
    let axes_ranges     = Axes3DRanges::new(0.0..100.0, 0.0..0.0, 0.0..100.0);
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
        axes_ranges,
        &drone_colorings,
        1.57,
        1.57
    );

    let mut simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn jamming_infection(config: &Config) {
    let cc_tx_control_area_radius    = match config.trx_system_type {
        TRXSystemType::Color    => 200.0,
        TRXSystemType::Strength => 300.0
    };
    let drone_tx_control_area_radius = match config.trx_system_type {
        TRXSystemType::Color     => 12.5,
        TRXSystemType::Strength => 30.0,
    };
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 
    let indicator_malware = indicator_malware();
    let jamming_malware   = jamming_malware(CONTROL_FREQUENCY);

    let command_center = DeviceBuilder::new()
        .set_real_position(Point3D::new(100.0, 50.0, 0.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
        .build();
    let command_center_id = command_center.id();

    let network_position = NetworkPosition::new(
        Point3D::new(50.0, 50.0, 0.0),
        -40.0..40.0,
        -40.0..40.0,
        0.0..10.0,
    );
    let vulnerabilities = [indicator_malware, jamming_malware];
    let devices = create_device_vec(
        command_center,
        &network_position,
        100, 
        config.trx_system_type,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &vulnerabilities
    ); 
    let infected_drone_id = devices[1].id();
    let indicator_scenario = vec!((
        CONTROL_FREQUENCY,
        Message::new(
            command_center_id,
            infected_drone_id,
            0, 
            MessageType::Malware(indicator_malware)
        ),
    ));
    let jamming_scenario = vec!((
        CONTROL_FREQUENCY,
        Message::new(
            command_center_id,
            infected_drone_id,
            0, 
            MessageType::Malware(jamming_malware)
        ),
    ));

    let drone_network_indicator = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(indicator_scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();
    let drone_network_jamming = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(jamming_scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();

    let output_filename = derive_filename(config, "infection_indicator");
    let drone_colorings = vec![DeviceColoring::Infection]; 
    let axes_ranges     = Axes3DRanges::new(0.0..100.0, 0.0..0.0, 0.0..100.0);
    let indicator_renderer = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
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
        config.plot_resolution,
        axes_ranges,
        &drone_colorings,
        1.57,
        1.57
    );

    let mut indicator_simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network_indicator],
        indicator_renderer
    );
    let mut jamming_simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network_jamming],
        jamming_renderer
    );

    // CTRL+C handler in `simulation` should be disabled in order for that 
    // example to work.
    indicator_simulation.run();
    jamming_simulation.run();
}

pub fn gps_spoofing(config: &Config) {
    let cc_tx_control_area_radius    = 300.0;
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = RED_SIGNAL_LEVEL; 
    let gps_spoofing_area_radius     = 200.0; 
        
    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
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
        2,
        config.trx_system_type,
        drone_tx_control_area_radius, 
        drone_gps_rx_signal_level, 
        &[]
    ); 
    let scenario = vec!(
        (
            CONTROL_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                0, 
                MessageType::SetTask(Task::Attack(DRONE_DESTINATION))
            )
        ),
    );
    let ewd_gps = DeviceBuilder::new()
        .set_real_position(Point3D::new(0.0, 5.0, 2.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            ewd_trx_system(
                config.trx_system_type, 
                GPS_L1_FREQUENCY, 
                gps_spoofing_area_radius
            )
        )
        .build();
    let spoofed_position = Point3D::new(-200.0, -100.0, -200.0);
    let attacker_devices = [
        AttackerDevice::new(ewd_gps, AttackType::GPSSpoofing(spoofed_position))
    ];

    let drone_network = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_attacker_devices(&attacker_devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();

    let output_filename = derive_filename(config, "gps_spoofing");
    let axes_ranges     = Axes3DRanges::new(0.0..200.0, 0.0..0.0, 0.0..200.0);
    let drone_colorings = vec![DeviceColoring::SingleColor(0, 0, 0)]; 
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
        axes_ranges,
        &drone_colorings,
        1.57,
        1.57
    );

    let mut simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn dos(config: &Config) {
    let cc_tx_control_area_radius    = match config.trx_system_type {
        TRXSystemType::Color    => 100.0,
        TRXSystemType::Strength => 300.0
    };
    let drone_tx_control_area_radius = match config.trx_system_type {
        TRXSystemType::Color  => 20.0,
        TRXSystemType::Strength => 30.0,
    };
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 
    let dos_area_radius = 50.0;
    let dos_malware     = Malware::new(
        500, 
        MalwareType::DoS(DEVICE_MAX_POWER), 
        true
    );

    let command_center = DeviceBuilder::new()
        .set_real_position(Point3D::new(100.0, 50.0, 0.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
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
        config.trx_system_type,
        drone_tx_control_area_radius,
        drone_gps_rx_signal_level,
        &[dos_malware]
    ); 
    
    let dos_attacker = DeviceBuilder::new()
        .set_real_position(Point3D::new(-10.0, 2.0, 0.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            ewd_trx_system(
                config.trx_system_type,
                CONTROL_FREQUENCY,
                dos_area_radius
            )
        )
        .build();
    let attacker_devices = [
        AttackerDevice::new(
            dos_attacker, 
            AttackType::MalwareDistribution(dos_malware)
        )
    ];

    let drone_network = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_attacker_devices(&attacker_devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_delay_multiplier(config.delay_multiplier)
        .build();

    let output_filename = derive_filename(config, "dos");
    let drone_colorings = vec![DeviceColoring::Signal]; 
    let axes_ranges     = Axes3DRanges::new(0.0..100.0, 0.0..0.0, 0.0..100.0);
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
        axes_ranges,
        &drone_colorings,
        1.57,
        1.57
    );

    let mut simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}

pub fn signal_loss_response(config: &Config) {
    let cc_tx_control_area_radius    = 600.0;
    let drone_tx_control_area_radius = 50.0;
    let drone_gps_rx_signal_level    = GREEN_SIGNAL_LEVEL; 
    let control_ewd_suppression_area_radius = 150.0;

    let command_center = DeviceBuilder::new()
        .set_real_position(COMMAND_CENTER_POSITION)
        .set_power_system(device_power_system())
        .set_trx_system(
            cc_trx_system(
                config.trx_system_type, 
                cc_tx_control_area_radius
            )
        )
        .set_signal_loss_response(SignalLossResponse::Ignore)
        .build();
    let command_center_id = command_center.id();
   
    let drone_builder = DeviceBuilder::new()
        .set_real_position(NETWORK_ORIGIN)
        .set_power_system(device_power_system())
        .set_movement_system(device_movement_system())
        .set_trx_system(
            drone_trx_system(
                config.trx_system_type, 
                drone_tx_control_area_radius, 
                drone_gps_rx_signal_level
            )
        );

    let ascend_drone = drone_builder
        .clone()
        .set_signal_loss_response(SignalLossResponse::Ascend)
        .build();
    let hover_drone = drone_builder
        .clone()
        .set_signal_loss_response(SignalLossResponse::Hover)
        .build();
    let rth_drone = drone_builder
        .clone()
        .set_signal_loss_response(
            SignalLossResponse::ReturnToHome(COMMAND_CENTER_POSITION)
        )
        .build();
    let shutdown_drone = drone_builder
        .set_signal_loss_response(SignalLossResponse::Shutdown)
        .build();

    let devices = [
        command_center, 
        ascend_drone, 
        hover_drone, 
        rth_drone, 
        shutdown_drone
    ]; 
    
    let scenario = vec!(
        (
            CONTROL_FREQUENCY,
            Message::new(
                command_center_id,
                BROADCAST_ID,
                0, 
                MessageType::SetTask(Task::Attack(DRONE_DESTINATION))
            )
        ),
    );

    let ewd_control = DeviceBuilder::new()
        .set_real_position(Point3D::new(-10.0, 2.0, 0.0))
        .set_power_system(device_power_system())
        .set_trx_system(
            ewd_trx_system(
                config.trx_system_type,
                CONTROL_FREQUENCY,
                control_ewd_suppression_area_radius
            )
        )
        .build();
    let attacker_devices = [
        AttackerDevice::new(ewd_control, AttackType::ElectronicWarfare)
    ];
    
    let drone_network = NetworkModelBuilder::new()
        .set_command_center_id(command_center_id)
        .set_devices(&devices)
        .set_attacker_devices(&attacker_devices)
        .set_gps(default_gps(config.trx_system_type))
        .set_topology(config.topology)
        .set_scenario(scenario)
        .set_delay_multiplier(config.delay_multiplier)
        .build();
 
    let output_filename = derive_filename(config, "signal_loss_response"); 
    let drone_colorings = vec![DeviceColoring::Signal]; 
    let renderer        = PlottersRenderer::new(
        &output_filename,
        &config.plot_caption,
        config.plot_resolution,
        Axes3DRanges::default(),
        &drone_colorings,
        0.15,
        0.5,
    );
    
    let mut simulation = Simulation::new(
        config.simulation_time,
        vec![drone_network],
        renderer
    );

    simulation.run();
}
