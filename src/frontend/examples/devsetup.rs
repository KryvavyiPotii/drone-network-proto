use std::collections::HashMap;
use std::ops::Range;

use rand::prelude::*;

use crate::backend::CONTROL_FREQUENCY;
use crate::backend::device::{
    Device, DeviceBuilder, SignalLossResponse, MAX_DRONE_SPEED 
};
use crate::backend::device::systems::{
    MovementSystem, PowerSystem, TRXModule, TRXSystem, TRXSystemType
};
use crate::backend::malware::Malware;
use crate::backend::mathphysics::{Megahertz, Meter, Point3D, PowerUnit};
use crate::backend::networkmodel::gps::GPS;
use crate::backend::signal::{
    SignalArea, SignalLevel, GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, 
};


pub const DEVICE_MAX_POWER: PowerUnit = 10_000;

const BIG_SIGNAL_STRENGTH_VALUE: f32 = 100_000.0;
const VULNERABILITY_PROBABILITY: f64 = 1.0;

const GPS_TX_RADIUS: Meter = 1_000.0;
const DEFAULT_GPS_POSITION_IN_METERS: Point3D = Point3D { 
    x: 0.0, 
    y: 0.0, 
    z: 200.0
};


pub fn generate_drone_positions(
    drone_count: usize,
    network_position: &NetworkPosition,
) -> Vec<Point3D> {
    let mut rng = rand::rng();

    (1..=drone_count)
        .map(|_| {
            let random_offset = Point3D::new(
                rng.random_range(network_position.x_offset_range.clone()),
                rng.random_range(network_position.y_offset_range.clone()),
                rng.random_range(network_position.z_offset_range.clone())
            );
            
            network_position.origin + random_offset
        })
        .collect()
}

pub fn generate_drone_vulnerabilities(
    drone_count: usize,
    vulnerabilities: &[Malware],
) -> Vec<Vec<Malware>> {
    (1..=drone_count)
        .map(|_| {
            if rand::random_bool(VULNERABILITY_PROBABILITY) {
                Vec::from(vulnerabilities)
            } else {
                Vec::new()
            }
        })
        .collect()
}

pub fn create_drone_vec(
    drone_count: usize, 
    drone_positions: &[Point3D],
    vulnerabilities: &[Vec<Malware>],
    trx_system_type: TRXSystemType,
    tx_control_area_radius: Meter,
    max_gps_rx_signal_level: SignalLevel,
) -> Vec<Device> {
    assert_eq!(drone_count, drone_positions.len());
    assert_eq!(drone_count, vulnerabilities.len());

    let power_system    = device_power_system();
    let movement_system = device_movement_system();
    let trx_system      = drone_trx_system(
        trx_system_type, 
        tx_control_area_radius,
        max_gps_rx_signal_level
    );

    let drone_builder = DeviceBuilder::new()
        .set_power_system(power_system)
        .set_movement_system(movement_system)
        .set_trx_system(trx_system)
        .set_signal_loss_response(SignalLossResponse::Shutdown);

    (0..drone_count)
        .map(|i| {
            let drone_builder = drone_builder.clone();

            drone_builder
                .set_real_position(drone_positions[i])
                .set_vulnerabilities(&vulnerabilities[i])
                .build()
        })  
        .collect()
}

pub fn cc_trx_system(
    trx_system_type: TRXSystemType, 
    tx_control_area_radius: Meter
) -> TRXSystem {
    let tx_module = cc_tx_module(tx_control_area_radius);
    let rx_module = tx_module.clone();

    TRXSystem::new( 
        trx_system_type,
        tx_module, 
        rx_module
    )
}

pub fn cc_tx_module(tx_control_area_radius: Meter) -> TRXModule {
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

    TRXModule::build(
        max_tx_signal_levels,
        tx_signal_levels
    ).unwrap()
}

pub fn drone_trx_system(
    trx_system_type: TRXSystemType, 
    tx_control_area_radius: Meter,
    max_gps_rx_signal_level: SignalLevel
) -> TRXSystem {
    TRXSystem::new(
        trx_system_type,
        drone_tx_module(tx_control_area_radius), 
        drone_rx_module(max_gps_rx_signal_level),
    )
}

pub fn drone_tx_module(tx_control_area_radius: Meter) -> TRXModule {
    let tx_control_area = SignalArea::build(tx_control_area_radius)
        .unwrap();
    let max_tx_control_signal_level = SignalLevel::from_area(
        tx_control_area, 
        CONTROL_FREQUENCY
    );

    let max_tx_signal_levels = HashMap::from([
        (CONTROL_FREQUENCY, max_tx_control_signal_level),
    ]);
    
    TRXModule::build(
        max_tx_signal_levels,
        HashMap::new()
    ).unwrap()
}
 
pub fn drone_rx_module(max_gps_rx_signal_level: SignalLevel) -> TRXModule {
    let max_rx_signal_levels = HashMap::from([
        (CONTROL_FREQUENCY, GREEN_SIGNAL_LEVEL),
        (GPS_L1_FREQUENCY, max_gps_rx_signal_level)
    ]);

    TRXModule::build(
        max_rx_signal_levels,
        HashMap::new()
    ).unwrap()
}

pub fn ewd_trx_system(
    trx_system_type: TRXSystemType,
    frequency: Megahertz,
    suppression_area_radius: Meter
) -> TRXSystem {
    TRXSystem::new( 
        trx_system_type,
        ewd_tx_module(frequency, suppression_area_radius), 
        TRXModule::default()
    )
}

pub fn ewd_tx_module(
    frequency: Megahertz,
    suppression_area_radius: Meter
) -> TRXModule {
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

    TRXModule::build(
        max_tx_signal_levels,
        tx_signal_levels
    ).unwrap()
}

pub fn default_gps(trx_system_type: TRXSystemType) -> GPS {
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

pub fn gps_tx_module() -> TRXModule {
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

pub fn device_power_system() -> PowerSystem {
    PowerSystem::build(DEVICE_MAX_POWER, DEVICE_MAX_POWER)
        .unwrap_or_else(|error| panic!("{}", error))
}

pub fn device_movement_system() -> MovementSystem {
    MovementSystem::build(MAX_DRONE_SPEED)
        .unwrap_or_else(|error| panic!("{}", error))
}


pub struct NetworkPosition {
    origin: Point3D,
    x_offset_range: Range<f32>,
    y_offset_range: Range<f32>,
    z_offset_range: Range<f32>,
}

impl NetworkPosition {
    #[must_use]
    pub fn new(
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



