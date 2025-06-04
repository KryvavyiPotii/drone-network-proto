use std::collections::HashMap;

use thiserror::Error;

use crate::backend::connections::{ConnectionGraph, ShortestPathError};
use crate::backend::device::{Device, DeviceId, IdToDeviceMap, IdToLevelMap};
use crate::backend::mathphysics::Megahertz;
use crate::backend::signal::NO_SIGNAL_LEVEL;


#[derive(Error, Debug)]
pub enum SignalUpdateError {
    #[error("Shortest Path algorithm failed to calculate path.")]
    AlgorithmError(ShortestPathError),
    #[error("Missing a drone")]
    MissingDevice,
}

impl From<ShortestPathError> for SignalUpdateError {
    fn from(error: ShortestPathError) -> Self {
        Self::AlgorithmError(error)  
    }
}


/// # Errors
///
/// Will return Err if the first drone on found shortest path is not 
/// present in `IdToDeviceMap`.
pub fn try_find_best_signal_levels(
    command_device_id: DeviceId,
    device_map: &IdToDeviceMap,
    connections: &ConnectionGraph,
    frequency: Megahertz
) -> Result<IdToLevelMap, SignalUpdateError> {
    let Some(command_device) = device_map.get(&command_device_id) else {
        return Err(SignalUpdateError::MissingDevice);
    };
    let mut best_signal_levels = HashMap::from([
        (command_device_id, *command_device.tx_signal_level(frequency))
    ]);

    for device_id in device_map.ids() {
        let Ok((_, path)) = connections.find_shortest_path_from_to(
            command_device_id, 
            *device_id
        ) else {
            continue;
        };
    
        traverse_path_and_set_better_signal_level(
            &path, 
            device_map, 
            &mut best_signal_levels, 
            frequency
        );
    }

    Ok(best_signal_levels)
}

fn traverse_path_and_set_better_signal_level(
    path: &[DeviceId], 
    device_map: &IdToDeviceMap,
    best_signal_levels: &mut IdToLevelMap,
    frequency: Megahertz
) {
    // Skipping the last element to avoid reading out of bounds.
    for i in 0..(path.len() - 1) {
        let tx_id = path[i];
        let rx_id = path[i + 1];
        
        let Some(tx) = device_map.get(&tx_id) else { break };
        let Some(rx) = device_map.get(&rx_id) else { break };

        try_set_better_signal_level(
            tx,
            rx,
            best_signal_levels,
            frequency
        );
    }
}

fn try_set_better_signal_level(
    tx: &Device,
    rx: &Device,
    signal_levels: &mut IdToLevelMap,
    frequency: Megahertz
) {
    // TODO find a way to avoid cloning (without multiple mutable borrows)
    let mut tx = tx.clone();

    if let Some(tx_signal_level) = signal_levels.get(&tx.id()) {
        tx.set_tx_signal_level(*tx_signal_level, frequency);
    }

    let rx_signal_level = signal_levels
        .get(&rx.id())
        .unwrap_or(&NO_SIGNAL_LEVEL);
    let signal_level_at_rx = tx.propagated_signal_level_at(rx, frequency);

    if signal_level_at_rx > *rx_signal_level {
        signal_levels.insert(rx.id(), signal_level_at_rx);
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::backend::CONTROL_FREQUENCY;
    use crate::backend::device::{Device, DeviceBuilder};
    use crate::backend::device::systems::{
        PowerSystem, TRXModule, TRXSystem, TRXSystemType
    };
    use crate::backend::mathphysics::{Meter, Point3D, PowerUnit};
    use crate::backend::signal::{
        GREEN_SIGNAL_STRENGTH_VALUE, NO_SIGNAL_LEVEL, SignalArea, SignalLevel, 
    };
    
    use super::*;


    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const DEVICE_MAX_POWER: PowerUnit   = 10_000;
    const VERY_BIG_STRENGTH_VALUE: f32  = GREEN_SIGNAL_STRENGTH_VALUE * 1_000.0;


    fn device_power_system() -> PowerSystem {
        PowerSystem::build(DEVICE_MAX_POWER, DEVICE_MAX_POWER)
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn drone_tx_module() -> TRXModule {
        let frequency = CONTROL_FREQUENCY;
        
        let max_tx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let tx_signal_levels = HashMap::from([(
            frequency, 
            SignalLevel::from_area(
                SignalArea::build(DRONE_TX_CONTROL_RADIUS) 
                    .unwrap_or_else(|error| panic!("{}", error)),
                frequency
            )
        )]);

        TRXModule::build(max_tx_signal_levels, tx_signal_levels)
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn drone_rx_module() -> TRXModule {
        let frequency = CONTROL_FREQUENCY;
        
        let max_rx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let rx_signal_levels = HashMap::from([
            (frequency, NO_SIGNAL_LEVEL)
        ]);

        TRXModule::build(max_rx_signal_levels, rx_signal_levels)
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn drone_with_trx_system_set(position: Point3D) -> Device {
        DeviceBuilder::new()
            .set_real_position(position)
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    drone_tx_module(),
                    drone_rx_module() 
                )
            )
            .build()
    } 

    
    #[test]
    fn better_signal_with_cc() {
        let frequency = CONTROL_FREQUENCY;

        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    drone_tx_module(),
                    TRXModule::default() 
                )
            )
            .build();

        let mut best_signal_levels = HashMap::new();
        
        let drones = [
            drone_with_trx_system_set(Point3D::new(1.0, 0.0, 0.0)), // Green
            drone_with_trx_system_set(Point3D::new(1.5, 0.0, 0.0)), // Yellow
            drone_with_trx_system_set(Point3D::new(4.0, 0.0, 0.0)), // Red
            drone_with_trx_system_set(Point3D::new(10.0, 0.0, 0.0)) // Black
        ]; 

        for drone in &drones {
            try_set_better_signal_level(
                &command_center,
                drone, 
                &mut best_signal_levels, 
                frequency
            );
        }

        assert!(
            best_signal_levels
                .get(&drones[0].id())
                .unwrap_or(&NO_SIGNAL_LEVEL)
                .is_green()
        );
        assert!(
            best_signal_levels
                .get(&drones[1].id())
                .unwrap_or(&NO_SIGNAL_LEVEL)
                .is_yellow()
        );
        assert!(
            best_signal_levels
                .get(&drones[2].id())
                .unwrap_or(&NO_SIGNAL_LEVEL)
                .is_red()
        );
        assert!(
            best_signal_levels
                .get(&drones[3].id())
                .unwrap_or(&NO_SIGNAL_LEVEL)
                .is_black()
        );
    }
}
