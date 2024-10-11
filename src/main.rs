use entities::*;
use constants::*;
use rand::prelude::*;

pub mod entities;
pub mod constants;


fn main() {
    let command_center = CommandCenter::new(&INITIAL_COMMAND_CENTER_POSITION);
    
    let mut drones: Vec<Drone> = Vec::new();
   
    for _ in 1..=100 {
        let random_x_offset = rand::thread_rng().gen_range(-40.0..40.0);
        let random_y_offset = rand::thread_rng().gen_range(-40.0..40.0);
        let random_z_offset = rand::thread_rng().gen_range(-20.0..20.0);
        
        let position = Coordinates3D::new(
            INITIAL_DRONE_POSITION.x + random_x_offset,
            INITIAL_DRONE_POSITION.y + random_y_offset,
            INITIAL_DRONE_POSITION.z + random_z_offset
        );

        drones.push(
            Drone::new(
                MAX_SPEED_IN_METRES_PER_S, 
                &position, 
                &INITIAL_DRONE_DESTINATION, 
                &command_center
            )
        );
    }

    let drone_network = DroneNetwork::new(&INITIAL_DRONE_DESTINATION, &drones);
    
    let radar_warfare_device_control = RadarWarfareDevice::new(
        &Coordinates3D::new(-20.0, 2.0, 0.0),
        SuppressionFrequencyType::Control,
        SuppressionAreaType::Dome(RWD_RADIUS_IN_METRES / 2.0)
    );

    let radar_warfare_device_gps = RadarWarfareDevice::new(
        &Coordinates3D::new(0.0, 5.0, 2.0),
        SuppressionFrequencyType::GPS,
        SuppressionAreaType::Dome(RWD_RADIUS_IN_METRES)
    );

    let radar_warfare_devices: Vec<RadarWarfareDevice> = vec![
        radar_warfare_device_gps,
        radar_warfare_device_control
    ];
    
    let mut world = World {
        command_center: command_center.clone(),
        drone_network: drone_network,
        radar_warfare_devices: radar_warfare_devices,
        current_time_in_millis: 0,
        end_time_in_millis: MAX_TIME_IN_MILLIS,
    };

    world.simulate();
}
