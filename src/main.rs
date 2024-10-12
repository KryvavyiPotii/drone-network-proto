use entities::*;
use constants::*;
use rand::prelude::*;

pub mod entities;
pub mod constants;


fn main() {
    let command_center = CommandCenter::new(
        Coordinates3D::from(INITIAL_COMMAND_CENTER_POSITION)
    );
    
    let mut drones: Vec<Drone> = Vec::new();
   
    for _ in 1..=100 {
        let random_x_offset = rand::thread_rng().gen_range(-40.0..40.0);
        let random_y_offset = rand::thread_rng().gen_range(-40.0..40.0);
        let random_z_offset = rand::thread_rng().gen_range(-20.0..20.0);
        
        let position = Coordinates3D::new(
            INITIAL_DRONE_POSITION.0 + random_x_offset,
            INITIAL_DRONE_POSITION.1 + random_y_offset,
            INITIAL_DRONE_POSITION.2 + random_z_offset
        );

        drones.push(
            Drone::new(
                MAX_SPEED_IN_METRES_PER_S, 
                position, 
                Coordinates3D::from(INITIAL_DRONE_DESTINATION), 
                command_center.clone()
            )
        );
    }

    let drone_network = DroneNetwork::new(
        Coordinates3D::from(INITIAL_DRONE_DESTINATION), 
        drones
    );
    
    let radar_warfare_device_control = RadarWarfareDevice::new(
        Coordinates3D::new(-20.0, 2.0, 0.0),
        SuppressionFrequencyType::Control,
        SuppressionAreaType::Dome(RWD_CONTROL_RADIUS_IN_METRES)
    );

    let radar_warfare_device_gps = RadarWarfareDevice::new(
        Coordinates3D::new(0.0, 5.0, 2.0),
        SuppressionFrequencyType::GPS,
        SuppressionAreaType::Dome(RWD_GPS_RADIUS_IN_METRES)
    );

    let radar_warfare_devices: Vec<RadarWarfareDevice> = vec![
        radar_warfare_device_gps,
        radar_warfare_device_control
    ];
    
    let mut world = World::new(
        command_center,
        drone_network,
        radar_warfare_devices,
        0,
        MAX_TIME_IN_MILLIS
    );

    world.simulate();
}
