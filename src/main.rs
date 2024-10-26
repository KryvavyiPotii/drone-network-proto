use entities::*;
use constants::*;
use rand::prelude::*;

pub mod entities;
pub mod constants;


fn main() {
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

        drones.push(Drone::new(position));
    }

    let drone_network = DroneNetworkType::ComplexNetwork(
        ComplexNetwork::new(drones)
    );

    let command_center = CommandCenter::new(
        Coordinates3D::from(INITIAL_COMMAND_CENTER_POSITION),
        SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
        drone_network
    );
    
    let radar_warfare_device_control = RadarWarfareDevice::new(
        Coordinates3D::new(-20.0, 2.0, 50.0),
        SignalFrequencyType::Control,
        SignalAreaType::Dome(RWD_CONTROL_RADIUS_IN_METRES)
    );

    let radar_warfare_device_gps = RadarWarfareDevice::new(
        Coordinates3D::new(0.0, 5.0, 2.0),
        SignalFrequencyType::GPS,
        SignalAreaType::Dome(RWD_GPS_RADIUS_IN_METRES)
    );

    let radar_warfare_devices: Vec<RadarWarfareDevice> = vec![
        radar_warfare_device_gps,
        radar_warfare_device_control
    ];
   
    let mut world = World::new(
        0,
        MAX_TIME_IN_MILLIS,
        command_center,
        radar_warfare_devices
    );

    world.simulate();
}
