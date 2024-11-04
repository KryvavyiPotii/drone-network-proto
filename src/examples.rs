use rand::prelude::*;
use crate::entities::*;
use crate::constants::*;


fn init_drone_vec(drone_count: u32) -> Vec<Drone> {
    let mut drones: Vec<Drone> = Vec::new();
    
    for _ in 1..=drone_count {
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

    drones
}

pub fn simulate_gps_only() {
    let drones = init_drone_vec(100); 
    
    let scenario: Vec<Message> = vec![
        Message::new(0, MessageType::SetDestination(
            Some(Coordinates3D::from(INITIAL_DRONE_DESTINATION)),
            Goal::Attack
        )),
    ];

    let drone_network = DroneNetworkType::ComplexNetwork(
        ComplexNetwork::new(drones, 1.0),
    );

    let command_center = CommandCenter::new(
        Coordinates3D::from(INITIAL_COMMAND_CENTER_POSITION),
        SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
        drone_network,
        scenario,
    );
    
    let radar_warfare_device_gps = RadarWarfareDevice::new(
        Coordinates3D::new(0.0, 5.0, 2.0),
        SignalFrequencyType::GPS,
        SignalAreaType::Dome(RWD_GPS_RADIUS_IN_METRES)
    );

    let mut world = World::new(
        0,
        MAX_TIME_IN_MILLIS,
        vec![command_center],
        vec![radar_warfare_device_gps],
    );

    world.simulate();
}

pub fn simulate_gps_and_control() {
    let drones = init_drone_vec(100); 

    let scenario: Vec<Message> = vec![
        Message::new(0, MessageType::SetDestination(
            Some(Coordinates3D::from(INITIAL_DRONE_DESTINATION)),
            Goal::Attack
        )),
    ];

    let drone_network = DroneNetworkType::ComplexNetwork(
        ComplexNetwork::new(drones, 1.0),
    );

    let command_center = CommandCenter::new(
        Coordinates3D::from(INITIAL_COMMAND_CENTER_POSITION),
        SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
        drone_network,
        scenario,
    );
    
    let radar_warfare_device_control = RadarWarfareDevice::new(
        Coordinates3D::new(-10.0, 2.0, 0.0),
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
        vec![command_center],
        radar_warfare_devices
    );

    world.simulate();
}

pub fn simulate_command_delay(display_delayless_network: bool) {
    let drones = init_drone_vec(100); 

    let scenario: Vec<Message> = vec![
        Message::new(0, MessageType::SetDestination(
            Some(Coordinates3D::from(INITIAL_DRONE_DESTINATION)),
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
            Some(Coordinates3D::from(INITIAL_DRONE_DESTINATION)),
            Goal::Attack
        )),
    ];

    let mut command_centers: Vec<CommandCenter> = Vec::new();

    if display_delayless_network {
        let delayless_drone_network = DroneNetworkType::ComplexNetwork(
            ComplexNetwork::new(drones.clone(), 0.0),
        );

        command_centers.push(
            CommandCenter::new(
                Coordinates3D::from(INITIAL_COMMAND_CENTER_POSITION),
                SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
                delayless_drone_network,
                scenario.clone(),
            ),
        );
    }

    let drone_network = DroneNetworkType::ComplexNetwork(
        ComplexNetwork::new(drones, 1.0),
    );
    
    command_centers.insert(
        0,
        CommandCenter::new(
            Coordinates3D::from(INITIAL_COMMAND_CENTER_POSITION),
            SignalAreaType::Dome(CC_CONTROL_RADIUS_IN_METRES),
            drone_network,
            scenario,
        ),
    );

    let mut world = World::new(
        0,
        MAX_TIME_IN_MILLIS,
        command_centers,
        Vec::new()
    );

    world.simulate();
}
