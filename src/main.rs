//use std::{thread, time};
use plotters::prelude::*;
use rand::prelude::*;

//const MAX_HEIGHT_IN_METRES: f32          = 50.0;
const MAX_SPEED_IN_METRES_PER_S: f32     = 25.0;
const MAX_TIME_IN_MILLIS: u64            = 15000;
const STEP_DURATION_IN_MILLIS: u64       = 100;
const RWD_RADIUS_IN_METRES: f32          = 50.0;
const DESTINATION_RADIUS_IN_METRES: f32  = 5.0;
const INITIAL_DRONE_POSITION: Coordinates3D = Coordinates3D {
    x: 150.3,
    y: 90.6,
    z: 25.5, 
};
const INITIAL_DRONE_DESTINATION: Coordinates3D = Coordinates3D {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};
const INITIAL_COMMAND_CENTER_POSITION: Coordinates3D = Coordinates3D {
    x: 200.0,
    y: 100.0,
    z: 0.0,
};
const PLOT_FILE_PATH: &str = "simulation.gif";
const PLOT_RESOLUTION_WH_IN_PIXELS: (u32, u32) = (800, 700); 
const PLOT_FONT_SIZE: u32 = PLOT_RESOLUTION_WH_IN_PIXELS.0 / 15;
const METRES_TO_PIXELS_SCALE: f32 = PLOT_RESOLUTION_WH_IN_PIXELS.1 as f32 / 400.0;

fn _convert_metres_to_pixels(value_in_metres: f32) -> u32 {
    (value_in_metres * METRES_TO_PIXELS_SCALE) as u32 
}


#[derive(Clone)]
struct Coordinates3D { 
    x: f32, 
    y: f32, 
    z: f32, 
}

impl Coordinates3D {
    fn size(&self) -> f32 {
        (self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0))
            .sqrt()
    }

    fn truncate_vector_size(&mut self, truncation_size: f32) { 
        let vector_size: f32 = self.size();
        
        if truncation_size > 0.0 && vector_size > truncation_size {
            self.x = self.x * truncation_size / vector_size;
            self.y = self.y * truncation_size / vector_size;
            self.z = self.z * truncation_size / vector_size;
        }
    }

    fn distance(&self, destination: &Coordinates3D) -> f32 {
        let difference = Coordinates3D {
            x: destination.x - self.x,
            y: destination.y - self.y,
            z: destination.z - self.z 
        };

        difference.size() 
    }
}


enum SignalLevel {
    Green,
    Yellow,
    Red,
    Black,
}


#[derive(Clone)]
struct CommandCenter {
    position: Coordinates3D,
}

impl CommandCenter {
    fn new(position: &Coordinates3D) -> CommandCenter {
        CommandCenter { position: position.clone() }
    }
}


#[derive(Clone)]
struct Drone {
    max_speed: f32,
    global_position: Coordinates3D,
    position: Coordinates3D,
    velocity: Coordinates3D,
    acceleration: Coordinates3D,
    destination: Coordinates3D,
    // TODO infection_state
    command_center: CommandCenter,
    control_connection: bool,
    gps_connection: bool,
}

impl Drone {
    fn new(max_speed: f32,
        position: &Coordinates3D,
        destination: &Coordinates3D,
        command_center: &CommandCenter) -> Drone {
        Drone {
            max_speed: max_speed,
            global_position: position.clone(),
            position: position.clone(),
            velocity: Coordinates3D { x: 0.0, y: 0.0, z: 0.0 },
            acceleration: Coordinates3D { x: 0.0, y: 0.0, z: 0.0 },
            destination: destination.clone(),
            command_center: command_center.clone(),
            control_connection: true,
            gps_connection: true,
        }
    }

    fn set_destination(&mut self, destination: Option<&Coordinates3D>) {
        match destination {
            Some(coordinates) => {
                self.destination.x = coordinates.x;
                self.destination.y = coordinates.y;
                self.destination.z = coordinates.z;
            },
            None => (),
        }

        self.velocity.x = self.destination.x - self.position.x;
        self.velocity.y = self.destination.y - self.position.y;
        self.velocity.z = self.destination.z - self.position.z;
        
        self.velocity.truncate_vector_size(self.max_speed);
        // TODO change acceleration
    }
    
    fn update_position(&mut self, time_in_millis: u64) {
        let time_in_secs = time_in_millis as f32 / 1000.0;

        self.global_position.x += self.velocity.x * time_in_secs +
            self.acceleration.x * time_in_secs.powf(2.0) / 2.0;
        self.global_position.y += self.velocity.y * time_in_secs +
            self.acceleration.y * time_in_secs.powf(2.0) / 2.0;
        self.global_position.z += self.velocity.z * time_in_secs +
            self.acceleration.z * time_in_secs.powf(2.0) / 2.0;
        
        if self.gps_connection {
            self.set_destination(None);

            self.position.x = self.global_position.x;
            self.position.y = self.global_position.y;
            self.position.z = self.global_position.z;
            
            self.reached_destination();
        }
        else {
            self.velocity.z = 0.0;
            self.acceleration.z = 0.0;

            self.velocity.truncate_vector_size(self.max_speed);
            // TODO truncate acceleration
        }

        self.gps_connection = true;
    }

    fn reached_destination(&mut self) {
        let distance_to_destination = self.position.distance(&self.destination); 
        
        if distance_to_destination <= DESTINATION_RADIUS_IN_METRES {
            self.control_connection = false;
        }
    }
}


struct DroneNetwork {
    destination: Coordinates3D,
    drones: Vec<Drone>,
    // TODO graph and cellular automata implementation
}

impl DroneNetwork {
    fn new(destination: &Coordinates3D, drones: &Vec<Drone>) -> DroneNetwork {
        DroneNetwork {
            destination: destination.clone(), 
            drones: drones.to_vec(),
        }
    }
    fn set_destination(&mut self, destination: Option<&Coordinates3D>) {
        match destination {
            Some(coordinates) => {
                self.destination.x = coordinates.x;
                self.destination.y = coordinates.y;
                self.destination.z = coordinates.z;
            },
            None => (),
        }
        
        for drone in self.drones.iter_mut() {
            drone.set_destination(destination);
        }
    }

    fn update_position(&mut self, time_in_millis: u64) {
        for drone in self.drones.iter_mut() {
            drone.update_position(time_in_millis);
        } 
    }

    fn disconnect_uncontrolled_drones(&mut self) {
        self.drones.retain(|drone| drone.control_connection);
    }
}

enum SuppressionFrequencyType {
    GPS,
    Control
}

enum SuppressionAreaType {
    Dome(f32)
    // TODO Rifle
}

struct RadarWarfareDevice {
    position: Coordinates3D,
    frequency: SuppressionFrequencyType,
    area: SuppressionAreaType,
}

impl RadarWarfareDevice {
    fn new(position: &Coordinates3D,
        frequency: SuppressionFrequencyType,
        area: SuppressionAreaType) -> RadarWarfareDevice {
        RadarWarfareDevice {
            position: position.clone(),
            frequency: frequency,
            area: area
        }
    }
    
    fn in_area(&self, drone: &Drone) -> bool {
        let drone_position = &drone.global_position;

        match self.area {
            SuppressionAreaType::Dome(radius) => {
                let distance: f32 = 
                    (drone_position.x - self.position.x).powf(2.0) +
                    (drone_position.y - self.position.y).powf(2.0) +
                    (drone_position.z - self.position.z).powf(2.0);
                
                if distance <= radius.powf(2.0) { 
                    true
                }
                else {
                    false
                }
            }
        }
    }
   
    fn suppress(&self, drone: &mut Drone) -> bool {
        if self.in_area(drone) {
            match self.frequency {
                SuppressionFrequencyType::GPS => {
                    drone.gps_connection = false;
                },
                SuppressionFrequencyType::Control => {
                    drone.control_connection = false;
                }
            }
            true
        }
        else {
            false
        }
    }

    fn suppress_network(&self, drone_network: &mut DroneNetwork) -> bool {
        match self.frequency {
            SuppressionFrequencyType::GPS =>
                return self._suppress_network_gps(drone_network),
            SuppressionFrequencyType::Control =>
                return self._suppress_network_control(drone_network)
        }
    }

    fn _suppress_network_gps(&self, drone_network: &mut DroneNetwork) -> bool {
        let mut suppressed = false;

        for drone in drone_network.drones.iter_mut() {
            if self.suppress(drone) {
                suppressed = true;
            }
        }

        suppressed
    }

    fn _suppress_network_control(&self,
        drone_network: &mut DroneNetwork) -> bool {
        let mut suppressed = false;

        for drone in drone_network.drones.iter_mut() {
            if self.suppress(drone) {
                suppressed = true;
            }
        }

        drone_network.disconnect_uncontrolled_drones();
        
        suppressed
    }
}


struct World {
    command_center: CommandCenter,
    drone_network: DroneNetwork,
    radar_warfare_devices: Vec<RadarWarfareDevice>,
    current_time_in_millis: u64,
    end_time_in_millis: u64,
}

impl World {
    fn simulate(&mut self) {
        let area = BitMapBackend::gif(
            PLOT_FILE_PATH, 
            PLOT_RESOLUTION_WH_IN_PIXELS, 
            STEP_DURATION_IN_MILLIS as u32
        ).unwrap().into_drawing_area();
        
        //let step_duration = time::Duration::from_millis(STEP_DURATION_IN_MILLIS);

        self.drone_network.set_destination(None);

        while self.current_time_in_millis < self.end_time_in_millis {
            area.fill(&WHITE).unwrap();
    
            self.drone_network.update_position(STEP_DURATION_IN_MILLIS);
                                
            for rwd in self.radar_warfare_devices.iter() {
                rwd.suppress_network(&mut self.drone_network);
            }
            
            let mut chart = ChartBuilder::on(&area)
                .margin(20)
                .caption("Drone network", ("sans-serif", PLOT_FONT_SIZE))
                .build_cartesian_3d(0.0..200.0, 0.0..200.0, 0.0..200.0)
                .unwrap();
            chart.configure_axes().draw().unwrap();

           let _ = chart.draw_series(
                vec![self.drone_network.destination.clone()]
                    .iter()
                    .map(|position| {
                        let point = (
                            position.x as f64,
                            position.z as f64,
                            position.y as f64,
                        );

                        Circle::new(
                            point, 
                            _convert_metres_to_pixels(DESTINATION_RADIUS_IN_METRES), 
                            &YELLOW)
                    }),
            );
            
            let _ = chart.draw_series(
                vec![self.command_center.position.clone()]
                    .iter()
                    .map(|position| {
                        let point = (
                            position.x as f64,
                            position.z as f64,
                            position.y as f64,
                        );

                        Circle::new(
                            point, 
                            _convert_metres_to_pixels(DESTINATION_RADIUS_IN_METRES),  
                            &GREEN)
                    }),
            );

            chart.draw_series(
                self.drone_network.drones
                    .iter()
                    .map(|drone| {
                        let point = (
                            drone.global_position.x as f64,
                            drone.global_position.z as f64,
                            drone.global_position.y as f64,
                        );

                        Circle::new(point, 1, &BLACK)
                    }),
            ).unwrap();

            chart.draw_series(
                self.radar_warfare_devices
                    .iter()
                    .map(|rwd| {
                        let point = (
                            rwd.position.x as f64,
                            rwd.position.z as f64,
                            rwd.position.y as f64,
                        );

                        let rwd_coverage = match rwd.area {
                            SuppressionAreaType::Dome(radius_in_metres) =>
                                _convert_metres_to_pixels(radius_in_metres)
                        };

                        let area_color = match rwd.frequency {
                            SuppressionFrequencyType::GPS => &RED,
                            SuppressionFrequencyType::Control => &BLUE
                        };

                        Circle::new(point, rwd_coverage, area_color)
                    }),
            ).unwrap();

            area.present().unwrap();

            println!("Current time: {}", self.current_time_in_millis);
            self.current_time_in_millis += STEP_DURATION_IN_MILLIS;
        }
    }

}

fn main() {
    let command_center = CommandCenter::new(&INITIAL_COMMAND_CENTER_POSITION);
    
    let mut drones: Vec<Drone> = Vec::new();
   
    for _ in 1..=100 {
        let random_x_offset = rand::thread_rng().gen_range(-40.0..40.0);
        let random_y_offset = rand::thread_rng().gen_range(-40.0..40.0);
        let random_z_offset = rand::thread_rng().gen_range(-20.0..20.0);
        
        let position = Coordinates3D {
            x: INITIAL_DRONE_POSITION.x + random_x_offset,
            y: INITIAL_DRONE_POSITION.y + random_y_offset,
            z: INITIAL_DRONE_POSITION.z + random_z_offset,
        };

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
        &Coordinates3D { x: -20.0, y: 2.0, z: 0.0, },
        SuppressionFrequencyType::Control,
        SuppressionAreaType::Dome(RWD_RADIUS_IN_METRES / 2.0)
    );

    let radar_warfare_device_gps = RadarWarfareDevice::new(
        &Coordinates3D { x: 0.0, y: 5.0, z: 2.0, },
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
