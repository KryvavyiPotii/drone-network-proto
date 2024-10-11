use plotters::prelude::*;
use crate::constants::*;


#[derive(Clone)]
pub struct Coordinates3D { 
    pub x: f32, 
    pub y: f32, 
    pub z: f32, 
}

impl Coordinates3D {
    pub fn new(x: f32, y: f32, z: f32) -> Coordinates3D {
        Coordinates3D { x, y, z }
    }

    pub fn size(&self) -> f32 {
        (self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0))
            .sqrt()
    }

    pub fn truncate_vector_size(&mut self, truncation_size: f32) { 
        let vector_size: f32 = self.size();
        
        if truncation_size > 0.0 && vector_size > truncation_size {
            self.x = self.x * truncation_size / vector_size;
            self.y = self.y * truncation_size / vector_size;
            self.z = self.z * truncation_size / vector_size;
        }
    }

    pub fn distance(&self, destination: &Coordinates3D) -> f32 {
        let difference = Coordinates3D {
            x: destination.x - self.x,
            y: destination.y - self.y,
            z: destination.z - self.z 
        };

        difference.size() 
    }
}


pub enum SignalLevel {
    Green,
    Yellow,
    Red,
    Black,
}


#[derive(Clone)]
pub struct CommandCenter {
    pub position: Coordinates3D,
}

impl CommandCenter {
    pub fn new(position: &Coordinates3D) -> CommandCenter {
        CommandCenter { position: position.clone() }
    }
}


#[derive(Clone)]
pub struct Drone {
    pub max_speed: f32,
    pub global_position: Coordinates3D,
    pub position: Coordinates3D,
    pub velocity: Coordinates3D,
    pub acceleration: Coordinates3D,
    pub destination: Coordinates3D,
    // TODO infection_state
    pub command_center: CommandCenter,
    pub control_connection: bool,
    pub gps_connection: bool,
}

impl Drone {
    pub fn new(max_speed: f32,
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

    pub fn set_destination(&mut self, destination: Option<&Coordinates3D>) {
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
    
    pub fn update_position(&mut self, time_in_millis: u64) {
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

    pub fn reached_destination(&mut self) {
        let distance_to_destination = self.position.distance(&self.destination); 
        
        if distance_to_destination <= DESTINATION_RADIUS_IN_METRES {
            self.control_connection = false;
        }
    }
}


pub struct DroneNetwork {
    pub destination: Coordinates3D,
    pub drones: Vec<Drone>,
    // TODO graph and cellular automata implementation
}

impl DroneNetwork {
    pub fn new(destination: &Coordinates3D, drones: &Vec<Drone>) -> DroneNetwork {
        DroneNetwork {
            destination: destination.clone(), 
            drones: drones.to_vec(),
        }
    }
    pub fn set_destination(&mut self, destination: Option<&Coordinates3D>) {
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

    pub fn update_position(&mut self, time_in_millis: u64) {
        for drone in self.drones.iter_mut() {
            drone.update_position(time_in_millis);
        } 
    }

    pub fn disconnect_uncontrolled_drones(&mut self) {
        self.drones.retain(|drone| drone.control_connection);
    }
}

pub enum SuppressionFrequencyType {
    GPS,
    Control
}

pub enum SuppressionAreaType {
    Dome(f32)
    // TODO Rifle
}

pub struct RadarWarfareDevice {
    pub position: Coordinates3D,
    pub frequency: SuppressionFrequencyType,
    pub area: SuppressionAreaType,
}

impl RadarWarfareDevice {
    pub fn new(position: &Coordinates3D,
        frequency: SuppressionFrequencyType,
        area: SuppressionAreaType) -> RadarWarfareDevice {
        RadarWarfareDevice {
            position: position.clone(),
            frequency: frequency,
            area: area
        }
    }
    
    pub fn in_area(&self, drone: &Drone) -> bool {
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
   
    pub fn suppress(&self, drone: &mut Drone) -> bool {
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

    pub fn suppress_network(&self, drone_network: &mut DroneNetwork) -> bool {
        match self.frequency {
            SuppressionFrequencyType::GPS =>
                return self._suppress_network_gps(drone_network),
            SuppressionFrequencyType::Control =>
                return self._suppress_network_control(drone_network)
        }
    }

    pub fn _suppress_network_gps(&self, drone_network: &mut DroneNetwork) -> bool {
        let mut suppressed = false;

        for drone in drone_network.drones.iter_mut() {
            if self.suppress(drone) {
                suppressed = true;
            }
        }

        suppressed
    }

    pub fn _suppress_network_control(&self,
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


pub struct World {
    pub command_center: CommandCenter,
    pub drone_network: DroneNetwork,
    pub radar_warfare_devices: Vec<RadarWarfareDevice>,
    pub current_time_in_millis: u64,
    pub end_time_in_millis: u64,
}

impl World {
    pub fn new(command_center: CommandCenter,
        drone_network: DroneNetwork,
        radar_warfare_devices: Vec<RadarWarfareDevice>,
        current_time_in_millis: u64,
        end_time_in_millis: u64) -> World {
        World {
            command_center,
            drone_network,
            radar_warfare_devices,
            current_time_in_millis,
            end_time_in_millis,
        }
    }
    pub fn simulate(&mut self) {
        let area = BitMapBackend::gif(
            PLOT_FILE_PATH, 
            PLOT_RESOLUTION_WH_IN_PIXELS, 
            STEP_DURATION_IN_MILLIS as u32
        ).unwrap().into_drawing_area();

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


