use plotters::prelude::*;
use crate::constants::*;
use std::hash::Hash;
use petgraph::graphmap::UnGraphMap;

// TODO acceleration
fn _equation_of_motion_1d(start_position: f32, velocity: f32,
    time_in_secs: f32) -> f32 {
    start_position + velocity * time_in_secs
}

// TODO acceleration
fn _equation_of_motion_3d(start_position: &Coordinates3D,
    velocity: &Coordinates3D, time_in_secs: f32) -> Coordinates3D {
    Coordinates3D::new(
        _equation_of_motion_1d(start_position.x, velocity.x, time_in_secs),
        _equation_of_motion_1d(start_position.y, velocity.y, time_in_secs),
        _equation_of_motion_1d(start_position.z, velocity.z, time_in_secs),
    )
}

#[derive(Clone)]
pub struct Coordinates3D { 
    x: f32, 
    y: f32, 
    z: f32, 
}

impl Coordinates3D {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Coordinates3D { x, y, z }
    }

    pub fn get_coordinates(&self) -> (f32, f32, f32) {
        (self.x, self.y, self.z)
    }

    pub fn set_coordinates(&mut self, coordinates: (f32, f32, f32)) {
        self.x = coordinates.0;
        self.y = coordinates.1;
        self.z = coordinates.2;
    }

    pub fn size(&self) -> f32 {
        (self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0))
            .sqrt()
    }

    pub fn truncate_vector_size(&mut self, truncation_size: f32) { 
        let vector_size: f32 = self.size();
        
        if truncation_size > 0.0 && vector_size > truncation_size {
            self.set_coordinates((
                self.x * truncation_size / vector_size,
                self.y * truncation_size / vector_size,
                self.z * truncation_size / vector_size
            ));
        }
    }

    pub fn point_vector_to(&self, other: &Coordinates3D) -> Self {
        Coordinates3D::new(
            other.x - self.x,
            other.y - self.y,
            other.z - self.z
        )
    }

    pub fn distance(&self, other: &Coordinates3D) -> f32 {
        self.point_vector_to(other).size() 
    }
}

impl From<(f32, f32, f32)> for Coordinates3D {
    fn from(value: (f32, f32, f32)) -> Self {
        Coordinates3D::new(value.0, value.1, value.2)
    }
}


#[derive(Clone)]
pub struct CommandCenter {
    position_in_metres: Coordinates3D,
}

impl CommandCenter {
    pub fn new(position: Coordinates3D) -> Self {
        CommandCenter { position_in_metres: position }
    }
    
    pub fn get_position(&self) -> Coordinates3D {
        Coordinates3D::from(self.position_in_metres.get_coordinates())
    }

    pub fn set_position(&mut self, position: Coordinates3D) {
        self.position_in_metres.set_coordinates(position.get_coordinates());
    }
}


pub enum SignalLevel {
    Green,
    Yellow,
    Red,
    Black,
}


#[derive(Clone)]
pub struct Drone {
    max_speed: f32,
    _global_position_in_metres: Coordinates3D,
    position_in_metres: Coordinates3D,
    velocity_in_metresps: Coordinates3D,
    // TODO acceleration_in_m2ps: Coordinates3D,
    destination: Coordinates3D,
    // TODO infection_state
    command_center: Option<CommandCenter>,
    control_connection: bool,
    gps_connection: bool,
}

impl Drone {
    pub fn new(max_speed: f32, position: Coordinates3D,
        destination: Coordinates3D) -> Self {
        Drone {
            max_speed,
            _global_position_in_metres: position.clone(),
            position_in_metres: position,
            velocity_in_metresps: Coordinates3D::new(0.0, 0.0, 0.0),
            // TODO acceleration_in_m2ps: Coordinates3D::new(0.0, 0.0, 0.0),
            destination,
            command_center: None,
            control_connection: true,
            gps_connection: true,
        }
    }

    pub fn connect_command_center(&mut self, command_center: CommandCenter) {
        self.command_center = Some(command_center);
    }

    pub fn set_destination(&mut self, destination: Option<Coordinates3D>) {
        match destination {
            Some(coordinates) => self.destination.set_coordinates(
                (coordinates.x, coordinates.y, coordinates.z)
            ),
            None => (),
        }

        let destination = self.destination.get_coordinates();
        let current_position = self.position_in_metres.get_coordinates();

        self.velocity_in_metresps.set_coordinates((
            destination.0 - current_position.0,
            destination.1 - current_position.1,
            destination.2 - current_position.2
        ));
        
        self.velocity_in_metresps.truncate_vector_size(self.max_speed);
        // TODO change acceleration_in_m2ps
    }
    
    pub fn update_position(&mut self, time_in_millis: u64) {
        let time_in_secs = time_in_millis as f32 / 1000.0;
        

        self._global_position_in_metres = _equation_of_motion_3d(
            &self._global_position_in_metres,
            &self.velocity_in_metresps,
            time_in_secs
        );        

        if self.gps_connection {
            self.set_destination(None);

            self.position_in_metres.set_coordinates((
                self._global_position_in_metres.x,
                self._global_position_in_metres.y,
                self._global_position_in_metres.z
            ));

            self.reached_destination();
        }
        else {
            self.velocity_in_metresps.z = 0.0;
            // TODO self.acceleration_in_m2ps.z = 0.0;

            self.velocity_in_metresps.truncate_vector_size(self.max_speed);
            // TODO truncate acceleration_in_m2ps
        }

        self.gps_connection = true;
    }

    pub fn reached_destination(&mut self) {
        let distance_to_destination = self.position_in_metres
            .distance(&self.destination); 
        
        if distance_to_destination <= DESTINATION_RADIUS_IN_METRES {
            self.control_connection = false;
        }
    }
}


pub struct DroneSwarm {
    command_center: CommandCenter,
    destination: Coordinates3D,
    drones: Vec<Drone>,
    // TODO cellular automata implementation
}

impl DroneSwarm {
    pub fn new(command_center: CommandCenter, destination: Coordinates3D,
        mut drones: Vec<Drone>) -> Self {
        drones.iter_mut()
            .for_each(|drone| {
                drone.connect_command_center(command_center.clone())
            });
        
        DroneSwarm { command_center, destination, drones }
    }

    pub fn connect_command_center(&mut self,
        command_center: Option<CommandCenter>) {
        match command_center {
            Some(cc) => self.command_center = cc,
            None => (),
        };
        
        self.drones.iter_mut()
            .for_each(|drone| {
                drone.connect_command_center(self.command_center.clone())
            });
    }

    pub fn set_destination(&mut self, destination: Option<&Coordinates3D>) {
        match destination {
            Some(coordinates) => self.destination.set_coordinates(
                    (coordinates.x, coordinates.y, coordinates.z)
                ),
            None => (),
        }
        
        self.drones.iter_mut()
            .for_each(|drone| drone.set_destination(destination.cloned()));
    }

    pub fn update_position(&mut self, time_in_millis: u64) {
        self.drones.iter_mut()
            .for_each(|drone| drone.update_position(time_in_millis)); 
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
    position_in_metres: Coordinates3D,
    frequency: SuppressionFrequencyType,
    area: SuppressionAreaType,
}

impl RadarWarfareDevice {
    pub fn new(position: Coordinates3D, frequency: SuppressionFrequencyType,
        area: SuppressionAreaType) -> Self {
        RadarWarfareDevice { 
            position_in_metres: position, 
            frequency, 
            area,
        }
    }
    
    pub fn in_area(&self, drone: &Drone) -> bool {
        let drone_position = &drone._global_position_in_metres;

        match self.area {
            SuppressionAreaType::Dome(radius) => {
                let distance: f32 = self.position_in_metres
                    .point_vector_to(&drone_position)
                    .size();
                
                distance <= radius
            }
        }
    }
   
    pub fn suppress(&self, drone: &mut Drone) -> bool {
        if !self.in_area(drone) {
            return false;
        }
        
        match self.frequency {
            SuppressionFrequencyType::GPS => drone.gps_connection = false,
            SuppressionFrequencyType::Control => 
                drone.control_connection = false,
        }
        
        true
    }

    pub fn suppress_swarm(&self, drone_swarm: &mut DroneSwarm) -> bool {
        match self.frequency {
            SuppressionFrequencyType::GPS =>
                self._suppress_swarm_gps(drone_swarm),
            SuppressionFrequencyType::Control =>
                self._suppress_swarm_control(drone_swarm),
        }
    }

    fn _suppress_swarm_gps(&self, drone_swarm: &mut DroneSwarm) -> bool {
        let mut suppressed = false;

        drone_swarm.drones.iter_mut()
            .for_each(|drone| if self.suppress(drone) { suppressed = true; });

        suppressed
    }

    fn _suppress_swarm_control(&self, drone_swarm: &mut DroneSwarm) -> bool {
        let mut suppressed = false;

        drone_swarm.drones.iter_mut()
            .for_each(|drone| if self.suppress(drone) { suppressed = true; });

        drone_swarm.disconnect_uncontrolled_drones();
        
        suppressed
    }
}


pub struct World {
    command_center: CommandCenter,
    drone_swarm: DroneSwarm,
    radar_warfare_devices: Vec<RadarWarfareDevice>,
    current_time_in_millis: u64,
    end_time_in_millis: u64,
}

impl World {
    pub fn new(command_center: CommandCenter, drone_swarm: DroneSwarm,
        radar_warfare_devices: Vec<RadarWarfareDevice>,
        current_time_in_millis: u64, end_time_in_millis: u64) -> Self {
        World {
            command_center,
            drone_swarm,
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

        self.drone_swarm.set_destination(None);

        while self.current_time_in_millis < self.end_time_in_millis {
            area.fill(&WHITE).unwrap();
    
            self.drone_swarm.update_position(STEP_DURATION_IN_MILLIS);
                                
            self.radar_warfare_devices.iter()
                .for_each(|rwd| { rwd.suppress_swarm(&mut self.drone_swarm); });
            
            let mut chart = ChartBuilder::on(&area)
                .margin(20)
                .caption("Drone network", ("sans-serif", PLOT_FONT_SIZE))
                .build_cartesian_3d(0.0..200.0, 0.0..200.0, 0.0..200.0)
                .unwrap();
            chart.configure_axes().draw().unwrap();

           let _ = chart.draw_series(
                vec![self.drone_swarm.destination.clone()]
                    .iter()
                    .map(|position_in_metres| {
                        let point = (
                            position_in_metres.x as f64,
                            position_in_metres.z as f64,
                            position_in_metres.y as f64,
                        );

                        Circle::new(
                            point, 
                            _convert_metres_to_pixels(
                                DESTINATION_RADIUS_IN_METRES
                            ), 
                            &YELLOW)
                    }),
            );
            
            let _ = chart.draw_series(
                vec![self.command_center.position_in_metres.clone()]
                    .iter()
                    .map(|position_in_metres| {
                        let point = (
                            position_in_metres.x as f64,
                            position_in_metres.z as f64,
                            position_in_metres.y as f64,
                        );

                        Circle::new(
                            point, 
                            _convert_metres_to_pixels(
                                DESTINATION_RADIUS_IN_METRES
                            ),  
                            &GREEN)
                    }),
            );

            chart.draw_series(
                self.drone_swarm.drones
                    .iter()
                    .map(|drone| {
                        let point = (
                            drone._global_position_in_metres.x as f64,
                            drone._global_position_in_metres.z as f64,
                            drone._global_position_in_metres.y as f64,
                        );

                        Circle::new(point, 1, &BLACK)
                    }),
            ).unwrap();

            chart.draw_series(
                self.radar_warfare_devices
                    .iter()
                    .map(|rwd| {
                        let point = (
                            rwd.position_in_metres.x as f64,
                            rwd.position_in_metres.z as f64,
                            rwd.position_in_metres.y as f64,
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


