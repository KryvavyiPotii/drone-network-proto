use crate::constants::*;
use plotters::prelude::*;
use petgraph::graphmap::UnGraphMap;
use rand::{thread_rng, Rng};
use std::collections::{HashMap, VecDeque};
use std::hash::{Hash, Hasher};
use std::cmp::Ordering;


pub trait Position {
    fn get_position(&self) -> Coordinates3D;
    fn set_position(&mut self, position: Coordinates3D);
    fn distance_to<U: Position>(&self, other: &U) -> f32 {
        self.get_position().point_vector_to(&other.get_position()).size()
    }
}


#[derive(Clone, Copy)]
pub enum SignalFrequencyType {
    GPS,
    Control,
}

#[derive(Clone, Copy)]
pub enum SignalAreaType {
    Dome(f32),
    // TODO Rifle,
}

#[derive(Clone, Copy)]
pub enum SignalLevel {
    Green,
    Yellow,
    Red,
    Black,
}

#[derive(Clone, Copy)]
pub enum MessageType {
    SetDestination(u32, Option<Coordinates3D>),
}

#[derive(Clone)]
pub enum DroneNetworkType {
    ComplexNetwork(ComplexNetwork),
    // TODO CellularAutomata,
}


fn _generate_drone_id() -> u32 {
    let mut rng = thread_rng();

    rng.gen_range(0..ID_RANGE)
}

fn _equation_of_motion_1d(start_position: f32, velocity: f32,
    time_in_secs: f32) -> f32 {
    start_position + velocity * time_in_secs
}

fn _equation_of_motion_3d(start_position: &Coordinates3D,
    velocity: &Coordinates3D, time_in_secs: f32) -> Coordinates3D {
    Coordinates3D::new(
        _equation_of_motion_1d(start_position.x, velocity.x, time_in_secs),
        _equation_of_motion_1d(start_position.y, velocity.y, time_in_secs),
        _equation_of_motion_1d(start_position.z, velocity.z, time_in_secs),
    )
}


#[derive(Copy, Clone, Debug)]
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
}

impl From<(f32, f32, f32)> for Coordinates3D {
    fn from(value: (f32, f32, f32)) -> Self {
        Coordinates3D::new(value.0, value.1, value.2)
    }
}

impl Position for Coordinates3D {
    fn get_position(&self) -> Self {
        *self
    }

    fn set_position(&mut self, position: Self) {
        *self = position;    
    }
}


#[derive(Clone)]
pub struct CommandCenter {
    id: u32,
    current_time_in_millis: u32,
    position_in_metres: Coordinates3D,
    area: SignalAreaType,
    drone_network: DroneNetworkType, 
}

impl CommandCenter {
    pub fn new(position: Coordinates3D, area: SignalAreaType,
        mut drone_network: DroneNetworkType) -> Self {
        let id = _generate_drone_id();

        drone_network.connect_command_center(id);

        CommandCenter {
            id,
            current_time_in_millis: 0,
            position_in_metres: position,
            area,
            drone_network,
        }
    }
    
    pub fn get_id(&self) -> u32 {
        self.id
    }

    pub fn set_time(&mut self, time_in_millis: u32) {
        self.current_time_in_millis = time_in_millis;
    }

    pub fn set_destination(&mut self, destination: Option<Coordinates3D>) {
        self.drone_network.add_message(
            MessageType::SetDestination(
                self.current_time_in_millis,
                destination
            )
        );
    }

    pub fn update_state(&mut self) {
        self.drone_network.update_state();
    }
}

impl PartialEq for CommandCenter {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Hash for CommandCenter {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Position for CommandCenter {
    fn get_position(&self) -> Coordinates3D {
        self.position_in_metres
    }

    fn set_position(&mut self, position: Coordinates3D) {
        self.position_in_metres = position;
    }
}


#[derive(Clone, Copy)]
pub struct Drone {
    id: u32,
    max_speed_in_metresps: f32,
    _global_position_in_metres: Coordinates3D,
    position_in_metres: Coordinates3D,
    velocity_in_metresps: Coordinates3D,
    destination_in_metres: Coordinates3D,
    // TODO infection_state
    command_center_id: u32,
    control_connection: bool,
    gps_connection: bool,
}

impl Drone {
    pub fn new(position: Coordinates3D) -> Self {
        Drone {
            id: _generate_drone_id(),
            max_speed_in_metresps: MAX_DRONE_SPEED_IN_METRES_PER_S,
            _global_position_in_metres: position.clone(),
            position_in_metres: position,
            velocity_in_metresps: Coordinates3D::new(0.0, 0.0, 0.0),
            destination_in_metres: Coordinates3D::new(0.0, 0.0, 0.0),
            command_center_id: 0,
            control_connection: true,
            gps_connection: true,
        }
    }

    pub fn get_id(&self) -> u32 {
        self.id
    }

    pub fn connect_command_center(&mut self, command_center_id: u32) {
        self.command_center_id = command_center_id;
    }

    pub fn process_message(&mut self, message: &MessageType) {
        match message {
            MessageType::SetDestination(_, destination) => {
                self._set_destination(*destination);
            },
        };
    }
    
    fn _set_destination(&mut self, destination: Option<Coordinates3D>) {
        if let Some(coordinates) = destination {
            self.destination_in_metres = coordinates;
        }

        self.velocity_in_metresps = self.position_in_metres.point_vector_to(
            &self.destination_in_metres
        );
        
        self.velocity_in_metresps.truncate_vector_size(
            self.max_speed_in_metresps
        );
    }
    
    pub fn update_state(&mut self) {
        let time_in_secs = STEP_DURATION_IN_MILLIS as f32 / 1000.0;
        
        self._global_position_in_metres = _equation_of_motion_3d(
            &self._global_position_in_metres,
            &self.velocity_in_metresps,
            time_in_secs,
        );        

        if self.gps_connection {
            self._set_destination(None);
            self.position_in_metres = self._global_position_in_metres;
            self.reached_destination();
        }
        else {
            self.velocity_in_metresps.z = 0.0;
            self.velocity_in_metresps.truncate_vector_size(
                self.max_speed_in_metresps
            );
        }
        
        // Try to establish GPS connection.
        self.gps_connection = true;
    }

    pub fn reached_destination(&mut self) {
        let distance = self.distance_to(&self.destination_in_metres);
        
        if distance <= DESTINATION_RADIUS_IN_METRES {
            self.control_connection = false;
        }
    }
}

impl Ord for Drone {
    fn cmp(&self, other: &Self) -> Ordering {
        let self_distance = self.distance_to(&self.destination_in_metres);
        
        let other_distance = other.distance_to(&other.destination_in_metres);

        self_distance.partial_cmp(&other_distance)
            .expect("Failed to compare f32 values")
    }
}

impl PartialOrd for Drone {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for Drone {}

impl PartialEq for Drone {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Hash for Drone {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Position for Drone {
    fn get_position(&self) -> Coordinates3D {
        self.position_in_metres
    }

    fn set_position(&mut self, point: Coordinates3D) {
        self.position_in_metres = point;
    }
}


impl DroneNetworkType {
    pub fn connect_command_center(&mut self, command_center_id: u32) {
        match self {
            Self::ComplexNetwork(complex_network) => 
                complex_network.connect_command_center(command_center_id),
        }
    }
    
    pub fn add_message(&mut self, message: MessageType) {
        match self {
            Self::ComplexNetwork(complex_network) => 
                complex_network.add_message(message),
        } 
    }

    pub fn update_state(&mut self) {
        match self {
            Self::ComplexNetwork(complex_network) => 
                complex_network.update_state(),
        }
    }
    
    pub fn add_drone(&mut self, drone: Drone) {
        match self {
            Self::ComplexNetwork(complex_network) => 
                complex_network.add_drone(drone),
        }
    }

    pub fn remove_uncontrolled_drones(&mut self) {
        match self {
            Self::ComplexNetwork(complex_network) => 
                complex_network.remove_uncontrolled_drones(),
        }
    }
}


#[derive(Clone)]
pub struct ComplexNetwork {
    current_time_in_millis: u32,
    destination_in_metres: Coordinates3D,
    drones: Vec<Drone>,
    connections: UnGraphMap<u32, f32>,
    delays: HashMap<u32, u32>,
    message_queue: VecDeque<MessageType>,
}

impl ComplexNetwork {
    pub fn new(drones: Vec<Drone>) -> Self {
        ComplexNetwork {
            current_time_in_millis: 0,
            destination_in_metres: Coordinates3D::new(0.0, 0.0, 0.0),
            drones,
            connections: UnGraphMap::new(),
            delays: HashMap::new(),
            message_queue: VecDeque::new(),
        }
    }
    
    pub fn set_time(&mut self, time_in_millis: u32) {
        self.current_time_in_millis = time_in_millis;
    }

    pub fn set_destination(&mut self, destination: Coordinates3D) {
        self.destination_in_metres = destination;
    }

    pub fn connect_command_center(&mut self, command_center_id: u32) {
        self.drones.iter_mut()
            .for_each(|drone| drone.connect_command_center(command_center_id));
    }

    pub fn add_message(&mut self, message: MessageType) {
        match message {
            MessageType::SetDestination(_, destination) =>
                if let Some(coordinates) = destination { 
                    self.set_destination(coordinates);
                },
        }

        self.message_queue.push_back(message);
    }

    fn _process_message_queue(&mut self) {
        // TODO add delays
        for message in self.message_queue.iter() {
            self.drones.iter_mut()
                .for_each(|drone| drone.process_message(message));
        }

        self.message_queue.clear();
    }

    pub fn update_state(&mut self) {
        self._update_connections();
        self._process_message_queue();
        
        self.drones.iter_mut()
            .for_each(|drone| drone.update_state());
    }

    fn _update_connections(&mut self) {
        self.connections.clear();

        for (i, drone1) in self.drones.iter().enumerate() {
            let node1 = self.connections.add_node(drone1.id);

            for drone2 in self.drones.iter().skip(i + 1) {
                let node2 = self.connections.add_node(drone2.id);
                let distance = drone1.distance_to(drone2);

                self.connections.add_edge(node1, node2, distance);
                // TODO self.delays.insert(drone, );
            }
        }
    }

    pub fn add_drone(&mut self, drone: Drone) {
        self.drones.push(drone);
        
        self._update_connections(); 
    }

    pub fn remove_drone(&mut self, drone_id: u32) {
        self.disconnect_drone(drone_id);

        let index = self.drones.iter().position(|drone| drone.id == drone_id)
            .expect("Failed to remove drone");
        self.drones.remove(index);

        self._update_connections()
    }

    pub fn disconnect_drone(&mut self, drone_id: u32) {
        self.connections.remove_node(drone_id);
    }
    
    pub fn remove_uncontrolled_drones(&mut self) {
        self.drones.retain(|drone| drone.control_connection);
    
        self._update_connections();
    }
}


#[derive(Clone, Copy)]
pub struct RadarWarfareDevice {
    id: u32,
    position_in_metres: Coordinates3D,
    frequency: SignalFrequencyType,
    area: SignalAreaType,
}

impl RadarWarfareDevice {
    pub fn new(position: Coordinates3D, frequency: SignalFrequencyType,
        area: SignalAreaType) -> Self {
        RadarWarfareDevice { 
            id: _generate_drone_id(),
            position_in_metres: position, 
            frequency, 
            area,
        }
    }
    
    pub fn in_area(&self, drone: &Drone) -> bool {
        match self.area {
            SignalAreaType::Dome(radius) => {
                let distance = self.position_in_metres.distance_to(
                    &drone._global_position_in_metres
                );
                
                distance <= radius
            }
        }
    }
   
    pub fn suppress(&self, drone: &mut Drone) -> bool {
        if !self.in_area(drone) {
            return false;
        }
        
        match self.frequency {
            SignalFrequencyType::GPS => drone.gps_connection = false,
            SignalFrequencyType::Control => 
                drone.control_connection = false,
        }

        true
    }

    pub fn suppress_network(&self, drone_network: &mut DroneNetworkType) -> bool {
        match self.frequency {
            SignalFrequencyType::GPS =>
                self._suppress_network_gps(drone_network),
            SignalFrequencyType::Control =>
                self._suppress_network_control(drone_network),
        }
    }

    fn _suppress_network_gps(&self, drone_network: &mut DroneNetworkType) -> bool {
        let mut suppressed = false;

        match drone_network {
            DroneNetworkType::ComplexNetwork(complex_network) => {
                complex_network.drones.iter_mut()
                    // TODO remove extra if checks
                    .for_each(|drone| if self.suppress(drone) {
                        suppressed = true;
                    });
            },
        }

        suppressed
    }

    fn _suppress_network_control(&self, drone_network: &mut DroneNetworkType)
        -> bool {
        let mut suppressed = false;

        match drone_network {
            DroneNetworkType::ComplexNetwork(complex_network) => {
                complex_network.drones.iter_mut()
                    // TODO remove extra if checks
                    .for_each(|drone| if self.suppress(drone) {
                        suppressed = true;
                    });
                
                complex_network.remove_uncontrolled_drones();
            },
        }

        suppressed
    }
}

impl Hash for RadarWarfareDevice {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Position for RadarWarfareDevice {
    fn get_position(&self) -> Coordinates3D {
        self.position_in_metres
    }

    fn set_position(&mut self, point: Coordinates3D) {
        self.position_in_metres = point;
    }
}

pub struct World {
    current_time_in_millis: u64,
    end_time_in_millis: u64,
    command_center: CommandCenter,
    radar_warfare_devices: Vec<RadarWarfareDevice>,
}

impl World {
    pub fn new(current_time: u64, end_time: u64, command_center: CommandCenter,
        radar_warfare_devices: Vec<RadarWarfareDevice>) -> Self {
        World {
            current_time_in_millis: current_time,
            end_time_in_millis: end_time,
            command_center,
            radar_warfare_devices,
        }
    }

    pub fn simulate(&mut self) {
        let area = BitMapBackend::gif(
            PLOT_FILE_PATH, 
            PLOT_RESOLUTION_WH_IN_PIXELS, 
            STEP_DURATION_IN_MILLIS as u32
        ).unwrap().into_drawing_area();

        self.command_center.set_destination(
            Some(Coordinates3D::from(INITIAL_DRONE_DESTINATION))
        );

        while self.current_time_in_millis < self.end_time_in_millis {
            area.fill(&WHITE).unwrap();
    
            self.command_center.update_state();
                                
            self.radar_warfare_devices.iter()
                .for_each(|rwd| {
                    rwd.suppress_network(
                        &mut self.command_center.drone_network
                    );
                });
            
            let mut chart = ChartBuilder::on(&area)
                .margin(20)
                .caption("Drone network", ("sans-serif", PLOT_FONT_SIZE))
                .build_cartesian_3d(0.0..200.0, 0.0..200.0, 0.0..200.0)
                .unwrap();
            chart.configure_axes().draw().unwrap();

            let destination = match &self.command_center.drone_network {
                DroneNetworkType::ComplexNetwork(complex_network) =>
                    complex_network.destination_in_metres
            };

            let _ = chart.draw_series(
                vec![destination]
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
                match &self.command_center.drone_network { 
                    DroneNetworkType::ComplexNetwork(complex_network) => {
                        complex_network.drones
                            .iter()
                            .map(|drone| {
                                let point = (
                                    drone._global_position_in_metres.x as f64,
                                    drone._global_position_in_metres.z as f64,
                                    //drone._global_position_in_metres.y as f64,
                                    0.0,
                                );

                                Circle::new(point, 1, &BLACK)
                            })
                    },
                }
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
                            SignalAreaType::Dome(radius_in_metres) =>
                                _convert_metres_to_pixels(radius_in_metres)
                        };

                        let area_color = match rwd.frequency {
                            SignalFrequencyType::GPS => &RED,
                            SignalFrequencyType::Control => &BLUE
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


