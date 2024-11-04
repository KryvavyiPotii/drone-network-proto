use crate::constants::*;
use plotters::prelude::*;
use petgraph::graphmap::UnGraphMap;
use rand::{thread_rng, Rng};
use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::cmp::Ordering;
use std::sync::{Arc, atomic::AtomicBool};


pub trait Position {
    fn position(&self) -> Coordinates3D;
    fn set_position(&mut self, position: Coordinates3D);
    fn distance_to<U>(&self, other: &U) -> f32
    where
        U: Position,
    {
        self.position().point_vector_to(&other.position()).size()
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


#[derive(Clone, Copy, Debug)]
pub enum Goal {
    Reposition,
    Attack,
}

#[derive(Clone, Copy, Debug)]
pub enum MessageType {
    SetDestination(Option<Coordinates3D>, Goal),
    // TODO Infect,
}

#[derive(Clone, Copy, Debug)]
pub enum MessageState {
    Waiting,
    InProgress,
    Finished,
}

#[derive(Clone, Copy, Debug)]
pub struct Message {
    pub creation_time_in_millis: u64,
    pub message_type: MessageType,
    pub message_state: MessageState,
}

impl Message {
    pub fn new(time_in_millis: u64, message_type: MessageType) -> Self {
        Message { 
            creation_time_in_millis: time_in_millis,
            message_type,
            message_state: MessageState::Waiting,
        }
    }
}

#[derive(Clone)]
pub enum DroneNetworkType {
    ComplexNetwork(ComplexNetwork),
    // TODO CellularAutomata,
}

impl DroneNetworkType {
    pub fn connect_command_center(&mut self,
        command_center_info: CommandCenterInfoInDrone) {
        match self {
            Self::ComplexNetwork(complex_network) => {
                complex_network.command_center_info = command_center_info;
                
                let command_center_id = complex_network.command_center_info.id;

                complex_network.drones
                    .iter_mut()
                    .for_each(|drone| {
                        drone.command_center_id = command_center_id
                    });
            },
        }
    }

    pub fn add_message(&mut self, message: Message) {
        match self {
            Self::ComplexNetwork(complex_network) => { 
                // Message preprocessing.
                match message.message_type {
                    MessageType::SetDestination(destination, _) =>
                        if let Some(coordinates) = destination { 
                            complex_network.destination_in_metres = coordinates;
                        },
                }

                complex_network.update_delays();         
                complex_network.message_queue.push(
                    (message, complex_network.delays.clone())
                );
            },
        } 
    }

    pub fn update_state(&mut self) {
        match self {
            Self::ComplexNetwork(complex_network) => {
                complex_network.process_message_queue();
                
                complex_network.drones
                    .iter_mut()
                    .for_each(|drone| drone.update_state());

                complex_network.update_connections();
                
                complex_network.current_time_in_millis +=
                    STEP_DURATION_IN_MILLIS;
            },
        }
    }
    
    pub fn drone_iter(&self) -> std::slice::Iter<'_, Drone> {
        match self {
            Self::ComplexNetwork(complex_network) =>{
                complex_network.drones.iter()}
        } 
    }

    pub fn drone_iter_mut(&mut self) -> std::slice::IterMut<Drone> {
        match self {
            Self::ComplexNetwork(complex_network) => 
                complex_network.drones.iter_mut(),
        } 
    }
    
    pub fn remove_uncontrolled_drones(&mut self) {
        match self {
            Self::ComplexNetwork(complex_network) => {
                complex_network.drones.retain(|drone| drone.control_connection);
            
                complex_network.update_connections();
            },
        }
    }
}

#[derive(Clone)]
pub enum Topology {
    Star,
    Mesh,
}


fn calculate_signal_delay(distance_in_metres: f32, multiplier: f32) -> u64 {
    let delay = ((distance_in_metres / SIGNAL_SPEED_IN_METRES_PER_S / 1000.0)
        .round() * multiplier) as u64;

    let reminder = delay % STEP_DURATION_IN_MILLIS;
    
    delay - reminder
}


fn generate_drone_id() -> u32 {
    let mut rng = thread_rng();

    rng.gen_range(1..ID_RANGE)
}

fn equation_of_motion_1d(start_position: f32, velocity: f32,
    time_in_secs: f32) -> f32 {
    start_position + velocity * time_in_secs
}

fn equation_of_motion_3d(start_position: &Coordinates3D,
    velocity: &Coordinates3D, time_in_secs: f32) -> Coordinates3D {
    Coordinates3D::new(
        equation_of_motion_1d(start_position.x, velocity.x, time_in_secs),
        equation_of_motion_1d(start_position.y, velocity.y, time_in_secs),
        equation_of_motion_1d(start_position.z, velocity.z, time_in_secs),
    )
}


#[derive(Copy, Clone, Debug)]
pub struct Coordinates3D { 
    pub x: f32, 
    pub y: f32, 
    pub z: f32, 
}

impl Coordinates3D {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Coordinates3D { x, y, z }
    }

    pub fn coordinates(&self) -> (f32, f32, f32) {
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
            self.x = self.x * truncation_size / vector_size;
            self.y = self.y * truncation_size / vector_size;
            self.z = self.z * truncation_size / vector_size;
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
    fn position(&self) -> Self {
        *self
    }

    fn set_position(&mut self, position: Self) {
        *self = position;    
    }
}


#[derive(Clone)]
pub struct CommandCenterInfoInDrone {
    pub id: u32,
    pub position_in_metres: Coordinates3D,
}

impl CommandCenterInfoInDrone {
    pub fn new(id: u32, position: Coordinates3D) -> Self {
        CommandCenterInfoInDrone {
            id,
            position_in_metres: position,
        }
    }
}

#[derive(Clone)]
pub struct CommandCenter {
    id: u32,
    pub current_time_in_millis: u64,
    pub position_in_metres: Coordinates3D,
    pub area: SignalAreaType,
    pub drone_network: DroneNetworkType,
    scenario: Vec<Message>
}

impl CommandCenter {
    pub fn new(position: Coordinates3D, area: SignalAreaType,
        mut drone_network: DroneNetworkType, scenario: Vec<Message>) -> Self {
        let id = generate_drone_id();

        drone_network.connect_command_center(
            CommandCenterInfoInDrone::new(id, position.clone())
        );

        CommandCenter {
            id,
            current_time_in_millis: 0,
            position_in_metres: position,
            area,
            drone_network,
            scenario,
        }
    }
    
    pub fn id(&self) -> u32 {
        self.id
    }

    fn send_task(&mut self) {
        self.scenario
            .iter_mut()
            .for_each(|task|
                if task.creation_time_in_millis >= self.current_time_in_millis {
                    self.drone_network.add_message(task.clone());
                    task.message_state = MessageState::Finished;
                }
            );

        self.scenario.retain(|message|
            matches!(
                message.message_state,
                MessageState::Waiting | MessageState::InProgress
            )
        );
    }

    pub fn update_state(&mut self) {
        self.send_task();
        self.drone_network.update_state();
        self.current_time_in_millis += STEP_DURATION_IN_MILLIS;
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
    fn position(&self) -> Coordinates3D {
        self.position_in_metres
    }

    fn set_position(&mut self, position: Coordinates3D) {
        self.position_in_metres = position;
    }
}


#[derive(Clone, Copy, Debug)]
pub struct Drone {
    id: u32,
    pub max_speed_in_metresps: f32,
    pub global_position_in_metres: Coordinates3D,
    pub position_in_metres: Coordinates3D,
    pub velocity_in_metresps: Coordinates3D,
    pub destination_in_metres: Coordinates3D,
    // TODO infection_state
    command_center_id: u32,
    pub control_connection: bool,
    pub gps_connection: bool,
    pub goal: Goal,
}

impl Drone {
    pub fn new(position: Coordinates3D) -> Self {
        Drone {
            id: generate_drone_id(),
            max_speed_in_metresps: MAX_DRONE_SPEED_IN_METRES_PER_S,
            global_position_in_metres: position.clone(),
            position_in_metres: position.clone(),
            velocity_in_metresps: Coordinates3D::new(0.0, 0.0, 0.0),
            destination_in_metres: position,
            command_center_id: 0,
            control_connection: true,
            gps_connection: true,
            goal: Goal::Reposition,
        }
    }

    pub fn id(&self) -> u32 {
        self.id
    }

    pub fn command_center_id(&self) -> u32 {
        self.command_center_id
    }

    pub fn process_message(&mut self, message: &Message) {
        match message.message_type {
            MessageType::SetDestination(destination, goal) => {
                self.set_destination(destination);
                self.goal = goal;
            },
        };
    }
    
    fn set_destination(&mut self, destination: Option<Coordinates3D>) {
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
        
        self.global_position_in_metres = equation_of_motion_3d(
            &self.global_position_in_metres,
            &self.velocity_in_metresps,
            time_in_secs,
        );        

        if self.gps_connection {
            self.set_destination(None);
            self.position_in_metres = self.global_position_in_metres;
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
        
        if distance > DESTINATION_RADIUS_IN_METRES {
            return;
        }

        if let Goal::Attack = self.goal { 
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
    fn position(&self) -> Coordinates3D {
        self.position_in_metres
    }

    fn set_position(&mut self, point: Coordinates3D) {
        self.position_in_metres = point;
    }
}


#[derive(Clone)]
pub struct ComplexNetwork {
    pub current_time_in_millis: u64,
    command_center_info: CommandCenterInfoInDrone,
    pub destination_in_metres: Coordinates3D,
    drones: Vec<Drone>,
    topology: Topology,
    connections: UnGraphMap<u32, f32>,
    delays: HashMap<u32, u64>,
    delay_multiplier: f32,
    message_queue: Vec<(Message, HashMap<u32, u64>)>,
}

impl ComplexNetwork {
    pub fn new(mut drones: Vec<Drone>, delay_multiplier: f32) -> Self {
        let command_center_info = CommandCenterInfoInDrone::new(
            0,
            Coordinates3D::new(0.0, 0.0, 0.0)
        );
        
        drones
            .iter_mut()
            .for_each(|drone| drone.command_center_id = command_center_info.id);

        let delays: HashMap<_, _> = drones
            .iter()
            .map(|drone| (drone.id(), 0))
            .collect();

        ComplexNetwork {
            current_time_in_millis: 0,
            command_center_info,
            destination_in_metres: Coordinates3D::new(0.0, 0.0, 0.0),
            drones,
            topology: Topology::Star,
            connections: UnGraphMap::new(),
            delays,
            delay_multiplier,
            message_queue: Vec::new(),
        }
    }

    fn process_message_queue(&mut self) {
        if self.message_queue.is_empty() {
            return;
        }
        
        let current_time = self.current_time_in_millis;

        for (message, delays_snapshot) in self.message_queue.iter_mut() {
            message.message_state = MessageState::InProgress;
            let message_time = message.creation_time_in_millis;

            self.drones
                .iter_mut()
                .for_each(|drone| {
                    let delay = delays_snapshot.get(&drone.id)
                        .expect("Delay for drone is missing");

                    if current_time >= message_time + delay {
                        drone.process_message(message);
                    }
                });

            // If every drone processed the message, set state to finished.
            let longest_delay = delays_snapshot.values().max().unwrap();

            if current_time >= message_time + longest_delay {
                message.message_state = MessageState::Finished; 
            }
        }

        self.clean_message_queue();
    }

    fn clean_message_queue(&mut self) {
        self.message_queue.retain(|(message, _)|
            matches!(
                message.message_state,
                MessageState::Waiting | MessageState::InProgress
            )
        );
    }

    fn update_connections(&mut self) {
        self.connections.clear();

        for (i, drone1) in self.drones.iter().enumerate() {
            let node1 = self.connections.add_node(drone1.id);

            for drone2 in self.drones.iter().skip(i + 1) {
                let node2 = self.connections.add_node(drone2.id);
                let distance = drone1.distance_to(drone2);

                self.connections.add_edge(node1, node2, distance);
            }
        } 
    }

    fn update_delays(&mut self) {
        // No need to calculate anything if delays are turned off.
        if self.delay_multiplier == 0.0 {
            return;
        }

        let command_center_position = self.command_center_info
            .position_in_metres;
       
        match self.topology {
            Topology::Star => {
                self.drones
                    .iter() 
                    .for_each(|drone| {
                        let distance = drone.distance_to(
                            &command_center_position
                        );
                        let delay = calculate_signal_delay(
                            distance,
                            self.delay_multiplier
                        );

                        self.delays.insert(drone.id(), delay);
                    });
            },

            Topology::Mesh => {
                let closest_to_cc_drone = self.drones
                    .iter()
                    .min_by(|x, y| {
                        let distance_x = x.distance_to(
                            &command_center_position
                        );
                        let distance_y = y.distance_to(
                            &command_center_position
                        );
                        
                        distance_x.partial_cmp(&distance_y)
                            .expect("Failed to compare f32 values")
                    })
                    .unwrap();

                self.drones
                    .iter() 
                    .for_each(|drone| {
                        let distance = drone.distance_to(closest_to_cc_drone);
                        let delay = calculate_signal_delay(
                            distance,
                            self.delay_multiplier
                        );
                        
                        self.delays.insert(drone.id(), delay);
                    });
            },
        }
    }
   
    pub fn remove_uncontrolled_drones(&mut self) {
        self.drones.retain(|drone| drone.control_connection);
    
        self.update_connections();
    }
}


#[derive(Clone, Copy)]
pub struct RadarWarfareDevice {
    id: u32,
    pub position_in_metres: Coordinates3D,
    pub frequency: SignalFrequencyType,
    pub area: SignalAreaType,
}

impl RadarWarfareDevice {
    pub fn new(position: Coordinates3D, frequency: SignalFrequencyType,
        area: SignalAreaType) -> Self {
        RadarWarfareDevice { 
            id: generate_drone_id(),
            position_in_metres: position, 
            frequency, 
            area,
        }
    }

    pub fn id(&self) -> u32 {
        self.id
    }
    
    pub fn in_area(&self, drone: &Drone) -> bool {
        match self.area {
            SignalAreaType::Dome(radius) => {
                let distance = self.distance_to(
                    &drone.global_position_in_metres
                );
                
                distance <= radius
            }
        }
    }
   
    pub fn suppress_drone(&self, drone: &mut Drone) -> bool {
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
                self.suppress_network_gps(drone_network),
            SignalFrequencyType::Control =>
                self.suppress_network_control(drone_network),
        }
    }

    fn suppress_network_gps(&self, drone_network: &mut DroneNetworkType) -> bool {
        let mut suppressed = false;

        match drone_network {
            DroneNetworkType::ComplexNetwork(complex_network) => {
                // TODO remove extra if checks
                complex_network.drones
                    .iter_mut()
                    .for_each(|drone| if self.suppress_drone(drone) {
                        suppressed = true;
                    });
            },
        }

        suppressed
    }

    fn suppress_network_control(&self, drone_network: &mut DroneNetworkType)
        -> bool {
        let mut suppressed = false;

        match drone_network {
            DroneNetworkType::ComplexNetwork(complex_network) => {
                // TODO remove extra if checks
                complex_network.drones
                    .iter_mut()
                    .for_each(|drone| if self.suppress_drone(drone) {
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
    fn position(&self) -> Coordinates3D {
        self.position_in_metres
    }

    fn set_position(&mut self, point: Coordinates3D) {
        self.position_in_metres = point;
    }
}

pub struct World {
    pub current_time_in_millis: u64,
    pub end_time_in_millis: u64,
    pub command_centers: Vec<CommandCenter>,
    pub radar_warfare_devices: Vec<RadarWarfareDevice>,
}

impl World {
    pub fn new(current_time: u64, end_time: u64,
        command_centers: Vec<CommandCenter>,
        radar_warfare_devices: Vec<RadarWarfareDevice>) -> Self {
        World {
            current_time_in_millis: current_time,
            end_time_in_millis: end_time,
            command_centers,
            radar_warfare_devices,
        }
    }

    pub fn simulate(&mut self) {
        // Handle SIGINT to stop rendering.
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        ctrlc_async::set_handler(move || {
            r.store(false, std::sync::atomic::Ordering::SeqCst);
        }).expect("Error setting Ctrl-C handler");

        let area = BitMapBackend::gif(
            PLOT_FILE_PATH, 
            PLOT_RESOLUTION_WH_IN_PIXELS, 
            STEP_DURATION_IN_MILLIS as u32
        ).unwrap().into_drawing_area();

        let begin = self.current_time_in_millis;
        let end = self.end_time_in_millis;
        
        self.command_centers
            .iter_mut()
            .for_each(|command_center|
                command_center.current_time_in_millis = begin
            );

        for _ in (begin..end).step_by(STEP_DURATION_IN_MILLIS as usize) {
            if !running.load(std::sync::atomic::Ordering::SeqCst) { 
                break;
            }

            self.command_centers.iter_mut()
                .for_each(|command_center| {
                    command_center.update_state();

                    self.radar_warfare_devices
                        .iter()
                        .for_each(|rwd| {
                            rwd.suppress_network(
                                &mut command_center.drone_network
                            );
                        });
                });
                                
            let mut chart = ChartBuilder::on(&area)
                .margin(20)
                .caption("Drone network", ("sans-serif", PLOT_FONT_SIZE))
                .build_cartesian_3d(0.0..200.0, 0.0..200.0, 0.0..200.0)
                .unwrap();
            chart.configure_axes().draw().unwrap();
            
            // Draw destination points.
            let destinations: Vec<Coordinates3D> = self.command_centers
                .iter()
                .map(|command_center| 
                    match &command_center.drone_network {
                        DroneNetworkType::ComplexNetwork(complex_network) =>
                            complex_network.destination_in_metres,
                    }
                )
                .collect();

            chart.draw_series(
                destinations
                    .iter()
                    .map(|position_in_metres| {
                        let point = (
                            position_in_metres.x as f64,
                            position_in_metres.z as f64,
                            position_in_metres.y as f64,
                        );

                        Circle::new(
                            point, 
                            convert_metres_to_pixels(
                                DESTINATION_RADIUS_IN_METRES
                            ), 
                            &YELLOW)
                    }),
            ).unwrap();
            
            // Draw command centers.
            chart.draw_series(
                self.command_centers
                    .iter()
                    .map(|command_center| {
                        let point = (
                            command_center.position_in_metres.x as f64,
                            command_center.position_in_metres.z as f64,
                            command_center.position_in_metres.y as f64,
                        );
                        let radius = convert_metres_to_pixels(
                            COMMAND_CENTER_RADIUS_IN_METRES
                        );  
                        
                        Circle::new(point, radius, &GREEN)
                    }),
            ).unwrap();
           
            // Draw drones.
            let mut color_byte: u8 = 0;
            let color_modifier: u8 = 255 / self.command_centers.len() as u8;
            
            self.command_centers
                .iter()
                .for_each(|command_center| {
                    let color = plotters::style::RGBColor(
                        color_byte,
                        color_byte,
                        color_byte,
                    );

                    chart.draw_series(
                        command_center.drone_network
                            .drone_iter()
                            .map(|drone| {
                                let point = (
                                    drone.global_position_in_metres.x as f64,
                                    drone.global_position_in_metres.z as f64,
                                    drone.global_position_in_metres.y as f64,
                                );

                                Circle::new(point, 1, color)
                            })
                    ).unwrap();

                    color_byte += color_modifier; 
                });

            // Draw radar warfare devices.
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
                                convert_metres_to_pixels(radius_in_metres)
                        };

                        let area_color = match rwd.frequency {
                            SignalFrequencyType::GPS => &RED,
                            SignalFrequencyType::Control => &BLUE
                        };

                        Circle::new(point, rwd_coverage, area_color)
                    }),
            ).unwrap();

            area.present().unwrap();
            
            area.fill(&WHITE).unwrap();

            println!("Current time: {}", self.current_time_in_millis);
            
            self.current_time_in_millis += STEP_DURATION_IN_MILLIS;
        }

        println!(
            "[INFO] Simulation finished at {} millis",
            self.current_time_in_millis
        );
    }
}


