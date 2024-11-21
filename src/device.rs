use std::collections::HashMap;
use std::cmp::Ordering;
use std::hash::{Hash, Hasher};

use rand::{thread_rng, Rng};

use crate::communication::{signal::*, message::*};
use crate::device::network::*;
use crate::math_physics::*;


pub mod network;


pub const CC_CONTROL_RADIUS_IN_METRES: f32  = 300.0;
pub const RWD_CONTROL_RADIUS_IN_METRES: f32 = 25.0;
pub const RWD_GPS_RADIUS_IN_METRES: f32     = 50.0;
pub const DESTINATION_RADIUS_IN_METRES: f32 = 5.0;

pub const STEP_DURATION_IN_MILLIS: u64 = 50;

const MAX_DRONE_SPEED_IN_METRES_PER_S: f32      = 25.0;
const MAX_DRONE_BROADCAST_RADIUS_IN_METRES: f32 = 10.0;

// TODO make it relative to power of the transmitter, characteristics of the
// medium through which the signal propagates, the sensitivity of the receiver
// etc.
const SIGNAL_STRENGTH_CONSTANT: f32    = 1000.0;
const SIGNAL_RED_ZONE_COEFFICIENT: f32 = 0.1;

const ID_RANGE: u32   = 100_000;
const UNKNOWN_ID: u32 = 0;


pub trait Transmitter: Position {
    fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel>;
    fn tx_signal_levels_mut(&mut self) -> &mut HashMap<SignalType, SignalLevel>;
    fn area(&self) -> &SignalAreaType;

    fn tx_signal_level(
        &self,
        signal_type: &SignalType
    ) -> Option<&SignalLevel> {
        self.tx_signal_levels().get(signal_type)
    }

    fn tx_signal_level_mut(
        &mut self,
        signal_type: &SignalType
    ) -> Option<&mut SignalLevel> {
        self.tx_signal_levels_mut().get_mut(signal_type)
    }
    
    fn calculate_signal_strength<U>(&self, receiver: &U) -> f32
    where
        U: Receiver
    {
        SIGNAL_STRENGTH_CONSTANT / self.distance_to(receiver).powi(2)
    }
   
    // Returns Some(distance) to object if it is in the transmitter's area.
    // Otherwise, returns None.
    fn connection_distance<U>(&self, other: &U) -> Option<f32>
    where
        U: Position
    {
        let distance = self.distance_to(other);

        match self.area() {
            SignalAreaType::Dome(radius) => { 
                if distance <= *radius {
                    return Some(distance);
                }
            },
        }

        None
    }

    fn signal_to<U>(&self, receiver: &U) -> (f32, SignalLevel)
    where
        U: Receiver
    {
        let distance_in_metres = self.distance_to(receiver);

        let signal_level = match self.area() {
            SignalAreaType::Dome(radius) => {
                let red_zone = SIGNAL_RED_ZONE_COEFFICIENT;

                if distance_in_metres <= *radius / 2.0 {
                    SignalLevel::Green
                }
                else if distance_in_metres <= *radius * (1.0 - red_zone) {
                    SignalLevel::Yellow
                }
                else if distance_in_metres <= *radius {
                    SignalLevel::Red
                }
                else {
                    SignalLevel::Black
                }
            },
        };

        (distance_in_metres, signal_level)
    }

    fn signal_to_mut<U>(
        &self,
        receiver: &mut U,
        signal_type: &SignalType
    ) -> f32
    where
        U: Receiver
    {
        let (distance, signal_level) = self.signal_to(receiver);

        match receiver.rx_signal_level_mut(signal_type) {
            Some(rx_signal_level) => *rx_signal_level = signal_level,
            None => (),
        }

        distance
    }

    fn propagated_signal_to<U>(
        &self,
        receiver: &U,
        signal_type: &SignalType
    ) -> (f32, SignalLevel)
    where
        U: Receiver
    {
        let mut signal_level = SignalLevel::Black;
        let (distance, rx_signal_level) = self.signal_to(receiver);
       
        match self.tx_signal_level(signal_type) {
            Some(tx_signal_level) => 
                signal_level = SignalLevel::propagation(
                    &tx_signal_level,
                    &rx_signal_level
                ),
            None => (),
        }

        (distance, signal_level)
    }
    
    fn propagated_signal_to_mut<U>(
        &self,
        receiver: &mut U,
        signal_type: &SignalType
    ) -> f32
    where
        U: Receiver
    {
        let (distance, signal_level) = self.propagated_signal_to(
            receiver,
            signal_type
        );

        match receiver.rx_signal_level_mut(signal_type) {
            Some(rx_signal_level) => *rx_signal_level = signal_level,
            None => (),
        }

        distance
    }
}

pub trait Receiver: Position {
    fn rx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel>;
    fn rx_signal_levels_mut(&mut self) -> &mut HashMap<SignalType, SignalLevel>;

    fn rx_signal_level(
        &self,
        signal_type: &SignalType
    ) -> Option<&SignalLevel> {
        self.rx_signal_levels().get(signal_type)
    }

    fn rx_signal_level_mut(
        &mut self,
        signal_type: &SignalType
    ) -> Option<&mut SignalLevel> {
        self.rx_signal_levels_mut().get_mut(signal_type)
    }
}

pub trait Transceiver: Transmitter + Receiver {}


fn generate_drone_id() -> u32 {
    thread_rng().gen_range(1..ID_RANGE)
}


#[derive(Clone)]
pub struct CommandCenter {
    id: u32,
    position_in_metres: Coordinates3D,
    signal_levels: HashMap<SignalType, SignalLevel>,
    area: SignalAreaType,
}

impl CommandCenter {
    pub fn new(
        position_in_metres: Coordinates3D,
        area: SignalAreaType
    ) -> Self {
        let mut signal_levels = HashMap::new();

        signal_levels.insert(SignalType::GPS, SignalLevel::Black);
        signal_levels.insert(SignalType::Control, SignalLevel::Green);
        
        Self {
            id: generate_drone_id(),
            position_in_metres,
            signal_levels,
            area,
        }
    }

    pub fn id(&self) -> u32 {
        self.id
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
}

// TODO separate tx and rx signals
impl Transmitter for CommandCenter {
    fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.signal_levels
    }
    
    fn tx_signal_levels_mut(
        &mut self
    ) -> &mut HashMap<SignalType, SignalLevel> {
        &mut self.signal_levels
    }

    fn area(&self) -> &SignalAreaType {
        &self.area
    }
}

impl Receiver for CommandCenter {
    fn rx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.signal_levels
    }
    
    fn rx_signal_levels_mut(
        &mut self
    ) -> &mut HashMap<SignalType, SignalLevel> {
        &mut self.signal_levels
    }
}

impl Transceiver for CommandCenter {}


#[derive(Clone, Debug)]
pub struct Drone {
    id: u32,
    max_speed_in_metresps: f32,
    tx_area: SignalAreaType, 
    global_position_in_metres: Coordinates3D,
    position_in_metres: Coordinates3D,
    velocity_in_metresps: Coordinates3D,
    destination_in_metres: Coordinates3D,
    signal_levels: HashMap<SignalType, SignalLevel>,
    // TODO infection_state
    command_center_id: u32,
    goal: Goal,
}

impl Drone {
    pub fn new(position_in_metres: Coordinates3D) -> Self {
        let tx_area = SignalAreaType::Dome(
            MAX_DRONE_BROADCAST_RADIUS_IN_METRES
        );
        let mut signal_levels = HashMap::new();

        signal_levels.insert(SignalType::GPS, SignalLevel::Green);
        signal_levels.insert(SignalType::Control, SignalLevel::Black);

        Self {
            id: generate_drone_id(), 
            max_speed_in_metresps: MAX_DRONE_SPEED_IN_METRES_PER_S,
            tx_area,
            global_position_in_metres: position_in_metres,
            position_in_metres,
            velocity_in_metresps: Coordinates3D::new(0.0, 0.0, 0.0),
            destination_in_metres: position_in_metres,
            signal_levels,
            command_center_id: UNKNOWN_ID,
            goal: Goal::Reposition,
        }
    }

    pub fn id(&self) -> u32 {
        self.id
    }

    pub fn connected(&self, signal_type: &SignalType) -> bool {
        match self.signal_levels.get(signal_type) {
            Some(signal_level) => {
                if let SignalLevel::Black = signal_level {
                    return false;
                }

                true
            }
            None => false
        }
    }

    pub fn process_message(&mut self, message: &Message) {
        match message.message_type() {
            MessageType::SetDestination(destination, goal) => {
                self.set_destination(*destination);
                self.goal = *goal;
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

        if self.connected(&SignalType::GPS) {
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
        self.signal_levels.insert(SignalType::GPS, SignalLevel::Green);
    }

    pub fn reached_destination(&mut self) {
        let distance = self.distance_to(&self.destination_in_metres);
        
        if distance > DESTINATION_RADIUS_IN_METRES {
            return;
        }

        if matches!(self.goal, Goal::Attack) { 
            self.signal_levels.insert(SignalType::Control, SignalLevel::Black);
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
        self.global_position_in_metres
    }
}

// TODO separate tx and rx signals
impl Transmitter for Drone {
    fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.signal_levels
    }
    
    fn tx_signal_levels_mut(
        &mut self
    ) -> &mut HashMap<SignalType, SignalLevel> {
        &mut self.signal_levels
    }

    fn area(&self) -> &SignalAreaType {
        &self.tx_area
    }
}

impl Receiver for Drone {
    fn rx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.signal_levels
    }
    
    fn rx_signal_levels_mut(
        &mut self
    ) -> &mut HashMap<SignalType, SignalLevel> {
        &mut self.signal_levels
    }
}

impl Transceiver for Drone {}
