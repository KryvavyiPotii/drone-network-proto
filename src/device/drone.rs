use std::hash::{Hash, Hasher};

use crate::communication::{
    FreqToLevelMap, Goal, GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL, Message, 
    MessageType, NO_SIGNAL_LEVEL, SignalArea, SignalLevel, WIFI_2_4GHZ_FREQUENCY,
};
use crate::device::{
    CommandCenter, Device, DeviceId, Receiver, STEP_DURATION, Transceiver, 
    Transmitter, UNKNOWN_ID, generate_device_id,
};
use crate::device::modules::{ReceiveMessageError, TRXSystem};
use crate::mathphysics::{
    Megahertz, Meter, MeterPerSecond, Point3D, Position, Vector3D, 
    equation_of_motion_3d, millis_to_secs, 
};


pub const DESTINATION_RADIUS: Meter = 5.0;
pub const MAX_DRONE_SPEED: MeterPerSecond  = 25.0;


pub struct DroneBuilder<'a> {
    id: DeviceId,
    command_center: Option<&'a CommandCenter>,
    global_position_in_meters: Option<Point3D>,
    goal: Option<Goal>,
    trx_system: Option<TRXSystem>
}

impl<'a> DroneBuilder<'a> {
    #[must_use]
    pub fn new() -> Self {
        Self {
            id: generate_device_id(),
            command_center: None,
            global_position_in_meters: None,
            goal: None,
            trx_system: None
        }
    }

    #[must_use]
    pub fn set_command_center(
        mut self, 
        command_center: &'a CommandCenter
    ) -> Self {
        self.command_center = Some(command_center);
        self
    }

    #[must_use]
    pub fn set_global_position(
        mut self, 
        global_position_in_meters: Point3D
    ) -> Self {
        self.global_position_in_meters = Some(global_position_in_meters);
        self
    }
    
    #[must_use]
    pub fn set_goal(mut self, goal: Goal) -> Self {
        self.goal = Some(goal);
        self
    }
    
    #[must_use]
    pub fn set_trx_system(mut self, trx_system: TRXSystem) -> Self {
        self.trx_system = Some(trx_system);
        self
    }
   
    #[must_use]
    pub fn build(self) -> Drone {
        let command_center_id = match self.command_center {
            Some(command_center) => command_center.id(),
            None => UNKNOWN_ID
        };

        Drone::new(
            self.id,
            command_center_id,
            self.global_position_in_meters.unwrap_or_default(),
            self.goal.unwrap_or_default(),
            self.trx_system.unwrap_or_default()
        )
    }
}

impl Default for DroneBuilder<'_> {
    fn default() -> Self {
        Self::new()
    }
}


// Interactivity:
//     Drone is supposed to be a black box for a user. Only network models 
//     should be able to interact with drones. 
//     The user is only allowed to create a Drone.
// MessageQueue:
//     It is possible to move message queue from network models to each drone 
//     individually but it will lead to substantial performance loss. 
#[derive(Clone, Debug)]
pub struct Drone {
    id: DeviceId,
    command_center_id: DeviceId,
    max_speed: MeterPerSecond,
    global_position_in_meters: Point3D,
    position_in_meters: Point3D,
    velocity: Vector3D,
    destination_in_meters: Point3D,
    goal: Goal,
    trx_system: TRXSystem,
    infection_state: bool
}

impl Drone {
    #[must_use]
    pub fn new(
        id: DeviceId,
        command_center_id: DeviceId,
        global_position_in_meters: Point3D,
        goal: Goal,
        trx_system: TRXSystem
    ) -> Self {
        Self {
            id,
            command_center_id,
            max_speed: MAX_DRONE_SPEED,
            global_position_in_meters,
            // Upon the creation the drone does not know its position.
            // To get it the drone needs GPS connection.
            position_in_meters: Point3D::default(),
            velocity: Vector3D::default(),
            destination_in_meters: global_position_in_meters,
            goal,
            trx_system,
            infection_state: false
        }
    }

    #[must_use]
    pub fn command_center_id(&self) -> DeviceId {
        self.command_center_id
    }
    
    #[must_use]
    pub fn position_without_gps(&self) -> &Point3D {
        &self.position_in_meters
    }

    #[must_use]
    pub fn is_infected(&self) -> bool {
        self.infection_state
    }

    pub fn connect_command_center(&mut self, command_center: &CommandCenter) {
        self.command_center_id = command_center.id();
    }
   
    /// # Errors
    ///
    /// Will return Err if destination device ID of message is not broadcast 
    /// nor drone's device ID or if `Drone::receive_message` returns Err.
    pub fn process_message(
        &mut self, 
        frequency: Megahertz, 
        message: &Message
    ) -> Result<(), ReceiveMessageError> {
        let destination_id = message.destination_id();

        if destination_id != UNKNOWN_ID && self.id() != destination_id {
            return Err(ReceiveMessageError::WrongDestination);
        }

        self.receive_message(frequency, message)?;

        match message.message_type() {
            MessageType::ChangeGoal(goal) => {
                self.goal = *goal;
            },
            MessageType::Infection => {
                self.infection_state = true;
            },
            MessageType::SetDestination(destination) => {
                self.destination_in_meters = *destination;
            },
        };

        Ok(())
    }

    pub fn update_state(&mut self) {
        self.update_position();
    }

    fn update_position(&mut self) {
        if self.receives_signal(GPS_L1_FREQUENCY) {
            self.update_current_position();
            self.update_movement_direction();
        } else {
            self.update_movement_direction_without_gps();
        }
        
        self.update_global_position();
        
        self.connect_gps();
    }
    
    fn update_movement_direction(&mut self) {
        self.velocity = Vector3D::new(
            self.position_in_meters,
            self.destination_in_meters
        );
        
        self.velocity.truncate(self.max_speed);
    }

    fn update_global_position(&mut self) {
        self.global_position_in_meters = equation_of_motion_3d(
            &self.global_position_in_meters,
            &self.velocity.displacement(),
            millis_to_secs(STEP_DURATION),
        );
    }

    fn update_current_position(&mut self) {
        self.position_in_meters = self.global_position_in_meters;
       
        // Drone can check if it has reached the destination only if it knows
        // its current position (if it has GPS connection).
        if self.reached_destination() && matches!(self.goal, Goal::Attack) {
            self.try_attack();
        }
    }

    fn update_movement_direction_without_gps(&mut self) {
        self.update_movement_direction();
        
        self.velocity.initial_point.z = 0.0;
        self.velocity.terminal_point.z = 0.0;
        self.velocity.truncate(self.max_speed);
    }

    fn connect_gps(&mut self) {
        self.trx_system
            .set_rx_signal_level(GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL);
    }

    #[must_use]
    pub fn reached_destination(&self) -> bool {
        let distance = self.distance_to(&self.destination_in_meters);
        
        distance <= DESTINATION_RADIUS
    }

    fn try_attack(&mut self) {
        self.selfdestruction();
    }

    fn selfdestruction(&mut self) {
        self.trx_system
            .set_rx_signal_level(WIFI_2_4GHZ_FREQUENCY, NO_SIGNAL_LEVEL);
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
    fn position(&self) -> &Point3D {
        &self.global_position_in_meters
    }
}

impl Device for Drone {
    fn id(&self) -> DeviceId {
        self.id
    }
}

impl Transmitter for Drone {
    fn signal_levels(&self) -> &FreqToLevelMap {
        self.trx_system.tx_signal_levels()
    }
    
    fn signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        self.trx_system.tx_signal_level(frequency)
    }
    
    fn set_signal_level(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    ) {
        self.trx_system.set_tx_signal_level(frequency, signal_level);
    }

    fn area(&self, frequency: Megahertz) -> SignalArea {
        self.trx_system.area(frequency)
    }

    fn connection_distance<P: Position>(
        &self, 
        object: &P, 
        frequency: Megahertz
    ) -> Option<Meter> {
        self.trx_system.connection_distance(
            self.distance_to(object), 
            frequency
        )
    }
    
    fn propagated_signal_level_at<R: Receiver>(
        &self,
        receiver: &R,
        frequency: Megahertz
    ) -> SignalLevel {
        let distance_to_rx = self.distance_to(receiver);

        self.trx_system.tx_signal_level_at(frequency, distance_to_rx)
    }

    fn propagate_signal<R: Receiver>(
        &self,
        receiver: &mut R,
        frequency: Megahertz
    ) {
        let propagated_signal_level_at_rx = self.propagated_signal_level_at(
            receiver, 
            frequency
        );

        receiver.receive_signal(frequency, propagated_signal_level_at_rx);
    }
}

impl Receiver for Drone {
    fn signal_levels(&self) -> &FreqToLevelMap {
        self.trx_system.rx_signal_levels()
    }

    fn signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        self.trx_system.rx_signal_level(frequency)
    }
  
    fn set_signal_level(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    ) {
        self.trx_system.set_rx_signal_level(frequency, signal_level);
    }

    fn receives_signal(&self, frequency: Megahertz) -> bool {
        self.trx_system.receives_signal(frequency)
    }
    
    fn receive_signal(
        &mut self, 
        frequency: Megahertz,
        signal_level: SignalLevel
    ) {
        self.trx_system.receive_signal(frequency, signal_level); 
    }

    fn receive_message(
        &mut self,
        frequency: Megahertz,
        message: &Message
    ) -> Result<(), ReceiveMessageError> {
        self.trx_system.receive_message(frequency, message) 
    }
    
    fn signal_level_suppression(
        &mut self,
        suppressor_frequency: Megahertz,
        suppressor_signal_level: SignalLevel
    ) {
        self.trx_system.suppress_signal(
            suppressor_frequency, 
            suppressor_signal_level
        );
    }
}

impl Transceiver for Drone {}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::device::modules::TRXModule;

    use super::*;


    fn drone_green_rx_module(frequency: Megahertz) -> TRXModule {
        let max_rx_signal_levels = HashMap::from([
            (frequency, GREEN_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (frequency, GREEN_SIGNAL_LEVEL)
        ]);

        TRXModule::build(
            max_rx_signal_levels,
            rx_signal_levels,
        ).unwrap()   
    }

    fn drone_green_rx_system(frequency: Megahertz) -> TRXSystem {
        TRXSystem::Color(drone_green_rx_module(frequency))
    }


    #[test]
    fn drone_movement_without_gps() {
        let destination_point = Point3D::new(MAX_DRONE_SPEED, 0.0, 0.0);
        
        let mut drone_without_gps = DroneBuilder::new()
            .set_trx_system(drone_green_rx_system(WIFI_2_4GHZ_FREQUENCY))
            .build();

        let set_destination_message = Message::new(
            UNKNOWN_ID, 
            drone_without_gps.id(), 
            0, 
            MessageType::SetDestination(destination_point)
        );
        let _ = drone_without_gps.process_message(
            WIFI_2_4GHZ_FREQUENCY, 
            &set_destination_message
        );

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            drone_without_gps.update_state();
        }

        assert_eq!(
            *drone_without_gps.position_without_gps(), 
            Point3D::default()
        );
        assert_eq!(
            *drone_without_gps.position(), 
            destination_point
        );
    }

    #[test]
    fn drone_reach_destination() {
        let destination_point = Point3D::new(MAX_DRONE_SPEED, 0.0, 0.0);
        
        let mut drone = DroneBuilder::new()
            .set_trx_system(drone_green_rx_system(WIFI_2_4GHZ_FREQUENCY))
            .build();
        
        let set_destination_message = Message::new(
            UNKNOWN_ID, 
            drone.id(), 
            0, 
            MessageType::SetDestination(destination_point)
        );
        let _ = drone.process_message(
            WIFI_2_4GHZ_FREQUENCY, 
            &set_destination_message
        ); 

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            drone.update_state();
        }

        assert!(drone.reached_destination());
    }

    #[test]
    fn process_correct_message() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let mut drone = DroneBuilder::new()
            .set_trx_system(drone_green_rx_system(frequency))
            .build();
        
        let message = Message::new(
            drone.id(),
            drone.id(),
            0, 
            MessageType::ChangeGoal(Goal::Attack)
        );

        let _ = drone.process_message(frequency, &message);

        assert!(matches!(drone.goal, Goal::Attack));
    }

    #[test]
    fn process_broadcast_message() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        let message = Message::new(
            UNKNOWN_ID,
            UNKNOWN_ID,
            0, 
            MessageType::ChangeGoal(Goal::Attack)
        );

        let mut drone = DroneBuilder::new()
            .set_trx_system(drone_green_rx_system(frequency))
            .build();

        let _ = drone.process_message(frequency, &message);

        assert!(matches!(drone.goal, Goal::Attack));
    }

    #[test]
    fn process_message_with_wrong_destination() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let mut drone = DroneBuilder::new()
            .set_trx_system(drone_green_rx_system(frequency))
            .build();
        
        let message = Message::new(
            drone.id() + 1,
            drone.id() + 1,
            0, 
            MessageType::ChangeGoal(Goal::Attack)
        );

        assert!(
            matches!(
                drone.process_message(frequency, &message),
                Err(ReceiveMessageError::WrongDestination)
            )
        );
    }
}
