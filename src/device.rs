use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::sync::atomic::{AtomicUsize, Ordering};

use thiserror::Error;

use crate::infection::{InfectionState, InfectionType, JAMMING_SIGNAL_LEVEL};
use crate::mathphysics::{
    Megahertz, Meter, MeterPerSecond, Millisecond, Point3D, Position, 
    equation_of_motion_3d, millis_to_secs, 
};
use crate::message::{Goal, Message, MessageType};
use crate::signal::{
    FreqToLevelMap, SignalArea, SignalLevel, GPS_L1_FREQUENCY, GPS_L2_FREQUENCY, 
    NO_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY
};

use systems::{
    MovementSystem, MovementSystemBuildError, ReceiveMessageError, TRXSystem
};


pub use connections::*;
pub use idmaps::{IdToDeviceMap, IdToGoalMap, IdToLevelMap};


pub mod connections;
pub mod idmaps;
pub mod systems;
pub mod networkmodel;


pub type DeviceId = usize;


pub const STEP_DURATION: Millisecond = 50;
// This ID is also a broadcast ID.
pub const UNKNOWN_ID: DeviceId       = 0;


static FREE_DEVICE_ID: AtomicUsize = AtomicUsize::new(1);


fn generate_device_id() -> DeviceId {
    FREE_DEVICE_ID.fetch_add(1, Ordering::SeqCst)
}


pub const DESTINATION_RADIUS: Meter = 5.0;
pub const MAX_DRONE_SPEED: MeterPerSecond  = 25.0;


#[derive(Clone, Debug, Error)]
pub enum DeviceBuildError {
    #[error("Failed to build a system")]
    SystemBuildFailure
}

impl From<MovementSystemBuildError> for DeviceBuildError {
    fn from(_error: MovementSystemBuildError) -> Self {
        Self::SystemBuildFailure
    }
}


#[derive(Clone, Debug)]
pub struct DeviceBuilder {
    global_position_in_meters: Option<Point3D>,
    goal: Option<Goal>,
    max_speed: Option<MeterPerSecond>,
    trx_system: Option<TRXSystem>,
    vulnerabilities: Option<Vec<InfectionType>>
}

impl DeviceBuilder {
    #[must_use]
    pub fn new() -> Self {
        Self {
            global_position_in_meters: None,
            goal: None,
            max_speed: None,
            trx_system: None,
            vulnerabilities: None
        }
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
    pub fn set_max_speed(mut self, max_speed: MeterPerSecond) -> Self {
        self.max_speed = Some(max_speed);
        self
    }
    
    #[must_use]
    pub fn set_trx_system(mut self, trx_system: TRXSystem) -> Self {
        self.trx_system = Some(trx_system);
        self
    }

    #[must_use]
    pub fn set_vulnerabilities(
        mut self, 
        vulnerabilities: &[InfectionType]
    ) -> Self {
        self.vulnerabilities = Some(vulnerabilities.to_vec());
        self
    }
   
    /// # Errors
    ///
    /// Will return `Err` if `Device::build` fails.
    #[must_use]
    pub fn build(self) -> Result<Device, DeviceBuildError> {
        Device::build(
            generate_device_id(),
            self.global_position_in_meters.unwrap_or_default(),
            self.goal.unwrap_or_default(),
            self.max_speed.unwrap_or_default(),
            self.trx_system.unwrap_or_default(),
            self.vulnerabilities.unwrap_or_default().as_ref()
        )
    }
}

impl Default for DeviceBuilder {
    fn default() -> Self {
        Self::new()
    }
}


#[derive(Clone, Debug)]
pub struct Device {
    id: DeviceId,
    global_position_in_meters: Point3D,
    goal: Goal,
    movement_system: MovementSystem,
    trx_system: TRXSystem,
    infection_states: HashMap<InfectionType, InfectionState>
}

impl Device {
    /// # Errors
    ///
    /// Will return `Err` if `MovementSystem::build` fails.
    pub fn build(
        id: DeviceId,
        global_position_in_meters: Point3D,
        goal: Goal,
        max_speed_in_mps: MeterPerSecond,
        trx_system: TRXSystem,
        vulnerabilities: &[InfectionType]
    ) -> Result<Self, DeviceBuildError> {
        let infection_states = vulnerabilities
            .iter()
            .map(|infection_type| (*infection_type, InfectionState::Vulnerable))
            .collect();
        let movement_system = MovementSystem::build(max_speed_in_mps)?;

        Ok(
            Self {
                id,
                global_position_in_meters,
                goal,
                movement_system,
                trx_system,
                infection_states
            }
        )
    }

    #[must_use]
    pub fn id(&self) -> DeviceId {
        self.id
    }
    
    #[must_use]
    pub fn goal(&self) -> &Goal {
        &self.goal
    }
    
    #[must_use]
    pub fn position_without_gps(&self) -> &Point3D {
        self.movement_system.position()
    }

    #[must_use]
    pub fn tx_signal_levels(&self) -> &FreqToLevelMap {
        self.trx_system.tx_signal_levels()
    }
    
    #[must_use]
    pub fn tx_signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        self.trx_system.tx_signal_level(frequency)
    }
    
    #[must_use]
    pub fn area(&self, frequency: Megahertz) -> SignalArea {
        self.trx_system.area(frequency)
    }

    #[must_use]
    pub fn connection_distance<P: Position>(
        &self, 
        object: &P, 
        frequency: Megahertz
    ) -> Option<Meter> {
        self.trx_system.connection_distance(
            self.distance_to(object), 
            frequency
        )
    }
    
    #[must_use]
    pub fn propagated_signal_level_at(
        &self,
        receiver: &Self,
        frequency: Megahertz
    ) -> SignalLevel {
        let distance_to_rx = self.distance_to(receiver);

        self.trx_system.tx_signal_level_at(frequency, distance_to_rx)
    }
    
    pub fn propagate_signal (
        &self,
        receiver: &mut Self,
        frequency: Megahertz
    ) {
        let propagated_signal_level_at_rx = self.propagated_signal_level_at(
            receiver, 
            frequency
        );

        receiver.receive_signal(propagated_signal_level_at_rx, frequency);
    }
    
    pub fn suppress_signal(
        &self,
        receiver: &mut Self,
        frequency: Megahertz
    ) {
        let suppressor_signal_level_at_rx = self.propagated_signal_level_at(
            receiver,
            frequency
        );
        
        receiver.signal_level_suppression(
            suppressor_signal_level_at_rx,
            frequency, 
        );
    }
   
    #[must_use]
    pub fn rx_signal_levels(&self) -> &FreqToLevelMap {
        self.trx_system.rx_signal_levels()
    }

    #[must_use]
    pub fn rx_signal_level(&self, frequency: Megahertz) -> &SignalLevel {
        self.trx_system.rx_signal_level(frequency)
    }
  
    #[must_use]
    pub fn receives_signal(&self, frequency: Megahertz) -> bool {
        self.trx_system.receives_signal(frequency)
    }
    
    #[must_use]
    pub fn infection_states(
        &self, 
    ) -> &HashMap<InfectionType, InfectionState> {
        &self.infection_states
    }   

    #[must_use]
    pub fn is_infected(&self) -> bool {
        self.infection_states
            .values()
            .any(|infection_state| 
                matches!(infection_state, InfectionState::Infected)
            )
    }
    
    #[must_use]
    pub fn infection_state(
        &self, 
        infection_type: &InfectionType
    ) -> &InfectionState {
        self.infection_states
            .get(infection_type)
            .unwrap_or(&InfectionState::Patched)
    } 

    pub fn set_tx_signal_level(
        &mut self, 
        signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        self.trx_system.set_tx_signal_level(signal_level, frequency);
    }
    
    pub fn set_rx_signal_level(
        &mut self, 
        signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        self.trx_system.set_rx_signal_level(signal_level, frequency);
    }
    
    pub fn receive_signal(
        &mut self, 
        signal_level: SignalLevel,
        frequency: Megahertz,
    ) {
        self.trx_system.receive_signal(signal_level, frequency); 
    }

    /// # Errors
    ///
    /// Will return `Err` if message is not received.
    pub fn receive_message(
        &mut self,
        message: &Message,
        frequency: Megahertz,
    ) -> Result<(), ReceiveMessageError> {
        let destination_id = message.destination_id();

        if destination_id != UNKNOWN_ID && self.id() != destination_id {
            return Err(ReceiveMessageError::WrongDestination);
        }

        self.trx_system.receive_message(frequency, message) 
    }
    
    pub fn signal_level_suppression(
        &mut self,
        suppressor_signal_level: SignalLevel,
        suppressor_frequency: Megahertz,
    ) {
        self.trx_system.suppress_signal(
            suppressor_signal_level,
            suppressor_frequency, 
        );
    }

    /// # Errors
    ///
    /// Will return Err if destination device ID of message is not broadcast 
    /// nor drone's device ID or if `Device::receive_message` returns Err.
    pub fn receive_and_process_message(
        &mut self, 
        message: &Message,
        frequency: Megahertz, 
    ) -> Result<(), ReceiveMessageError> {
        self.receive_message(message, frequency)?;

        match message.message_type() {
            MessageType::SetGoal(goal) => {
                self.goal = *goal;
            },
            MessageType::Infection(infection_type) => {
                if let InfectionState::Vulnerable = self
                    .infection_state(infection_type)
                {
                    self.infection_states.insert(
                        *infection_type, 
                        InfectionState::Infected
                    );
                }
            },
        };

        Ok(())
    }

    pub fn update_state(&mut self) {
        let gps_is_connected = self.receives_signal(GPS_L1_FREQUENCY) 
            || self.receives_signal(GPS_L2_FREQUENCY);

        match self.goal {
            Goal::Attack(destination) | Goal::Reposition(destination) 
                if gps_is_connected => {
                self.update_current_position();
                self.movement_system.update_movement_direction(destination);
            },
            Goal::Attack(destination) | Goal::Reposition(destination) =>
                self.update_movement_direction_without_gps(destination),
            Goal::Undefined if gps_is_connected =>
                self.update_current_position(),
            Goal::Undefined => ()
        };
        
        self.update_global_position();
    }
    
    fn update_movement_direction_without_gps(&mut self, destination: Point3D) {
        self.movement_system.update_movement_direction(destination);
        
        let mut velocity = *self.movement_system.velocity();

        velocity.initial_point.z = 0.0;
        velocity.terminal_point.z = 0.0;

        self.movement_system.set_velocity(velocity);
    }

    fn update_global_position(&mut self) {
        self.global_position_in_meters = equation_of_motion_3d(
            &self.global_position_in_meters,
            &self.movement_system.velocity().displacement(),
            millis_to_secs(STEP_DURATION),
        );
    }

    fn update_current_position(&mut self) {
        self.movement_system.set_position(self.global_position_in_meters);
        self.try_reach_goal();
    }

    // Device can check if it has reached the goal only if it knows
    // its current position (if it has GPS connection).
    fn try_reach_goal(&mut self) {
        match self.goal {
            Goal::Attack(destination) 
                if self.distance_to(&destination) <= DESTINATION_RADIUS => 
                self.selfdestruction(),
            Goal::Reposition(destination) 
                if self.distance_to(&destination) <= DESTINATION_RADIUS => 
                self.goal = Goal::Undefined,
            _ => (),
        }
    }

    fn selfdestruction(&mut self) {
        self.trx_system
            .set_rx_signal_level(NO_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY);
    }

    // Handling is done on the network model level.
    pub fn handle_infection(&mut self) {
        let infections: Vec<InfectionType> = self.infection_states
            .iter()
            .filter_map(|(infection_type, infection_state)| 
                match infection_state {
                    InfectionState::Infected => Some(*infection_type),
                    _ => None
                }
            )
            .collect();

        for infection_type in &infections {
            match infection_type {
                InfectionType::Indicator => (),
                InfectionType::Jamming => self.handle_jamming()
            }
        }
    }

    fn handle_jamming(&mut self) {
        let rx_frequencies: Vec<Megahertz> = self
            .rx_signal_levels()
            .keys()
            .copied()
            .collect();

        for frequency in rx_frequencies {
            self.signal_level_suppression(JAMMING_SIGNAL_LEVEL, frequency);
        }
    }
}

impl Default for Device {
    fn default() -> Self {
        let infection_states = HashMap::from([
            (InfectionType::Jamming, InfectionState::Vulnerable)
        ]);
        let movement_system = MovementSystem::build(MAX_DRONE_SPEED)
            .unwrap();

        Self {
            id: generate_device_id(),
            global_position_in_meters: Point3D::default(),
            goal: Goal::Undefined,
            movement_system,
            trx_system: TRXSystem::default(),
            infection_states
        }
    }
}

impl Eq for Device {}

impl PartialEq for Device {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Hash for Device {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

impl Position for Device {
    fn position(&self) -> &Point3D {
        &self.global_position_in_meters
    }
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::device::systems::TRXModule;
    use crate::signal::{
        BLACK_SIGNAL_LEVEL, GREEN_SIGNAL_LEVEL, RED_SIGNAL_LEVEL
    };

    use super::*;


    const CC_TX_CONTROL_RADIUS: Meter    = 300.0;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const EWD_TX_CONTROL_RADIUS: Meter   = 100.0;
    const EWD_TX_GPS_RADIUS: Meter       = 50.0;
    

    fn cc_tx_module() -> TRXModule {
        let max_tx_signal_levels = HashMap::from([(
            WIFI_2_4GHZ_FREQUENCY, 
            SignalLevel::from_area(
                SignalArea::build(CC_TX_CONTROL_RADIUS).unwrap(), 
                WIFI_2_4GHZ_FREQUENCY
            )
        )]);

        TRXModule::build(
            max_tx_signal_levels.clone(), 
            max_tx_signal_levels
        ).unwrap()
    }

    fn drone_tx_module() -> TRXModule {
        let max_tx_signal_levels = HashMap::from([(
            WIFI_2_4GHZ_FREQUENCY, 
            SignalLevel::from_area(
                SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap(), 
                WIFI_2_4GHZ_FREQUENCY
            )
        )]);

        TRXModule::build(
            max_tx_signal_levels.clone(),
            max_tx_signal_levels
        ).unwrap()
    }
    
    fn drone_rx_module() -> TRXModule {
        let max_rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL),
            (WIFI_2_4GHZ_FREQUENCY, GREEN_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, RED_SIGNAL_LEVEL),
            (WIFI_2_4GHZ_FREQUENCY, NO_SIGNAL_LEVEL)
        ]);

        TRXModule::build(
            max_rx_signal_levels,
            rx_signal_levels
        ).unwrap()
    }
    
    fn ewd_signal_levels() -> FreqToLevelMap {
        HashMap::from([
            (
                WIFI_2_4GHZ_FREQUENCY, 
                SignalLevel::from_area(
                    SignalArea::build(EWD_TX_CONTROL_RADIUS).unwrap(), 
                    WIFI_2_4GHZ_FREQUENCY
                )
            ),
            (
                GPS_L1_FREQUENCY, 
                SignalLevel::from_area(
                    SignalArea::build(EWD_TX_GPS_RADIUS).unwrap(),
                    GPS_L1_FREQUENCY
                )
            )
        ])
    }

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

    fn reached_destination(drone: &Device, destination: &Point3D) -> bool {
        drone.distance_to(destination) <= DESTINATION_RADIUS
    }


    #[test]
    fn unique_device_ids() {
        let shared_device_builder = DeviceBuilder::new();

        let command_center = shared_device_builder
            .clone()
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let electronic_warfare = shared_device_builder
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let drone = DeviceBuilder::new()
            .build()
            .unwrap_or_else(|error| panic!("{}", error));

        assert_ne!(command_center.id(), electronic_warfare.id());
        assert_ne!(command_center.id(), drone.id());
        assert_ne!(electronic_warfare.id(), drone.id());
    }

    #[test]
    fn cc_connection_to_drone() {
        let command_center = DeviceBuilder::new()
            .set_trx_system(
                TRXSystem::Strength {
                    tx_module: cc_tx_module(),
                    rx_module: TRXModule::default()
                }
            )
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let mut drone = DeviceBuilder::new()
            .set_trx_system(
                TRXSystem::Strength {
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module()
                }
            )
            .build()
            .unwrap_or_else(|error| panic!("{}", error));

        assert!(
            drone
                .rx_signal_level(WIFI_2_4GHZ_FREQUENCY)
                .is_black()
        );

        command_center.propagate_signal(&mut drone, WIFI_2_4GHZ_FREQUENCY);
        
        assert!(
            drone
                .rx_signal_level(WIFI_2_4GHZ_FREQUENCY)
                .is_green()
        );
    }
    
    #[test]
    fn suppress_tranceivers_by_strength() {
        let control_frequency = WIFI_2_4GHZ_FREQUENCY;
        let gps_frequency = GPS_L1_FREQUENCY;
        
        let drone_trx_system = TRXSystem::Strength {
            tx_module: drone_tx_module(), 
            rx_module: drone_rx_module()
        };
        
        let ewd = DeviceBuilder::new()
            .set_trx_system(
                TRXSystem::Strength {
                    tx_module: TRXModule::build(
                        ewd_signal_levels(),
                        ewd_signal_levels()
                    ).unwrap(),
                    rx_module: TRXModule::default()
                }
            )
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let mut drone_inside = DeviceBuilder::new()
            .set_trx_system(drone_trx_system.clone())
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let mut drone_outside = DeviceBuilder::new()
            .set_global_position(
                Point3D::new(
                    EWD_TX_CONTROL_RADIUS * 20.0,
                    0.0,
                    0.0
                )
            )
            .set_trx_system(drone_trx_system)
            .build()
            .unwrap_or_else(|error| panic!("{}", error));

        ewd.suppress_signal(&mut drone_inside, control_frequency);
        ewd.suppress_signal(&mut drone_inside, gps_frequency);
        ewd.suppress_signal(&mut drone_outside, control_frequency);
        ewd.suppress_signal(&mut drone_outside, gps_frequency);

        assert!(
            *drone_inside
                .tx_signal_level(control_frequency) < BLACK_SIGNAL_LEVEL
        );
        assert!(
            *drone_inside
                .rx_signal_level(gps_frequency) < BLACK_SIGNAL_LEVEL
        );
        assert!(
            *drone_outside
                .tx_signal_level(control_frequency) >= BLACK_SIGNAL_LEVEL
        );
        assert!(
            *drone_outside
                .rx_signal_level(gps_frequency) >= BLACK_SIGNAL_LEVEL
        );
    }
    
    #[test]
    fn no_movement_without_destination_set() {
        let drone_position = Point3D::new(5.0, 0.0, 0.0);

        let mut drone = DeviceBuilder::new()
            .set_global_position(drone_position)
            .set_max_speed(MAX_DRONE_SPEED)
            .set_trx_system(drone_green_rx_system(GPS_L1_FREQUENCY))
            .build()
            .unwrap_or_else(|error| panic!("{}", error));

        assert_eq!(
            *drone.position_without_gps(), 
            Point3D::default()
        );
        assert_eq!(
            *drone.position(), 
            drone_position
        );

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            drone.update_state();

            assert_eq!(
                *drone.position_without_gps(), 
                drone_position
            );
            assert_eq!(
                *drone.position(), 
                drone_position
            );
        }
    }

    #[test]
    fn drone_movement_without_gps() {
        let destination_point = Point3D::new(MAX_DRONE_SPEED, 0.0, 0.0);
        
        let mut drone_without_gps = DeviceBuilder::new()
            .set_max_speed(MAX_DRONE_SPEED)
            .set_trx_system(drone_green_rx_system(WIFI_2_4GHZ_FREQUENCY))
            .build()
            .unwrap_or_else(|error| panic!("{}", error));

        let reposition_message = Message::new(
            UNKNOWN_ID, 
            drone_without_gps.id(), 
            0, 
            MessageType::SetGoal(Goal::Reposition(destination_point))
        );
        let _ = drone_without_gps.receive_and_process_message(
            &reposition_message,
            WIFI_2_4GHZ_FREQUENCY, 
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
        
        let mut drone = DeviceBuilder::new()
            .set_max_speed(MAX_DRONE_SPEED)
            .set_trx_system(drone_green_rx_system(WIFI_2_4GHZ_FREQUENCY))
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        
        let reposition_message = Message::new(
            UNKNOWN_ID, 
            drone.id(), 
            0, 
            MessageType::SetGoal(Goal::Reposition(destination_point))
        );
        let _ = drone.receive_and_process_message(
            &reposition_message,
            WIFI_2_4GHZ_FREQUENCY, 
        ); 

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            drone.update_state();
        }

        assert!(reached_destination(&drone, &destination_point));
    }

    #[test]
    fn process_correct_message() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        let goal = Goal::Attack(Point3D::new(5.0, 0.0, 0.0));

        let mut drone = DeviceBuilder::new()
            .set_trx_system(drone_green_rx_system(frequency))
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        
        let message = Message::new(
            drone.id(),
            drone.id(),
            0, 
            MessageType::SetGoal(goal)
        );

        assert!(drone.receive_and_process_message(&message, frequency).is_ok());
        assert_eq!(goal, drone.goal);
    }

    #[test]
    fn process_broadcast_message() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        let goal = Goal::Attack(Point3D::new(5.0, 0.0, 0.0));
        let mut drone = DeviceBuilder::new()
            .set_trx_system(drone_green_rx_system(frequency))
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        
        let message = Message::new(
            UNKNOWN_ID,
            UNKNOWN_ID,
            0, 
            MessageType::SetGoal(goal)
        );

        assert!(drone.receive_and_process_message(&message, frequency).is_ok());
        assert_eq!(goal, drone.goal);
    }

    #[test]
    fn receive_and_process_message_with_wrong_destination() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let mut drone = DeviceBuilder::new()
            .set_trx_system(drone_green_rx_system(frequency))
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        
        let message = Message::new(
            drone.id() + 1,
            drone.id() + 1,
            0, 
            MessageType::SetGoal(Goal::Undefined)
        );

        assert!(
            matches!(
                drone.receive_and_process_message(&message, frequency),
                Err(ReceiveMessageError::WrongDestination)
            )
        );
    }

    #[test]
    fn invulnerable_drone_infection() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        let infection_type = InfectionType::Jamming;
        let mut drone = DeviceBuilder::new()
            .set_trx_system(drone_green_rx_system(frequency))
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        
        let message = Message::new(
            UNKNOWN_ID,
            UNKNOWN_ID,
            0, 
            MessageType::Infection(infection_type)
        );

        assert!(!drone.is_infected());
        assert!(
            matches!(
                drone.infection_state(&infection_type),
                InfectionState::Patched
            )
        );

        assert!(drone.receive_and_process_message(&message, frequency).is_ok());
        assert!(!drone.is_infected());
        assert!(
            matches!(
                drone.infection_state(&infection_type),
                InfectionState::Patched
            )
        );
    }

    #[test]
    fn vulnerable_drone_infection() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        let infection_type = InfectionType::Jamming;
        let mut drone = DeviceBuilder::new()
            .set_trx_system(drone_green_rx_system(frequency))
            .set_vulnerabilities(&[infection_type])
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        
        let message = Message::new(
            UNKNOWN_ID,
            UNKNOWN_ID,
            0, 
            MessageType::Infection(infection_type)
        );

        assert!(!drone.is_infected());
        assert!(
            matches!(
                drone.infection_state(&infection_type),
                InfectionState::Vulnerable
            )
        );

        assert!(drone.receive_and_process_message(&message, frequency).is_ok());
        assert!(drone.is_infected());
        assert!(
            matches!(
                drone.infection_state(&infection_type),
                InfectionState::Infected
            )
        );
    }
}
