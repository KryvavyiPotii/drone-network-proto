use std::hash::{Hash, Hasher};
use std::sync::atomic::{AtomicUsize, Ordering};

use thiserror::Error;

use super::mathphysics::{
    Megahertz, Meter, MeterPerSecond, Millisecond, Point3D, Position, PowerUnit, 
    equation_of_motion_3d, millis_to_secs, 
};
use super::message::{Goal, Message, MessageType};
use super::malware::{
    InfectionState, Malware, MalwareToStateMap, MalwareType, 
    JAMMING_SIGNAL_LEVEL
};
use super::signal::{
    FreqToLevelMap, SignalArea, SignalLevel, GPS_L1_FREQUENCY
};

use systems::{
    MovementSystem, PowerSystem, PowerSystemError, TRXSystem, TRXSystemError
};


pub use connections::*;
pub use idmaps::{IdToDeviceMap, IdToGoalMap, IdToLevelMap};


pub mod connections;
pub mod idmaps;
pub mod systems;
pub mod networkmodel;


pub type DeviceId = usize;


pub const STEP_DURATION: Millisecond = 50;

pub const BROADCAST_ID: DeviceId     = 0;
pub const UNKNOWN_ID: DeviceId       = 0;

pub const DESTINATION_RADIUS: Meter = 5.0;
pub const MAX_DRONE_SPEED: MeterPerSecond  = 25.0;


const MOVEMENT_POWER_CONSUMPTION:   PowerUnit = 5; 
const PASSIVE_POWER_CONSUMPTION:    PowerUnit = 1; 
const PROCESSING_POWER_CONSUMPTION: PowerUnit = 5; 

static FREE_DEVICE_ID: AtomicUsize = AtomicUsize::new(1);


fn generate_device_id() -> DeviceId {
    FREE_DEVICE_ID.fetch_add(1, Ordering::SeqCst)
}


#[derive(Debug, Error)]
pub enum DeviceError {
    #[error("System is disabled")]
    DisabledSystem,
    #[error("Power system failed with error `{0}`")]
    PowerSystemError(#[from] PowerSystemError),
    #[error("TRX system failed with error `{0}`")]
    TRXSystemError(#[from] TRXSystemError),
}


#[derive(Clone, Debug)]
pub struct DeviceBuilder {
    real_position_in_meters: Option<Point3D>,
    goal: Option<Goal>,
    power_system: Option<PowerSystem>,
    movement_system: Option<MovementSystem>,
    trx_system: Option<TRXSystem>,
    vulnerabilities: Option<Vec<Malware>>,
}

impl DeviceBuilder {
    #[must_use]
    pub fn new() -> Self {
        Self {
            real_position_in_meters: None,
            goal: None,
            power_system: None,
            movement_system: None,
            trx_system: None,
            vulnerabilities: None
        }
    }

    #[must_use]
    pub fn set_real_position(
        mut self, 
        real_position_in_meters: Point3D
    ) -> Self {
        self.real_position_in_meters = Some(real_position_in_meters);
        self
    }
    
    #[must_use]
    pub fn set_goal(mut self, goal: Goal) -> Self {
        self.goal = Some(goal);
        self
    }
    
    #[must_use]
    pub fn set_power_system(mut self, power_system: PowerSystem) -> Self {
        self.power_system = Some(power_system);
        self
    }
    
    #[must_use]
    pub fn set_movement_system(
        mut self, 
        movement_system: MovementSystem
    ) -> Self {
        self.movement_system = Some(movement_system);
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
        vulnerabilities: &[Malware]
    ) -> Self {
        self.vulnerabilities = Some(vulnerabilities.to_vec());
        self
    }
   
    #[must_use]
    pub fn build(self) -> Device {
        Device::new(
            generate_device_id(),
            self.real_position_in_meters.unwrap_or_default(),
            self.goal.unwrap_or_default(),
            self.power_system.unwrap_or_default(),
            self.movement_system.unwrap_or_default(),
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
    real_position_in_meters: Point3D,
    goal: Goal,
    power_system: PowerSystem,
    movement_system: MovementSystem,
    trx_system: TRXSystem,
    // TODO HashMap<MalwareId, InfectionState>
    infection_states: MalwareToStateMap,
}

impl Device {
    #[must_use]
    pub fn new(
        id: DeviceId,
        real_position_in_meters: Point3D,
        goal: Goal,
        power_system: PowerSystem,
        movement_system: MovementSystem,
        trx_system: TRXSystem,
        vulnerabilities: &[Malware]
    ) -> Self {
        let infection_states = vulnerabilities
            .iter()
            .map(|malware| (*malware, InfectionState::Vulnerable))
            .collect();

        Self {
            id,
            real_position_in_meters,
            goal,
            power_system,
            movement_system,
            trx_system,
            infection_states,
        }
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
    pub fn gps_position(&self) -> &Point3D {
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
    
    // self - transmitter
    #[must_use]
    pub fn transmits_at(&self, distance: Meter, frequency: Megahertz) -> bool {
        self.trx_system.transmits_to(distance, frequency)
    }
    
    // self - transmitter
    #[must_use]
    pub fn transmits_to<P: Position>(
        &self, 
        object: &P, 
        frequency: Megahertz
    ) -> bool {
        let distance = self.distance_to(object);

        self.transmits_at(distance, frequency)
    }
    
    // self - transmitter
    #[must_use]
    pub fn propagated_signal_level_at(
        &self,
        receiver: &Self,
        frequency: Megahertz
    ) -> SignalLevel {
        let distance_to_rx = self.distance_to(receiver);

        self.trx_system.tx_signal_level_at(frequency, distance_to_rx)
    }
    
    // self - transmitter
    pub fn propagate_signal(
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

    // self - EWD 
    pub fn suppress_all_signals(&self, receiver: &mut Self) {
        for frequency in self.tx_signal_levels().keys() {
            self.suppress_signal(receiver, *frequency);    
        }
    }
    
    // self - EWD 
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
    pub fn infection_states(&self) -> &MalwareToStateMap {
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
    pub fn is_infected_with(&self, malware: &Malware) -> bool {
        matches!(
            self.infection_states.get(malware),
            Some(InfectionState::Infected),
        )
    }
    
    #[must_use]
    pub fn infection_state(
        &self, 
        malware: &Malware
    ) -> &InfectionState {
        self.infection_states
            .get(malware)
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
    ) -> Result<(), TRXSystemError> {
        let destination_id = message.destination_id();

        if destination_id != BROADCAST_ID && self.id() != destination_id {
            return Err(TRXSystemError::WrongMessageDestination);
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
    ) -> Result<(), DeviceError> {
        self.receive_message(message, frequency)?;
        self.try_consume_power(PROCESSING_POWER_CONSUMPTION)?;
        self.process_message(message); 

        Ok(())
    }

    fn process_message(&mut self, message: &Message) {
        match message.message_type() {
            MessageType::GPS(gps_position) => {
                self.movement_system.set_position(*gps_position);
            },
            MessageType::Malware(malware)  => {
                self.process_malware(malware);
            },
            MessageType::SetGoal(goal)     => {
                self.goal = *goal;
            },
        }
    }

    fn process_malware(&mut self, malware: &Malware) {
        if matches!(
            self.infection_state(malware), 
            InfectionState::Vulnerable
        ) {
            self.infection_states.insert(*malware, InfectionState::Infected);
        }
    }

    /// # Errors
    ///
    /// Will return `Err` if all power is consumed or the movement system is
    /// disabled.
    pub fn update_state(&mut self) -> Result<(), DeviceError> {
        self.try_consume_power(PASSIVE_POWER_CONSUMPTION)?;
        self.process_goal();
        self.update_real_position()?;

        Ok(())
    }

    fn process_goal(&mut self) {
        let gps_is_connected = self.receives_signal(GPS_L1_FREQUENCY); 

        match self.goal {
            Goal::Attack(destination) | Goal::Reposition(destination) 
                if gps_is_connected => {
                self.movement_system.update_movement_direction(destination);
                self.try_reach_goal();
            },
            Goal::Attack(_) | Goal::Reposition(_) => {
                self.update_movement_direction_without_gps();
            },
            Goal::Undefined => ()
        }
    }
    
    fn update_movement_direction_without_gps(&mut self) {
        let mut velocity = *self.movement_system.velocity();

        velocity.initial_point.z = 0.0;
        velocity.terminal_point.z = 0.0;
        velocity.scale_to(self.movement_system.max_speed());

        self.movement_system.set_velocity(velocity);
    }

    fn update_real_position(&mut self) -> Result<(), DeviceError> {
        if self.movement_system.is_disabled() {
            return Err(DeviceError::DisabledSystem);
        }

        self.try_consume_power(MOVEMENT_POWER_CONSUMPTION)?;
        
        self.real_position_in_meters = equation_of_motion_3d(
            &self.real_position_in_meters,
            &self.movement_system.velocity().displacement(),
            millis_to_secs(STEP_DURATION),
        );
        
        Ok(())
    }

    fn try_consume_power(
        &mut self, 
        power: PowerUnit
    ) -> Result<(), PowerSystemError> {
        self.power_system
            .try_consume_power(power)
            .map_err(|error| {
                self.selfdestruction();

                error
            })
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
        self.goal            = Goal::Undefined;
        self.power_system    = PowerSystem::default();
        self.movement_system = MovementSystem::default();
        self.trx_system      = TRXSystem::default();
    }

    // Handling is done on the network model level.
    pub fn handle_infection(&mut self) {
        let infections: Vec<Malware> = self.infection_states
            .iter()
            .filter_map(|(infection_type, infection_state)| 
                match infection_state {
                    InfectionState::Infected => Some(*infection_type),
                    _ => None
                }
            )
            .collect();

        for malware in &infections {
            match malware.malware_type() {
                MalwareType::DoS(lost_power)    => 
                    self.handle_dos(*lost_power),
                MalwareType::Indicator          => 
                    (),
                MalwareType::Jamming(frequency) => 
                    self.handle_jamming(*frequency)
            }
        }
    }
    
    fn handle_dos(&mut self, lost_power: PowerUnit) {
        let _ = self.try_consume_power(lost_power);
    }

    fn handle_jamming(&mut self, frequency: Megahertz) {
        self.signal_level_suppression(JAMMING_SIGNAL_LEVEL, frequency);
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
        &self.real_position_in_meters
    }
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::backend::device::systems::TRXModule;
    use crate::backend::signal::{
        BLACK_SIGNAL_LEVEL, GREEN_SIGNAL_LEVEL, NO_SIGNAL_LEVEL, 
        RED_SIGNAL_LEVEL, WIFI_2_4GHZ_FREQUENCY
    };

    use super::*;


    const CC_TX_CONTROL_RADIUS: Meter    = 300.0;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const EWD_TX_CONTROL_RADIUS: Meter   = 100.0;
    const EWD_TX_GPS_RADIUS: Meter       = 50.0;
    const DEVICE_MAX_POWER: PowerUnit    = 1_000;
    

    fn device_power_system() -> PowerSystem {
        PowerSystem::build(DEVICE_MAX_POWER, DEVICE_MAX_POWER)
            .unwrap_or_else(|error| panic!("{}", error))
    }

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

    fn drone_movement_system() -> MovementSystem {
        MovementSystem::build(MAX_DRONE_SPEED)
            .unwrap_or_else(|error| panic!("{}", error))
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

    fn device_is_destructed(device: &Device) -> bool {
        device.goal == Goal::Undefined
        && device.power_system == PowerSystem::default()
        && device.trx_system == TRXSystem::default()
        && device.movement_system == MovementSystem::default()
    }

    fn jamming_malware(jammed_frequency: Megahertz) -> Malware {
        Malware::new(
            0, 
            MalwareType::Jamming(jammed_frequency),
            false,
        )
    }


    #[test]
    fn unique_device_ids() {
        let shared_device_builder = DeviceBuilder::new();

        let command_center = shared_device_builder
            .clone()
            .build();
        let electronic_warfare = shared_device_builder.build();
        let drone = DeviceBuilder::new().build();

        assert_ne!(command_center.id(), electronic_warfare.id());
        assert_ne!(command_center.id(), drone.id());
        assert_ne!(electronic_warfare.id(), drone.id());
    }

    #[test]
    fn same_device_ids_on_clone() {
        let device = DeviceBuilder::new().build();
        let cloned_device = device.clone();

        assert_eq!(device.id(), cloned_device.id())
    }

    #[test]
    fn cc_connection_to_device() {
        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::Strength {
                    tx_module: cc_tx_module(),
                    rx_module: TRXModule::default()
                }
            )
            .build();
        let mut device = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::Strength {
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module()
                }
            )
            .build();

        assert!(
            device
                .rx_signal_level(WIFI_2_4GHZ_FREQUENCY)
                .is_black()
        );

        command_center.propagate_signal(&mut device, WIFI_2_4GHZ_FREQUENCY);
        
        assert!(
            device
                .rx_signal_level(WIFI_2_4GHZ_FREQUENCY)
                .is_green()
        );
    }

    #[test]
    fn device_selfdestructs_after_consuming_all_power() {
        let goal  = Goal::Attack(Point3D::new(5.0, 5.0, 5.0));
        let power = PASSIVE_POWER_CONSUMPTION + MOVEMENT_POWER_CONSUMPTION;
        
        let power_system    = PowerSystem::build(
            power, 
            power,
        ).unwrap_or_else(|error| panic!("{}", error));
        let movement_system = MovementSystem::build(25.0)
            .unwrap_or_else(|error| panic!("{}", error));
        let trx_system      = drone_green_rx_system(WIFI_2_4GHZ_FREQUENCY);

        let mut device = DeviceBuilder::new()
            .set_goal(goal)
            .set_power_system(power_system.clone())
            .set_movement_system(movement_system.clone())
            .set_trx_system(trx_system.clone())
            .build();

        assert_eq!(device.goal, goal);
        assert_eq!(device.power_system, power_system);
        assert_eq!(device.trx_system, trx_system);

        assert!(device.update_state().is_ok());
        assert!(!device_is_destructed(&device));
        
        let _expected_error = DeviceError::PowerSystemError(
            PowerSystemError::NotEnoughPower
        );

        assert!(
            matches!(device.update_state(), Err(_expected_error))
        );
        assert!(device_is_destructed(&device));
    }
    
    #[test]
    fn suppress_tranceivers_by_strength() {
        let control_frequency = WIFI_2_4GHZ_FREQUENCY;
        let gps_frequency = GPS_L1_FREQUENCY;
        
        let device_trx_system = TRXSystem::Strength {
            tx_module: drone_tx_module(), 
            rx_module: drone_rx_module()
        };
        
        let ewd = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::Strength {
                    tx_module: TRXModule::build(
                        ewd_signal_levels(),
                        ewd_signal_levels()
                    ).unwrap(),
                    rx_module: TRXModule::default()
                }
            )
            .build();
            
        let mut device_inside = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(device_trx_system.clone())
            .build();
            
        let mut device_outside = DeviceBuilder::new()
            .set_real_position(
                Point3D::new(
                    EWD_TX_CONTROL_RADIUS * 20.0,
                    0.0,
                    0.0
                )
            )
            .set_power_system(device_power_system())
            .set_trx_system(device_trx_system)
            .build();
            

        ewd.suppress_signal(&mut device_inside, control_frequency);
        ewd.suppress_signal(&mut device_inside, gps_frequency);
        ewd.suppress_signal(&mut device_outside, control_frequency);
        ewd.suppress_signal(&mut device_outside, gps_frequency);

        assert!(
            *device_inside
                .tx_signal_level(control_frequency) < BLACK_SIGNAL_LEVEL
        );
        assert!(
            *device_inside
                .rx_signal_level(gps_frequency) < BLACK_SIGNAL_LEVEL
        );
        assert!(
            *device_outside
                .tx_signal_level(control_frequency) >= BLACK_SIGNAL_LEVEL
        );
        assert!(
            *device_outside
                .rx_signal_level(gps_frequency) >= BLACK_SIGNAL_LEVEL
        );
    }
    
    #[test]
    fn no_movement_without_destination_set() {
        let device_position = Point3D::new(5.0, 0.0, 0.0);

        let mut device = DeviceBuilder::new()
            .set_real_position(device_position)
            .set_power_system(device_power_system())
            .set_movement_system(drone_movement_system())
            .set_trx_system(drone_green_rx_system(GPS_L1_FREQUENCY))
            .build();
            

        assert_eq!(
            *device.gps_position(), 
            Point3D::default()
        );
        assert_eq!(
            *device.position(), 
            device_position
        );

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            assert!(device.update_state().is_ok());

            assert_eq!(
                *device.gps_position(), 
                Point3D::default()
            );
            assert_eq!(
                *device.position(), 
                device_position
            );
        }
    }

    #[test]
    fn device_movement_without_gps() {
        let destination_point = Point3D::new(MAX_DRONE_SPEED, 0.0, 0.0);
        let goal              = Goal::Reposition(destination_point);
        
        let mut device_without_gps = DeviceBuilder::new()
            .set_goal(goal)
            .set_power_system(device_power_system())
            .set_movement_system(drone_movement_system())
            .build();

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            assert!(device_without_gps.update_state().is_ok());
        }

        assert_eq!(
            *device_without_gps.gps_position(), 
            Point3D::default()
        );
        assert_eq!(
            *device_without_gps.position(), 
            Point3D::default()
        );
    }

    #[test]
    fn device_reaching_destination() {
        let destination_point = Point3D::new(MAX_DRONE_SPEED, 0.0, 0.0);
        let goal              = Goal::Reposition(destination_point);
        
        let mut device = DeviceBuilder::new()
            .set_goal(goal)
            .set_power_system(device_power_system())
            .set_movement_system(drone_movement_system())
            .set_trx_system(drone_green_rx_system(GPS_L1_FREQUENCY))
            .build();
            
        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            let gps_message = Message::new(
                UNKNOWN_ID, 
                device.id(), 
                0,
                MessageType::GPS(*device.position())
            );
            assert!(
                device.receive_and_process_message(
                    &gps_message,
                    GPS_L1_FREQUENCY, 
                ).is_ok()
            ); 

            assert!(device.update_state().is_ok());
        }

        assert!(reached_destination(&device, &destination_point));
    }

    #[test]
    fn device_selfdestruction() {
        let goal            = Goal::Attack(Point3D::new(5.0, 5.0, 5.0));
        let power_system    = device_power_system();
        let movement_system = MovementSystem::build(25.0)
            .unwrap_or_else(|error| panic!("{}", error));
        let trx_system      = drone_green_rx_system(WIFI_2_4GHZ_FREQUENCY);

        let mut device = DeviceBuilder::new()
            .set_goal(goal)
            .set_power_system(power_system.clone())
            .set_trx_system(trx_system.clone())
            .set_movement_system(movement_system.clone())
            .build();

        assert_eq!(device.goal, goal);
        assert_eq!(device.power_system, power_system);
        assert_eq!(device.trx_system, trx_system);
        assert_eq!(device.movement_system, movement_system);

        device.selfdestruction();

        assert!(device_is_destructed(&device));
    }

    #[test]
    fn receive_and_process_correct_set_goal_message() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        let goal      = Goal::Attack(Point3D::new(5.0, 0.0, 0.0));

        let mut device = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(drone_green_rx_system(frequency))
            .build();
            
        let message = Message::new(
            UNKNOWN_ID,
            device.id(),
            0, 
            MessageType::SetGoal(goal)
        );

        assert!(
            device.receive_and_process_message(&message, frequency)
                .is_ok()
        );
        assert_eq!(goal, device.goal);
    }
    
    #[test]
    fn receive_and_process_correct_gps_message() {
        let frequency       = GPS_L1_FREQUENCY;
        let global_position = Point3D::new(5.0, 0.0, 0.0);
        let gps_position    = Point3D::new(0.0, 0.0, 5.0);

        let mut device = DeviceBuilder::new()
            .set_real_position(global_position)
            .set_power_system(device_power_system())
            .set_trx_system(drone_green_rx_system(frequency))
            .build();
            
        assert_eq!(device.real_position_in_meters, global_position);
        assert_eq!(device.gps_position(), Point3D::default());

        let message = Message::new(
            UNKNOWN_ID,
            device.id(),
            0, 
            MessageType::GPS(gps_position)
        );

        assert!(
            device.receive_and_process_message(&message, frequency)
                .is_ok()
        );
        assert_eq!(device.real_position_in_meters, global_position);
        assert_eq!(device.gps_position(), gps_position);
    }

    #[test]
    fn receive_and_process_broadcast_message() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        let goal      = Goal::Attack(Point3D::new(5.0, 0.0, 0.0));
        
        let mut device = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(drone_green_rx_system(frequency))
            .build();    
        
        let message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            0, 
            MessageType::SetGoal(goal)
        );

        assert!(
            device.receive_and_process_message(&message, frequency)
                .is_ok()
        );
        assert_eq!(goal, device.goal);
    }

    #[test]
    fn not_receive_and_process_message_with_wrong_destination() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let mut device = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(drone_green_rx_system(frequency))
            .build();
            
        let message = Message::new(
            device.id() + 1,
            device.id() + 1,
            0, 
            MessageType::SetGoal(Goal::Undefined)
        );

        let _expected_error = DeviceError::TRXSystemError(
            TRXSystemError::WrongMessageDestination
        );

        assert!(
            matches!(
                device.receive_and_process_message(&message, frequency),
                Err(_expected_error)
            )
        );
    }

    #[test]
    fn invulnerable_device_infection() {
        let frequency  = WIFI_2_4GHZ_FREQUENCY;
        let malware    = jamming_malware(frequency); 
        let mut device = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(drone_green_rx_system(frequency))
            .build(); 
        
        let message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            0, 
            MessageType::Malware(malware)
        );

        assert!(!device.is_infected());
        assert!(!device.is_infected_with(&malware));
        assert!(
            matches!(
                device.infection_state(&malware),
                InfectionState::Patched
            )
        );

        assert!(
            device.receive_and_process_message(&message, frequency)
                .is_ok()
        );
        
        assert!(!device.is_infected());
        assert!(!device.is_infected_with(&malware));
        assert!(
            matches!(
                device.infection_state(&malware),
                InfectionState::Patched
            )
        );
    }

    #[test]
    fn vulnerable_device_infection() {
        let frequency  = WIFI_2_4GHZ_FREQUENCY;
        let malware    = jamming_malware(frequency); 
        let mut device = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(drone_green_rx_system(frequency))
            .set_vulnerabilities(&[malware])
            .build(); 
        
        let message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            0, 
            MessageType::Malware(malware)
        );

        assert!(!device.is_infected());
        assert!(!device.is_infected_with(&malware));
        assert!(
            matches!(
                device.infection_state(&malware),
                InfectionState::Vulnerable
            )
        );

        assert!(
            device.receive_and_process_message(&message, frequency)
                .is_ok()
        );
        assert!(device.is_infected());
        assert!(device.is_infected_with(&malware));
        assert!(
            matches!(
                device.infection_state(&malware),
                InfectionState::Infected
            )
        );
    }

    #[test]
    fn jamming_affects_only_specified_frequency() {
        let jammed_frequency    = WIFI_2_4GHZ_FREQUENCY;
        let untouched_frequency = GPS_L1_FREQUENCY;
        let jamming_malware     = jamming_malware(jammed_frequency); 

        let green_signal_levels = HashMap::from([
            (jammed_frequency, GREEN_SIGNAL_LEVEL),
            (untouched_frequency, GREEN_SIGNAL_LEVEL)
        ]); 
        let rx_module = TRXModule::build(
            green_signal_levels.clone(), 
            green_signal_levels
        ).unwrap_or_else(|error| panic!("{}", error));
        let rx_system = TRXSystem::Color(rx_module);

        let mut device = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(rx_system)
            .set_vulnerabilities(&[jamming_malware])
            .build();
        
        let message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            0, 
            MessageType::Malware(jamming_malware)
        );

        let signal_level_on_jammed_frequency = *device.rx_signal_level(
            jammed_frequency
        );
        let signal_level_on_untouched_frequency = *device.rx_signal_level(
            untouched_frequency
        );

        assert!(
            device.receive_and_process_message(&message, jammed_frequency)
                .is_ok()
        );
        device.handle_infection();

        let new_signal_level_on_jammed_frequency = device.rx_signal_level(
            jammed_frequency
        );
        let new_signal_level_on_untouched_frequency = device.rx_signal_level(
            untouched_frequency
        );

        assert_eq!(
            signal_level_on_untouched_frequency,
            new_signal_level_on_untouched_frequency
        );
        assert_ne!(
            signal_level_on_jammed_frequency,
            new_signal_level_on_jammed_frequency
        );
    }
}
