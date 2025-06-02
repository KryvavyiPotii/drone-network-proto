use std::sync::atomic::{AtomicUsize, Ordering};

use thiserror::Error;

use super::{DESTINATION_RADIUS, ITERATION_TIME};
use super::mathphysics::{
    equation_of_motion_3d, millis_to_secs, Megahertz, Meter, MeterPerSecond, 
    Point3D, Position, PowerUnit,
};
use super::message::{Task, Message, MessageType};
use super::malware::{
    InfectionState, Malware, MalwareToStateMap, MalwareType, 
    JAMMING_SIGNAL_LEVEL
};
use super::signal::{
    FreqToLevelMap, SignalArea, SignalLevel, GPS_L1_FREQUENCY, 
    WIFI_2_4GHZ_FREQUENCY
};

use systems::{
    MovementSystem, PowerSystem, PowerSystemError, TRXSystem, TRXSystemError
};


pub use idmaps::*;


pub mod idmaps;
pub mod systems;


pub type DeviceId = usize;


pub const BROADCAST_ID: DeviceId = DeviceId::MAX;
pub const UNKNOWN_ID: DeviceId   = 0;

pub const MAX_DRONE_SPEED: MeterPerSecond = 25.0;


const MOVEMENT_POWER_CONSUMPTION: PowerUnit   = 5; 
const PASSIVE_POWER_CONSUMPTION: PowerUnit    = 1; 
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


#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum SignalLossResponse {
    Ascend,
    #[default]
    Ignore,
    Hover,
    ReturnToHome(Point3D), // Point3D is a home point
    Shutdown,
}


#[derive(Clone, Debug)]
pub struct DeviceBuilder {
    real_position_in_meters: Option<Point3D>,
    task: Option<Task>,
    power_system: Option<PowerSystem>,
    movement_system: Option<MovementSystem>,
    trx_system: Option<TRXSystem>,
    vulnerabilities: Option<Vec<Malware>>,
    signal_loss_response: Option<SignalLossResponse>,
}

impl DeviceBuilder {
    #[must_use]
    pub fn new() -> Self {
        Self {
            real_position_in_meters: None,
            task: None,
            power_system: None,
            movement_system: None,
            trx_system: None,
            vulnerabilities: None,
            signal_loss_response: None,
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
    pub fn set_task(mut self, task: Task) -> Self {
        self.task = Some(task);
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
    pub fn set_signal_loss_response(
        mut self,
        signal_loss_response: SignalLossResponse
    ) -> Self {
        self.signal_loss_response = Some(signal_loss_response);
        self
    }
   
    #[must_use]
    pub fn build(self) -> Device {
        Device::new(
            generate_device_id(),
            self.real_position_in_meters.unwrap_or_default(),
            self.task.unwrap_or_default(),
            self.power_system.unwrap_or_default(),
            self.movement_system.unwrap_or_default(),
            self.trx_system.unwrap_or_default(),
            self.vulnerabilities.unwrap_or_default().as_ref(),
            self.signal_loss_response.unwrap_or_default(),
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
    task: Task,
    power_system: PowerSystem,
    movement_system: MovementSystem,
    trx_system: TRXSystem,
    // TODO HashMap<MalwareId, InfectionState>
    infection_states: MalwareToStateMap,
    signal_loss_response: SignalLossResponse,
}

impl Device {
    #[must_use]
    pub fn new(
        id: DeviceId,
        real_position_in_meters: Point3D,
        task: Task,
        power_system: PowerSystem,
        movement_system: MovementSystem,
        trx_system: TRXSystem,
        vulnerabilities: &[Malware],
        signal_loss_response: SignalLossResponse,
    ) -> Self {
        let infection_states = vulnerabilities
            .iter()
            .map(|malware| (*malware, InfectionState::Vulnerable))
            .collect();

        Self {
            id,
            real_position_in_meters,
            task,
            power_system,
            movement_system,
            trx_system,
            infection_states,
            signal_loss_response,
        }
    }

    #[must_use]
    pub fn id(&self) -> DeviceId {
        self.id
    }
    
    #[must_use]
    pub fn task(&self) -> &Task {
        &self.task
    }
    
    #[must_use]
    pub fn gps_position(&self) -> &Point3D {
        self.movement_system.position()
    }
    
    #[must_use]
    pub fn signal_loss_response(&self) -> &SignalLossResponse {
        &self.signal_loss_response
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

        self.trx_system.receive_message(message, frequency) 
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
            MessageType::SetTask(task)     => {
                self.task = *task;
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
        
        // TODO generalize for any frequency
        if self.receives_signal(WIFI_2_4GHZ_FREQUENCY) {
            self.process_task();
        // TODO add autonomous work as a parameter
        } else {
            self.handle_signal_loss();
        }

        self.update_real_position()?;

        Ok(())
    }

    fn try_consume_power(
        &mut self, 
        power: PowerUnit
    ) -> Result<(), PowerSystemError> {
        self.power_system
            .try_consume_power(power)
            .inspect_err(|_| self.selfdestruction())
    }

    fn process_task(&mut self) {
        let gps_is_connected = self.receives_signal(GPS_L1_FREQUENCY); 

        match self.task {
            Task::Attack(destination) 
                | Task::Reconnect(destination)
                | Task::Reposition(destination)
            if gps_is_connected  => {
                self.movement_system.set_direction(destination);
                self.try_reach_task();
            },
            Task::Attack(_) 
                | Task::Reposition(_) 
                | Task::Reconnect(_) => {
                self.set_horizontal_velocity();
            },
            Task::Undefined          => ()
        }
    }
    
    fn set_horizontal_velocity(&mut self) {
        let mut velocity = *self.movement_system.velocity();

        velocity.initial_point.z = 0.0;
        velocity.terminal_point.z = 0.0;
        velocity.scale_to(self.movement_system.max_speed());

        self.movement_system.set_velocity(velocity);
    }

    fn handle_signal_loss(&mut self) {
        match self.signal_loss_response {
            SignalLossResponse::Ascend                   => {
                let mut upward_point = self.real_position_in_meters;
                upward_point.z += 1.0;

                self.movement_system.set_direction(upward_point);
                self.task = Task::Reconnect(upward_point);
            },
            SignalLossResponse::Hover                    => {
                self.task = Task::Reconnect(self.real_position_in_meters);
                self.process_task();
            },
            SignalLossResponse::Ignore                   =>
                (),
            SignalLossResponse::ReturnToHome(home_point) => {
                self.task = Task::Reconnect(home_point);
                self.process_task();
            },
            SignalLossResponse::Shutdown                 =>
                self.selfdestruction(),
        }
    }

    fn update_real_position(&mut self) -> Result<(), DeviceError> {
        if self.movement_system.is_disabled() {
            return Err(DeviceError::DisabledSystem);
        }

        self.try_consume_power(MOVEMENT_POWER_CONSUMPTION)?;
        
        self.real_position_in_meters = equation_of_motion_3d(
            &self.real_position_in_meters,
            &self.movement_system.velocity().displacement(),
            millis_to_secs(ITERATION_TIME),
        );
        
        Ok(())
    }

    // Device can check if it has reached the task only if it knows
    // its current position (if it has GPS connection).
    fn try_reach_task(&mut self) {
        match self.task {
            Task::Attack(destination) if 
                self.at_destination(&destination) => 
                self.selfdestruction(),
            Task::Reposition(destination) if 
                self.at_destination(&destination) => 
                self.task = Task::Undefined,
            _ => (),
        }
    }

    fn at_destination(&self, destination: &Point3D) -> bool {
        self.distance_to(destination) <= DESTINATION_RADIUS 
    }

    fn selfdestruction(&mut self) {
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

impl Default for Device {
    fn default() -> Self {
        Self {
            id: generate_device_id(),
            real_position_in_meters: Point3D::default(),
            task: Task::default(),
            power_system: PowerSystem::default(),
            movement_system: MovementSystem::default(),
            trx_system: TRXSystem::default(),
            infection_states: MalwareToStateMap::default(),
            signal_loss_response: SignalLossResponse::default(),
        }
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
    const DEVICE_MAX_POWER: PowerUnit    = 10_000;
    

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
        ).unwrap_or_else(|error| panic!("{}", error))
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

    fn drone_green_rx_system(frequency: Megahertz) -> TRXSystem {
        TRXSystem::Color(drone_green_rx_module(frequency))
    }

    fn device_is_destructed(device: &Device) -> bool {
        device.power_system == PowerSystem::default()
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
        let task  = Task::Attack(Point3D::new(5.0, 5.0, 5.0));
        let power = PASSIVE_POWER_CONSUMPTION + MOVEMENT_POWER_CONSUMPTION;
        
        let power_system    = PowerSystem::build(
            power, 
            power,
        ).unwrap_or_else(|error| panic!("{}", error));
        let movement_system = MovementSystem::build(25.0)
            .unwrap_or_else(|error| panic!("{}", error));
        let trx_system      = drone_green_rx_system(WIFI_2_4GHZ_FREQUENCY);

        let mut device = DeviceBuilder::new()
            .set_task(task)
            .set_power_system(power_system.clone())
            .set_movement_system(movement_system.clone())
            .set_trx_system(trx_system.clone())
            .build();

        assert_eq!(device.task, task);
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
    fn ascending_on_signal_loss() {
        let signal_loss_response = SignalLossResponse::Ascend;
        let destination_point = Point3D::new(5.0, 5.0, 5.0);
        let task = Task::Reposition(destination_point);
        
        let mut device_without_signal = DeviceBuilder::new()
            .set_task(task)
            .set_power_system(device_power_system())
            .set_movement_system(drone_movement_system())
            .set_trx_system(drone_green_rx_system(GPS_L1_FREQUENCY))
            .set_signal_loss_response(signal_loss_response)
            .build();
        let original_position = device_without_signal.real_position_in_meters;

        let many_iterations = 10;
        for _ in 0..many_iterations {
            let gps_message = Message::new(
                UNKNOWN_ID, 
                device_without_signal.id(), 
                0,
                MessageType::GPS(*device_without_signal.position())
            );
            let _ = device_without_signal.receive_and_process_message(
                &gps_message,
                GPS_L1_FREQUENCY, 
            );
            
            let _ = device_without_signal.update_state();
        }

        assert_eq!(
            device_without_signal.real_position_in_meters.x,
            original_position.x
        );
        assert_eq!(
            device_without_signal.real_position_in_meters.y,
            original_position.y
        );
        assert!(device_without_signal.real_position_in_meters.z > 0.0);
    }
    
    #[test]
    fn hovering_on_signal_loss() {
        let signal_loss_response = SignalLossResponse::Hover;
        let destination_point = Point3D::new(5.0, 5.0, 5.0);
        let task = Task::Reposition(destination_point);
        
        let mut device_without_signal = DeviceBuilder::new()
            .set_task(task)
            .set_power_system(device_power_system())
            .set_movement_system(drone_movement_system())
            .set_trx_system(drone_green_rx_system(GPS_L1_FREQUENCY))
            .set_signal_loss_response(signal_loss_response)
            .build();
        let original_position = device_without_signal.real_position_in_meters;

        let many_iterations = 500;
        for _ in 0..many_iterations {
            let gps_message = Message::new(
                UNKNOWN_ID, 
                device_without_signal.id(), 
                0,
                MessageType::GPS(*device_without_signal.position())
            );
            let _ = device_without_signal.receive_and_process_message(
                &gps_message,
                GPS_L1_FREQUENCY, 
            );

            let _ = device_without_signal.update_state();
        }

        assert_eq!(
            device_without_signal.real_position_in_meters.x,
            original_position.x
        );
        assert_eq!(
            device_without_signal.real_position_in_meters.y,
            original_position.y
        );
        assert_eq!(
            device_without_signal.real_position_in_meters.z,
            original_position.z
        );
    }
    
    #[test]
    fn returning_to_home_on_signal_loss() {
        let home_point = Point3D::new(
            -MAX_DRONE_SPEED / 3.0, 
            -MAX_DRONE_SPEED / 3.0, 
            -MAX_DRONE_SPEED / 3.0
        );
        let signal_loss_response = SignalLossResponse::ReturnToHome(home_point);
        let destination_point = Point3D::new(
            MAX_DRONE_SPEED / 3.0, 
            MAX_DRONE_SPEED / 3.0, 
            MAX_DRONE_SPEED / 3.0
        );
        let task = Task::Reposition(destination_point);
        
        let mut device_without_signal = DeviceBuilder::new()
            .set_task(task)
            .set_power_system(device_power_system())
            .set_movement_system(drone_movement_system())
            .set_trx_system(drone_green_rx_system(GPS_L1_FREQUENCY))
            .set_signal_loss_response(signal_loss_response)
            .build();

        for _ in (0..500).step_by(ITERATION_TIME as usize) {
            let gps_message = Message::new(
                UNKNOWN_ID, 
                device_without_signal.id(), 
                0,
                MessageType::GPS(*device_without_signal.position())
            );
            assert!(
                device_without_signal.receive_and_process_message(
                    &gps_message,
                    GPS_L1_FREQUENCY, 
                ).is_ok()
            );
            
            let _ = device_without_signal.update_state();
        }

        assert!(device_without_signal.at_destination(&home_point));
    }
    
    #[test]
    fn shutting_down_on_signal_loss() {
        let signal_loss_response = SignalLossResponse::Shutdown;
        let destination_point = Point3D::new(5.0, 5.0, 5.0);
        let task = Task::Reposition(destination_point);
        
        let mut device_without_signal = DeviceBuilder::new()
            .set_task(task)
            .set_power_system(device_power_system())
            .set_signal_loss_response(signal_loss_response)
            .build();

        let many_iterations = 500;
        for _ in 0..many_iterations {
            let gps_message = Message::new(
                UNKNOWN_ID, 
                device_without_signal.id(), 
                0,
                MessageType::GPS(*device_without_signal.position())
            );
            let _ = device_without_signal.receive_and_process_message(
                &gps_message,
                GPS_L1_FREQUENCY, 
            );

            let _ = device_without_signal.update_state();
        }

        assert!(device_is_destructed(&device_without_signal));
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

        for _ in (0..1000).step_by(ITERATION_TIME as usize) {
            let _ = device.update_state();

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
        let task              = Task::Reposition(destination_point);
        
        let mut device_without_gps = DeviceBuilder::new()
            .set_task(task)
            .set_power_system(device_power_system())
            .set_movement_system(drone_movement_system())
            .build();

        for _ in (0..1000).step_by(ITERATION_TIME as usize) {
            let _ = device_without_gps.update_state();
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
        let task              = Task::Reposition(destination_point);
        let max_rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL),
            (WIFI_2_4GHZ_FREQUENCY, GREEN_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL),
            (WIFI_2_4GHZ_FREQUENCY, GREEN_SIGNAL_LEVEL)
        ]);
        let rx_module = TRXModule::build(
            max_rx_signal_levels,
            rx_signal_levels
        ).unwrap();
        let trx_system = TRXSystem::Strength { 
            tx_module: TRXModule::default(), 
            rx_module 
        };
        
        let mut device = DeviceBuilder::new()
            .set_task(task)
            .set_power_system(device_power_system())
            .set_movement_system(drone_movement_system())
            .set_trx_system(trx_system)
            .build();
            
        for _ in (0..1000).step_by(ITERATION_TIME as usize) {
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

        assert!(device.at_destination(&destination_point));
    }

    #[test]
    fn device_selfdestruction() {
        let task            = Task::Attack(Point3D::new(5.0, 5.0, 5.0));
        let power_system    = device_power_system();
        let movement_system = MovementSystem::build(25.0)
            .unwrap_or_else(|error| panic!("{}", error));
        let trx_system      = drone_green_rx_system(WIFI_2_4GHZ_FREQUENCY);

        let mut device = DeviceBuilder::new()
            .set_task(task)
            .set_power_system(power_system.clone())
            .set_trx_system(trx_system.clone())
            .set_movement_system(movement_system.clone())
            .build();

        assert_eq!(device.task, task);
        assert_eq!(device.power_system, power_system);
        assert_eq!(device.trx_system, trx_system);
        assert_eq!(device.movement_system, movement_system);

        device.selfdestruction();

        assert!(device_is_destructed(&device));
    }

    #[test]
    fn receive_and_process_correct_set_task_message() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        let task      = Task::Attack(Point3D::new(5.0, 0.0, 0.0));

        let mut device = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(drone_green_rx_system(frequency))
            .build();
            
        let message = Message::new(
            UNKNOWN_ID,
            device.id(),
            0, 
            MessageType::SetTask(task)
        );

        assert!(
            device.receive_and_process_message(&message, frequency)
                .is_ok()
        );
        assert_eq!(task, device.task);
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
        let task      = Task::Attack(Point3D::new(5.0, 0.0, 0.0));
        
        let mut device = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(drone_green_rx_system(frequency))
            .build();    
        
        let message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            0, 
            MessageType::SetTask(task)
        );

        assert!(
            device.receive_and_process_message(&message, frequency)
                .is_ok()
        );
        assert_eq!(task, device.task);
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
            MessageType::SetTask(Task::Undefined)
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
