use std::{
    collections::{
        HashMap, 
        hash_map::{Iter, IterMut, Keys, Values, ValuesMut}
    },
    hash::{Hash, Hasher},
    sync::atomic::{AtomicUsize, Ordering}
};

use crate::communication::{signal::*, message::*};
use crate::mathphysics::*;

use self::networkmodel::*;
use self::modules::*;


pub mod networkmodel;
pub mod modules;


pub type DeviceId = usize;


pub const DESTINATION_RADIUS: Meter = 5.0;

pub const STEP_DURATION: Millisecond = 50;


static FREE_DEVICE_ID: AtomicUsize = AtomicUsize::new(1);


const MAX_DRONE_SPEED: MeterPerSecond  = 25.0;

const UNKNOWN_ID: DeviceId = 0;


pub trait Device: Position {
    fn id(&self) -> DeviceId;
}


fn generate_device_id() -> DeviceId {
    FREE_DEVICE_ID.fetch_add(1, Ordering::SeqCst)
}

pub struct CommandCenterBuilder {
    id: DeviceId,
    position: Option<Point3D>,
    tx_module: Option<TRXModule>,
    rx_module: Option<TRXModule>
}

impl CommandCenterBuilder {
    pub fn new() -> Self {
        Self {
            id: generate_device_id(),
            position: None,
            tx_module: None,
            rx_module: None
        }
    }
    
    pub fn set_position(mut self, position_in_meters: Point3D) -> Self {
        self.position = Some(position_in_meters);
        self
    }

    pub fn set_tx_module(mut self, tx_module: TRXModule) -> Self {
        self.tx_module = Some(tx_module);
        self
    }
    
    pub fn set_rx_module(mut self, rx_module: TRXModule) -> Self {
        self.rx_module = Some(rx_module);
        self
    }

    pub fn set_trx_module(mut self, mut trx_system: TRXSystem) -> Self {
        self.tx_module = Some(std::mem::take(trx_system.tx_module_mut()));
        self.rx_module = Some(std::mem::take(trx_system.rx_module_mut()));
        self
    }

    pub fn build(self) -> CommandCenter {
        let trx_system = TRXSystem::new(
            self.tx_module.unwrap_or_default(),
            self.rx_module.unwrap_or_default()
        );

        CommandCenter::new(
            self.id,
            self.position.unwrap_or_default(),
            trx_system 
        )
    }
}


#[derive(Clone)]
pub struct CommandCenter {
    id: DeviceId,
    position_in_meters: Point3D,
    trx_system: TRXSystem
}

impl CommandCenter {
    pub fn new(
        id: DeviceId, 
        position_in_meters: Point3D,
        trx_system: TRXSystem
    ) -> Self {
        Self {
            id,
            position_in_meters,
            trx_system
        }
    }
}

impl Default for CommandCenter {
    fn default() -> Self {
        Self {
            id: generate_device_id(),
            position_in_meters: Point3D::default(),
            trx_system: TRXSystem::default()
        }
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
    fn position(&self) -> &Point3D {
        &self.position_in_meters
    }
}

impl Device for CommandCenter {
    fn id(&self) -> DeviceId {
        self.id
    }
}

impl Transmitter for CommandCenter {
    fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        self.trx_system.tx_signal_levels()
    }
    
    fn tx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.trx_system.tx_signal_level(signal_type)
    }

    fn area(&self, signal_type: &SignalType) -> SignalArea {
        self.trx_system.area(signal_type)
    }

    fn connection_distance<P: Position>(
        &self, 
        object: &P, 
        signal_type: &SignalType
    ) -> Option<Meter> {
        self.trx_system.connection_distance(
            self.distance_to(object), 
            signal_type
        )
    }
    
    fn propagated_signal_level_at<R: Receiver>(
        &self,
        receiver: &R,
        signal_type: &SignalType
    ) -> SignalLevel {
        let distance_to_rx = self.distance_to(receiver);

        self.trx_system.tx_signal_level_at(distance_to_rx, signal_type)
    }

    fn propagate_signal_level<R: Receiver>(
        &self,
        receiver: &mut R,
        signal_type: SignalType
    ) {
        let propagated_signal_level_at_rx = self.propagated_signal_level_at(
            receiver, 
            &signal_type
        );

        receiver.receive_signal(signal_type, propagated_signal_level_at_rx);
    }
}

impl Receiver for CommandCenter {
    fn rx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        self.trx_system.rx_signal_levels()
    }
    
    fn rx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.trx_system.rx_signal_level(signal_type)
    }

    fn receives_signal(&self, signal_type: &SignalType) -> bool {
        self.trx_system.receives_signal_level(signal_type)
    }
    
    fn receive_signal(
        &mut self, 
        signal_type: SignalType,
        signal_level: SignalLevel
    ) {
        self.trx_system.receive_signal_level(signal_type, signal_level); 
    }

    fn receive_message(
        &mut self,
        signal_type: SignalType,
        message: &Message
    ) -> Result<(), ()> {
        self.trx_system.receive_message(signal_type, message) 
    }

    fn signal_level_suppression(
        &mut self,
        suppressor_signal_type: SignalType,
        suppressor_signal_level: SignalLevel
    ) {
        self.trx_system.suppress_signal_level(
            suppressor_signal_type, 
            suppressor_signal_level
        );
    }
}

impl Transceiver for CommandCenter {}


pub struct DroneBuilder<'a> {
    drone_id: DeviceId,
    command_center: Option<&'a CommandCenter>,
    global_position_in_meters: Option<Point3D>,
    destination_in_meters: Option<Point3D>,
    goal: Option<Goal>,
    tx_module: Option<TRXModule>,
    rx_module: Option<TRXModule>
}

impl<'a> DroneBuilder<'a> {
    pub fn new() -> Self {
        Self {
            drone_id: generate_device_id(),
            command_center: None,
            global_position_in_meters: None,
            destination_in_meters: None,
            goal: None,
            tx_module: None,
            rx_module: None
        }
    }

    pub fn set_command_center(
        mut self, 
        command_center: &'a CommandCenter
    ) -> Self {
        self.command_center = Some(command_center);
        self
    }

    pub fn set_global_position(
        mut self, 
        global_position_in_meters: Point3D
    ) -> Self {
        self.global_position_in_meters = Some(global_position_in_meters);
        self
    }
    
    pub fn set_destination(
        mut self, 
        destination_in_meters: Point3D
    ) -> Self {
        self.destination_in_meters = Some(destination_in_meters);
        self
    }
    
    pub fn set_goal(mut self, goal: Goal) -> Self {
        self.goal = Some(goal);
        self
    }
    
    pub fn set_tx_module(mut self, tx_module: TRXModule) -> Self {
        self.tx_module = Some(tx_module);
        self
    }
    
    pub fn set_rx_module(mut self, rx_module: TRXModule) -> Self {
        self.rx_module = Some(rx_module);
        self
    }
    
    pub fn set_trx_module(mut self, mut trx_system: TRXSystem) -> Self {
        self.tx_module = Some(std::mem::take(trx_system.tx_module_mut()));
        self.rx_module = Some(std::mem::take(trx_system.rx_module_mut()));
        self
    }
   
    pub fn build(self) -> Drone {
        let trx_system = TRXSystem::new(
            self.tx_module.unwrap_or_default(),
            self.rx_module.unwrap_or_default()
        );

        let mut drone = Drone::new(
            self.drone_id,
            UNKNOWN_ID,
            self.global_position_in_meters.unwrap_or_default(),
            self.destination_in_meters.unwrap_or_default(),
            self.goal.unwrap_or_default(),
            trx_system
        );

        if let Some(cc) = self.command_center {
            drone.connect_command_center(cc);
        }

        drone
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
    trx_system: TRXSystem
    // TODO infection_state
}

impl Drone {
    pub fn new(
        id: DeviceId,
        command_center_id: DeviceId,
        global_position_in_meters: Point3D,
        destination_in_meters: Point3D,
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
            destination_in_meters,
            goal,
            trx_system
        }
    }

    fn command_center_id(&self) -> DeviceId {
        self.command_center_id
    }

    fn connect_command_center(&mut self, command_center: &CommandCenter) {
        self.command_center_id = command_center.id();

        command_center.propagate_signal_level(
            self,
            SignalType::Control
        );
    }
    
    fn process_message(&mut self, signal_type: SignalType, message: &Message) {
        if self
            .receive_message(signal_type, message)
            .is_err()
        {
            return;   
        }

        match message.message_type() {
            MessageType::SetDestination(destination, goal) => {
                self.destination_in_meters = *destination;
                self.goal = *goal;
            },
            MessageType::ChangeGoal(goal) => {
                self.goal = *goal;
            }
        };
    }

    fn update_state(&mut self) {
        self.update_position();
    }

    fn update_position(&mut self) {
        if self.receives_signal(&SignalType::GPS) {
            self.update_current_position();
            self.update_movement_direction();
        } else {
            self.update_movement_direction_without_gps();
        }
        
        self.update_global_position();
        
        let _ = self.connect_gps();
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
        if self.reached_destination() {
            let _ = self.try_attack();
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
            .set_rx_signal_level(SignalType::GPS, GREEN_SIGNAL_LEVEL);
    }

    fn reached_destination(&self) -> bool {
        let distance = self.distance_to(&self.destination_in_meters);
        
        distance <= DESTINATION_RADIUS
    }

    fn try_attack(&mut self) -> Result<(), ()> {
        if matches!(self.goal, Goal::Attack) { 
            self.selfdestruction();
            return Ok(());
        }

        Err(())
    }

    fn selfdestruction(&mut self) {
        self.trx_system
            .set_rx_signal_level(SignalType::Control, NO_SIGNAL_LEVEL);
    }
  
    // This function was introduced for convenience to change signal levels in
    // IdToDroneMap::set_rx_signal_levels().
    // It was left private to protect rules of signal receiving in
    // Receiver::receive_signal().
    fn set_rx_signal_level(
        &mut self, 
        signal_type: SignalType,
        signal_level: SignalLevel
    ) {
        self.trx_system.set_rx_signal_level(signal_type, signal_level);
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
    fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        self.trx_system.tx_signal_levels()
    }
    
    fn tx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.trx_system.tx_signal_level(signal_type)
    }

    fn area(&self, signal_type: &SignalType) -> SignalArea {
        self.trx_system.area(signal_type)
    }

    fn connection_distance<P: Position>(
        &self, 
        object: &P, 
        signal_type: &SignalType
    ) -> Option<Meter> {
        self.trx_system.connection_distance(
            self.distance_to(object), 
            signal_type
        )
    }
    
    fn propagated_signal_level_at<R: Receiver>(
        &self,
        receiver: &R,
        signal_type: &SignalType
    ) -> SignalLevel {
        let distance_to_rx = self.distance_to(receiver);

        self.trx_system.tx_signal_level_at(distance_to_rx, signal_type)
    }

    fn propagate_signal_level<R: Receiver>(
        &self,
        receiver: &mut R,
        signal_type: SignalType
    ) {
        let propagated_signal_level_at_rx = self.propagated_signal_level_at(
            receiver, 
            &signal_type
        );

        receiver.receive_signal(signal_type, propagated_signal_level_at_rx);
    }
}

impl Receiver for Drone {
    fn rx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        self.trx_system.rx_signal_levels()
    }

    fn rx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.trx_system.rx_signal_level(signal_type)
    }

    fn receives_signal(&self, signal_type: &SignalType) -> bool {
        self.trx_system.receives_signal_level(signal_type)
    }
    
    fn receive_signal(
        &mut self, 
        signal_type: SignalType,
        signal_level: SignalLevel
    ) {
        self.trx_system.receive_signal_level(signal_type, signal_level); 
    }

    fn receive_message(
        &mut self,
        signal_type: SignalType,
        message: &Message
    ) -> Result<(), ()> {
        self.trx_system.receive_message(signal_type, message) 
    }
    
    fn signal_level_suppression(
        &mut self,
        suppressor_signal_type: SignalType,
        suppressor_signal_level: SignalLevel
    ) {
        self.trx_system.suppress_signal_level(
            suppressor_signal_type, 
            suppressor_signal_level
        );
    }
}

impl Transceiver for Drone {}


#[derive(Clone, Debug)]
struct IdToDroneMap(HashMap<DeviceId, Drone>);

impl IdToDroneMap {
    fn get(&self, drone_id: &DeviceId) -> Option<&Drone> {
        self.0.get(drone_id)
    }

    fn ids(&self) -> Keys<'_, DeviceId, Drone> {
        self.0.keys()
    }

    fn drones(&self) -> Values<'_, DeviceId, Drone> {
        self.0.values()
    }
    
    fn drones_mut(&mut self) -> ValuesMut<'_, DeviceId, Drone> {
        self.0.values_mut()
    }

    fn iter(&self) -> Iter<'_, DeviceId, Drone> {
        self.0.iter()
    }

    fn iter_mut(&mut self) -> IterMut<'_, DeviceId, Drone> {
        self.0.iter_mut()
    }

    fn len(&self) -> usize {
        self.0.len()
    }

    fn all_rx_signal_levels(
        &self, 
        signal_type: &SignalType
    ) -> HashMap<DeviceId, SignalLevel> {
        self
            .drones()
            .map(|drone| (
                drone.id,
                *drone.rx_signal_level(signal_type)
            ))
            .collect()
    }

    fn connect_command_center(&mut self, command_center: &CommandCenter) {
        self
            .drones_mut()
            .for_each(|drone| drone.connect_command_center(command_center));
    }

    fn update_states(&mut self) {
        self
            .drones_mut()
            .for_each(Drone::update_state);
    }

    fn remove_uncontrolled_drones(&mut self) { 
        self.0.retain(|_, drone| 
            drone.receives_signal(&SignalType::Control)
        );
    }

    fn set_rx_signal_levels(
        &mut self,
        signal_levels: HashMap<DeviceId, SignalLevel>,
        signal_type: &SignalType
    ) {
        for (id, drone) in self.iter_mut() {
            let signal_level = match signal_levels.get(&id) {
                Some(signal_level) => signal_level,
                None => continue
            };

            drone.set_rx_signal_level(*signal_type, *signal_level);
        }
    }

    fn clear_rx_signal_levels(&mut self, signal_type: &SignalType) {
        self
            .drones_mut()
            .for_each(|drone|
                drone.set_rx_signal_level(*signal_type, NO_SIGNAL_LEVEL)
            );
    }
}

impl From<&[Drone]> for IdToDroneMap {
    fn from(drones: &[Drone]) -> Self {
       Self(
           drones 
                .iter()
                .map(|drone| (drone.id, drone.clone()))
                .collect()
       )
    }
}

impl<const N: usize> From<[Drone; N]> for IdToDroneMap {
    fn from(drones: [Drone; N]) -> Self {
       Self(
           drones 
                .iter()
                .map(|drone| (drone.id, drone.clone()))
                .collect()
       )
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    
    const CC_TX_CONTROL_RADIUS: f32    = 300.0;
    const DRONE_TX_CONTROL_RADIUS: f32 = 10.0;
    
    fn cc_tx_module() -> TRXModule {
        let max_tx_signal_levels = HashMap::from([(
            SignalType::Control, 
            SignalLevel::from(SignalArea::build(CC_TX_CONTROL_RADIUS).unwrap())
        )]);

        TRXModule::build(
            AntennaType::Strength,
            max_tx_signal_levels.clone(), 
            max_tx_signal_levels
        ).unwrap()
    }

    fn drone_tx_module() -> TRXModule {
        let max_tx_signal_levels = HashMap::from([(
            SignalType::Control, 
            SignalLevel::from(
                SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap()
            )
        )]);

        TRXModule::build(
            AntennaType::Strength,
            max_tx_signal_levels.clone(),
            max_tx_signal_levels
        ).unwrap()
    }
    
    fn drone_rx_module() -> TRXModule {
        let max_rx_signal_levels = HashMap::from([
            (SignalType::GPS, GREEN_SIGNAL_LEVEL),
            (SignalType::Control, GREEN_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (SignalType::GPS, GREEN_SIGNAL_LEVEL),
            (SignalType::Control, NO_SIGNAL_LEVEL)
        ]);

        TRXModule::build(
            AntennaType::Strength,
            max_rx_signal_levels,
            rx_signal_levels
        ).unwrap()
    }
    
    #[test]
    fn cc_connection_to_drone() {
        let command_center = CommandCenterBuilder::new()
            .set_tx_module(cc_tx_module())
            .build();
        let mut drone = DroneBuilder::new()
            .set_tx_module(drone_tx_module())
            .set_rx_module(drone_rx_module())
            .build();

        assert!(drone.command_center_id() == UNKNOWN_ID);
        assert!(
            &drone.trx_system
                .rx_signal_level(&SignalType::Control)
                .is_black()
        );

        drone.connect_command_center(&command_center);
        
        assert!(drone.command_center_id() == command_center.id());
        assert!(
            drone.trx_system
                .rx_signal_level(&SignalType::Control)
                .is_green()
        );
    }

    #[test]
    fn drone_movement_without_gps() {
        let destination_point = Point3D::new(MAX_DRONE_SPEED, 0.0, 0.0);
        
        let mut drone_without_gps = DroneBuilder::new()
            .set_destination(destination_point)
            .build();

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            drone_without_gps.update_state();
        }

        assert_eq!(
            drone_without_gps.position_in_meters, 
            Point3D::default()
        );
        assert_eq!(
            drone_without_gps.global_position_in_meters, 
            destination_point
        );
    }

    #[test]
    fn drone_reach_destination() {
        let destination_point = Point3D::new(MAX_DRONE_SPEED, 0.0, 0.0);
        
        let mut drone = DroneBuilder::new()
            .set_destination(destination_point)
            .build();

        for _ in (0..1000).step_by(STEP_DURATION as usize) {
            drone.update_state();
        }

        assert!(drone.reached_destination());
    }
}
