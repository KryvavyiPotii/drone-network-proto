use std::ops;

use derive_more::{Add, Mul};
use impl_ops::{
    _impl_binary_op_borrowed_borrowed, _impl_binary_op_borrowed_owned, 
    _impl_binary_op_internal, _impl_binary_op_owned_borrowed, 
    _impl_binary_op_owned_owned, _parse_binary_op, impl_op, impl_op_ex
};
use thiserror::Error;

use crate::device::{DeviceId, networkmodel::ConnectionGraph}; 
use crate::mathphysics::{Megahertz, Millisecond, Point3D};

use super::signal::GREEN_SIGNAL_STRENGTH_VALUE;


pub const CHANGE_GOAL_COST: MessageCost     = MessageCost(
    GREEN_SIGNAL_STRENGTH_VALUE / 100.0 
);
pub const SET_DESTINATION_COST: MessageCost = MessageCost(
    GREEN_SIGNAL_STRENGTH_VALUE / 50.0 
);
pub const INFECTION_COST: MessageCost       = MessageCost(
    GREEN_SIGNAL_STRENGTH_VALUE / 50.0
);


fn define_message_execution_cost(message_type: &MessageType) -> MessageCost {
    match message_type {
        MessageType::ChangeGoal(_) => CHANGE_GOAL_COST,
        MessageType::Infection => INFECTION_COST,
        MessageType::SetDestination(..) =>  SET_DESTINATION_COST,
    }
}


#[derive(
    Clone, Copy, Debug, Default,
    Add, Mul, PartialEq, PartialOrd
)]
pub struct MessageCost(f32);

impl MessageCost {
    #[must_use]
    pub fn new(value: f32) -> Self {
        let message_cost = if value <= 0.0 {
            0.0
        } else {
            value
        };

        Self(message_cost)
    }

    #[must_use]
    pub fn value(&self) -> f32 {
        self.0
    }
}

impl_op_ex!(
    - |a: &MessageCost, b: &MessageCost| -> MessageCost { 
        MessageCost(a.0 - b.0) 
    }
);
impl_op_ex!(
    - |a: &MessageCost, b: &f32| -> MessageCost { 
        MessageCost(a.0 - b) 
    }
);
impl_op_ex!(
    / |a: &MessageCost, b: &MessageCost| -> MessageCost { 
        MessageCost(a.0 / b.0) 
    }
);
impl_op_ex!(
    / |a: &MessageCost, b: &f32| -> MessageCost { 
        MessageCost(a.0 / b) 
    }
);


#[derive(Clone, Copy, Debug, Default)]
pub enum Goal {
    #[default]
    Reposition,
    Attack,
}


#[derive(Clone, Copy, Debug)]
pub enum MessageType {
    ChangeGoal(Goal),
    Infection,
    SetDestination(Point3D),
}


#[derive(Clone, Copy, Debug)]
pub enum MessageState {
    Waiting,
    InProgress,
    Finished,
}


#[derive(Error, Debug)]
pub enum MessagePreprocessError {
    #[error("Message is already preprocessed")]
    AlreadyPreprocessed,
    #[error("Message execution time is set to be later")]
    TooEarly,
    #[error("Message source device ID is unknown")]
    UnknownSource
}


#[derive(Clone, Copy, Debug)]
pub struct Message {
    source_id: DeviceId,
    destination_id: DeviceId,
    execution_time: Millisecond,
    execution_cost: MessageCost,
    message_type: MessageType,
    message_state: MessageState
}

impl Message {
    #[must_use]
    pub fn new(
        source_id: DeviceId,
        destination_id: DeviceId,
        execution_time: Millisecond,
        message_type: MessageType
    ) -> Self {
        Self { 
            source_id,
            destination_id,
            execution_time,
            execution_cost: define_message_execution_cost(&message_type),
            message_type,
            message_state: MessageState::Waiting,
        }
    }

    #[must_use]
    pub fn source_id(&self) -> DeviceId {
        self.source_id
    }
    
    #[must_use]
    pub fn destination_id(&self) -> DeviceId {
        self.destination_id
    }

    #[must_use]
    pub fn time(&self) -> Millisecond {
        self.execution_time
    }

    #[must_use]
    pub fn cost(&self) -> MessageCost {
        self.execution_cost
    }

    #[must_use]
    pub fn message_type(&self) -> &MessageType {
        &self.message_type
    }

    #[must_use]
    pub fn message_state(&self) -> &MessageState {
        &self.message_state
    }

    pub fn process(&mut self) {
        if let MessageState::Waiting = self.message_state {
            self.message_state = MessageState::InProgress;
        }
    }

    pub fn finish(&mut self) {
        match self.message_state {
            MessageState::Waiting | MessageState::InProgress => 
                self.message_state = MessageState::Finished,
            MessageState::Finished => ()
        }
    }

    #[must_use]
    pub fn is_in_progress(&self) -> bool {
        matches!(self.message_state, MessageState::InProgress)
    }

    #[must_use]
    pub fn is_finished(&self) -> bool {
        matches!(self.message_state, MessageState::Finished)
    }

    /// # Errors
    ///
    /// Will return `Err` if execution time is greater than current time or
    /// message is already in progress or `ConnectionGraph` does not contain
    /// source device ID in message.
    pub fn try_preprocess(
        &mut self,
        current_time: Millisecond,
        connections: &ConnectionGraph,
    ) -> Result<(), MessagePreprocessError> {
        if self.is_in_progress() {
            return Err(MessagePreprocessError::AlreadyPreprocessed);
        }
        if current_time < self.time() {
            return Err(MessagePreprocessError::TooEarly);
        }
        if !connections.contains_device(self.source_id()) {
            return Err(MessagePreprocessError::UnknownSource);
        }

        self.process();

        Ok(())
    }
}


#[derive(Clone, Debug, Default)]
pub struct MessageQueue(Vec<(Megahertz, Message)>);

impl MessageQueue {
    #[must_use]
    pub fn new() -> Self {
        Self(Vec::new())
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }
    
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
    
    pub fn iter(&self) -> std::slice::Iter<'_, (Megahertz, Message)> {
        self.0.iter()
    }

    pub fn iter_mut(
        &mut self
    ) -> std::slice::IterMut<'_, (Megahertz, Message)> {
        self.0.iter_mut()
    }
    
    pub fn add_message(&mut self, frequency: Megahertz, message: Message) {
        self.0.push((frequency, message));
        self.0.sort_by_key(|(_, message)| message.time());
    }

    pub fn remove_finished_messages(&mut self) {
        self.0.retain(|(_, message)| !message.is_finished());
    }
}

impl<'a> IntoIterator for &'a MessageQueue{
    type Item = &'a (Megahertz, Message);
    type IntoIter = std::slice::Iter<'a, (Megahertz, Message)>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter()
    }
}

impl<'a> IntoIterator for &'a mut MessageQueue{
    type Item = &'a mut (Megahertz, Message);
    type IntoIter = std::slice::IterMut<'a, (Megahertz, Message)>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter_mut()
    }
}

impl From<&[(Megahertz, Message)]> for MessageQueue {
    fn from(messages: &[(Megahertz, Message)]) -> Self {
        let mut message_queue = Self(messages.to_vec());

        message_queue.0.sort_by_key(|(_, message)| message.time());

        message_queue
    }
}

impl<const N: usize> From<[(Megahertz, Message); N]> for MessageQueue {
    fn from(messages: [(Megahertz, Message); N]) -> Self {
        let mut message_queue = Self(messages.to_vec());

        message_queue.0.sort_by_key(|(_, message)| message.time());

        message_queue
    }
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::communication::GREEN_SIGNAL_LEVEL;
    use crate::device::{
        CommandCenter, CommandCenterBuilder, Device, DeviceId, Drone,
        DroneBuilder, UNKNOWN_ID
    };
    use crate::device::modules::{TRXModule, TRXSystem};
    use crate::device::networkmodel::{IdToDroneMap, Topology};

    use super::*;


    const SOME_FREQUENCY: Megahertz = 2_000;


    fn message_vec() -> Vec<(Megahertz, Message)> {
        vec![
            (
                SOME_FREQUENCY,
                Message::new(
                    2, 
                    3, 
                    25, 
                    MessageType::ChangeGoal(Goal::Reposition)
                )
            ),
            (
                SOME_FREQUENCY,
                Message::new(
                    1, 
                    2, 
                    5, 
                    MessageType::ChangeGoal(Goal::Reposition)
                )
            ),
            (
                SOME_FREQUENCY,
                Message::new(
                    1, 
                    3, 
                    10, 
                    MessageType::ChangeGoal(Goal::Reposition)
                )
            )
        ]
    }

    fn simple_trx_system() -> TRXSystem {
        TRXSystem::Color(
            TRXModule::build(
                HashMap::from([(SOME_FREQUENCY, GREEN_SIGNAL_LEVEL)]),
                HashMap::from([(SOME_FREQUENCY, GREEN_SIGNAL_LEVEL)]),
            ).unwrap()
        )
    }

    fn simple_cc() -> CommandCenter {
        CommandCenterBuilder::new()
            .set_trx_system(simple_trx_system())
            .build()
    }
    
    fn simple_drone() -> Drone {
        DroneBuilder::new()
            .set_trx_system(simple_trx_system())
            .build()
    }

    fn connection_graph_with_cc_id() -> (ConnectionGraph, DeviceId) {
        let command_center = simple_cc();
        let cc_id = command_center.id();

        let drone_map = IdToDroneMap::from([simple_drone()]);

        let mut connections = ConnectionGraph::new();
        connections.update(
            &command_center, 
            &drone_map, 
            Topology::Star, 
            SOME_FREQUENCY
        );

        (connections, cc_id)
    }


    #[test]
    fn removing_finished_messages() {
        let mut messages = message_vec();

        messages[1].1.finish();

        let mut message_queue = MessageQueue::from(messages.as_slice());
        message_queue.remove_finished_messages();

        assert_eq!(message_queue.len(), 2);
        for (_, message) in message_queue.iter() {
            assert!(!message.is_finished());
        }
    }
   
    #[test]
    fn preprocessing_already_preprocessed_message() {
        let current_time = 10;
        let mut message = Message::new(
            UNKNOWN_ID,
            UNKNOWN_ID,
            current_time, 
            MessageType::ChangeGoal(Goal::Reposition)
        );
        message.process();

        assert!(
            matches!(
                message.try_preprocess(current_time, &ConnectionGraph::new()), 
                Err(MessagePreprocessError::AlreadyPreprocessed)
            )
        );
    }

    #[test]
    fn too_early_message_preprocessing() {
        let current_time = 12;
        let mut message = Message::new(
            UNKNOWN_ID,
            UNKNOWN_ID,
            50, 
            MessageType::ChangeGoal(Goal::Reposition)
        );

        assert!(
            matches!(
                message.try_preprocess(current_time, &ConnectionGraph::new()), 
                Err(MessagePreprocessError::TooEarly)
            )
        );
    }
    
    #[test]
    fn message_preprocessing_with_unknown_source() {
        let current_time = 12;
        let mut message = Message::new(
            UNKNOWN_ID,
            UNKNOWN_ID,
            current_time, 
            MessageType::ChangeGoal(Goal::Reposition)
        );

        let (connections, _) = connection_graph_with_cc_id();

        assert!(
            matches!(
                message.try_preprocess(current_time, &connections), 
                Err(MessagePreprocessError::UnknownSource)
            )
        );
    }
    
    #[test]
    fn correct_message_preprocessing() {
        let (connections, cc_id) = connection_graph_with_cc_id();
        
        let current_time = 12;
        let mut message = Message::new(
            cc_id,
            UNKNOWN_ID,
            current_time, 
            MessageType::ChangeGoal(Goal::Reposition)
        );

        assert!(
            message.try_preprocess(current_time, &connections)
                .is_ok()
        );
    }
}
