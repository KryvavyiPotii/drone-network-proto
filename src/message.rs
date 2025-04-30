use std::ops;

use derive_more::{Add, Mul};
use impl_ops::{
    _impl_binary_op_borrowed_borrowed, _impl_binary_op_borrowed_owned, 
    _impl_binary_op_internal, _impl_binary_op_owned_borrowed, 
    _impl_binary_op_owned_owned, _parse_binary_op, impl_op, impl_op_ex
};
use thiserror::Error;

use crate::device::DeviceId; 
use crate::infection::InfectionType;
use crate::mathphysics::{Millisecond, Point3D};

use super::signal::GREEN_SIGNAL_STRENGTH_VALUE;


pub use queue::MessageQueue;


pub mod queue;


const GPS_COST: MessageCost       = MessageCost(
    GREEN_SIGNAL_STRENGTH_VALUE / 100.0 
);
const INFECTION_COST: MessageCost = MessageCost(
    GREEN_SIGNAL_STRENGTH_VALUE / 50.0
);
const SET_GOAL_COST: MessageCost  = MessageCost(
    GREEN_SIGNAL_STRENGTH_VALUE / 100.0 
);


fn define_message_execution_cost(message_type: &MessageType) -> MessageCost {
    match message_type {
        MessageType::GPS(_)       => GPS_COST,
        // TODO add cost dependency on infection type
        MessageType::Infection(_) => INFECTION_COST,
        MessageType::SetGoal(_)   => SET_GOAL_COST,
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


#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Goal {
    #[default]
    Undefined,
    Reposition(Point3D),
    Attack(Point3D),
}


#[derive(Clone, Copy, Debug)]
pub enum MessageType {
    GPS(Point3D),
    Infection(InfectionType),
    SetGoal(Goal),
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
    ) -> Result<(), MessagePreprocessError> {
        if self.is_in_progress() {
            return Err(MessagePreprocessError::AlreadyPreprocessed);
        }
        if current_time < self.time() {
            return Err(MessagePreprocessError::TooEarly);
        }

        self.process();

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::device::{BROADCAST_ID, UNKNOWN_ID};

    use super::*;


    #[test]
    fn preprocessing_already_preprocessed_message() {
        let current_time = 10;
        let mut message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            current_time, 
            MessageType::SetGoal(Goal::Undefined)
        );
        message.process();

        assert!(
            matches!(
                message.try_preprocess(current_time), 
                Err(MessagePreprocessError::AlreadyPreprocessed)
            )
        );
    }

    #[test]
    fn too_early_message_preprocessing() {
        let current_time = 12;
        let mut message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            50, 
            MessageType::SetGoal(Goal::Undefined)
        );

        assert!(
            matches!(
                message.try_preprocess(current_time), 
                Err(MessagePreprocessError::TooEarly)
            )
        );
    }
    
    #[test]
    fn correct_message_preprocessing() {
        let current_time = 12;
        let mut message = Message::new(
            UNKNOWN_ID,
            BROADCAST_ID,
            current_time, 
            MessageType::SetGoal(Goal::Undefined)
        );

        assert!(
            message.try_preprocess(current_time)
                .is_ok()
        );
    }
}
