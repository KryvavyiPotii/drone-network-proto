use std::ops;

use derive_more::{Add, Mul};
use impl_ops::{
    _impl_binary_op_borrowed_borrowed, _impl_binary_op_borrowed_owned, 
    _impl_binary_op_internal, _impl_binary_op_owned_borrowed, 
    _impl_binary_op_owned_owned, _parse_binary_op, impl_op, impl_op_ex
};

use super::device::DeviceId; 
use super::malware::{Malware, MalwareType};
use super::mathphysics::{Millisecond, Point3D};


pub use queue::MessageQueue;


pub mod queue;


const GPS_COST: MessageCost           = MessageCost(5.0);
const MAL_INDICATOR_COST: MessageCost = MessageCost(1.0);
const MAL_JAMMING_COST: MessageCost   = MessageCost(15.0);
const MAL_DOS_COST: MessageCost       = MessageCost(10.0);
const SET_TASK_COST: MessageCost      = MessageCost(10.0); 


fn message_transmission_cost(message_type: &MessageType) -> MessageCost {
    match message_type {
        MessageType::GPS(_)           => GPS_COST,
        MessageType::Malware(malware) => malware_transmission_cost(
            *malware.malware_type()
        ),
        MessageType::SetTask(_)       => SET_TASK_COST,
    }
}

fn malware_transmission_cost(malware_type: MalwareType) -> MessageCost {
    match malware_type {
        MalwareType::DoS(_)     => MAL_DOS_COST, 
        MalwareType::Indicator  => MAL_INDICATOR_COST,
        MalwareType::Jamming(_) => MAL_JAMMING_COST
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
pub enum Task {
    Attack(Point3D),
    Reconnect(Point3D),
    Reposition(Point3D),
    #[default]
    Undefined,
}


#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MessageType {
    GPS(Point3D),
    Malware(Malware),
    SetTask(Task),
}


#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MessageState {
    Waiting,
    InProgress,
    Finished,
}


#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Message {
    source_id: DeviceId,
    destination_id: DeviceId,
    execution_time: Millisecond,
    transmission_cost: MessageCost,
    message_type: MessageType,
    state: MessageState
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
            transmission_cost: message_transmission_cost(&message_type),
            message_type,
            state: MessageState::Waiting,
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
        self.transmission_cost
    }

    #[must_use]
    pub fn message_type(&self) -> &MessageType {
        &self.message_type
    }

    #[must_use]
    pub fn state(&self) -> &MessageState {
        &self.state
    }

    #[must_use]
    pub fn is_gps(&self) -> bool {
        matches!(self.message_type, MessageType::GPS(_))
    }
    
    #[must_use]
    pub fn is_malware(&self) -> bool {
        matches!(self.message_type, MessageType::Malware(_))
    }
    
    #[must_use]
    pub fn is_set_task(&self) -> bool {
        matches!(self.message_type, MessageType::SetTask(_))
    }

    pub fn process(&mut self) {
        if let MessageState::Waiting = self.state {
            self.state = MessageState::InProgress;
        }
    }

    pub fn finish(&mut self) {
        match self.state {
            MessageState::Waiting | MessageState::InProgress => 
                self.state = MessageState::Finished,
            MessageState::Finished => ()
        }
    }

    #[must_use]
    pub fn is_in_progress(&self) -> bool {
        matches!(self.state, MessageState::InProgress)
    }

    #[must_use]
    pub fn is_finished(&self) -> bool {
        matches!(self.state, MessageState::Finished)
    }
}

#[cfg(test)]
mod tests {
    use crate::backend::device::UNKNOWN_ID;

    use super::*;


    fn message_is_waiting(message: &Message) -> bool {
        matches!(message.state(), MessageState::Waiting) 
    }


    #[test]
    fn process_waiting_message() {
        let not_important_message_type = MessageType::GPS(Point3D::default());

        let mut waiting_message = Message::new(
            UNKNOWN_ID, 
            UNKNOWN_ID, 
            0, 
            not_important_message_type
        );
        
        assert!(message_is_waiting(&waiting_message));

        waiting_message.process();

        assert!(waiting_message.is_in_progress());
    }
    
    #[test]
    fn not_process_finished_message() {
        let not_important_message_type = MessageType::GPS(Point3D::default());

        let mut finished_message = Message::new(
            UNKNOWN_ID, 
            UNKNOWN_ID, 
            0, 
            not_important_message_type
        );

        finished_message.finish();
        
        assert!(finished_message.is_finished());

        finished_message.process();

        assert!(!finished_message.is_in_progress());
        assert!(finished_message.is_finished());
    }
}
