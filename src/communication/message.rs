use std::ops;

use derive_more::{Add, Mul};
use impl_ops::{
    _impl_binary_op_borrowed_borrowed, _impl_binary_op_borrowed_owned, 
    _impl_binary_op_internal, _impl_binary_op_owned_borrowed, 
    _impl_binary_op_owned_owned, _parse_binary_op, impl_op, impl_op_ex
};

use crate::mathphysics::{Point3D, Millisecond, Megahertz};

use super::signal::GREEN_SIGNAL_STRENGTH_VALUE;


pub const CHANGE_GOAL_COST: MessageCost     = MessageCost(
    GREEN_SIGNAL_STRENGTH_VALUE / 100.0 
);
pub const SET_DESTINATION_COST: MessageCost = MessageCost(
    GREEN_SIGNAL_STRENGTH_VALUE / 50.0 
);


fn define_message_execution_cost(message_type: &MessageType) -> MessageCost {
    match message_type {
        MessageType::ChangeGoal(_) => CHANGE_GOAL_COST,
        MessageType::SetDestination(..) =>  SET_DESTINATION_COST
    }
}

fn sort_messages_by_time_ascending(messages: &mut [(Megahertz, Message)]) {
    messages.sort_by_key(|(_, message)| message.time());
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
    SetDestination(Point3D, Goal),
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
    execution_time: Millisecond,
    execution_cost: MessageCost,
    message_type: MessageType,
    message_state: MessageState
}

impl Message {
    #[must_use]
    pub fn new(
        execution_time: Millisecond,
        message_type: MessageType
    ) -> Self {
        Self { 
            execution_time,
            execution_cost: define_message_execution_cost(&message_type),
            message_type,
            message_state: MessageState::Waiting,
        }
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
}


#[derive(Clone, Debug, Default)]
pub struct Scenario(Vec<(Megahertz, Message)>);

impl Scenario {
    pub fn iter(&self) -> std::slice::Iter<'_, (Megahertz, Message)> {
        self.0.iter()
    }

    #[must_use]
    pub fn vec(&self) -> Vec<(Megahertz, Message)> {
        self.0.clone()
    }
}

impl<'a> IntoIterator for &'a Scenario{
    type Item = &'a (Megahertz, Message);
    type IntoIter = std::slice::Iter<'a, (Megahertz, Message)>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter()
    }
}

impl From<&[(Megahertz, Message)]> for Scenario {
    fn from(messages: &[(Megahertz, Message)]) -> Self {
        let mut scenario = Self(messages.to_vec());

        sort_messages_by_time_ascending(&mut scenario.0);

        scenario
    }
}

impl<const N: usize> From<[(Megahertz, Message); N]> for Scenario {
    fn from(messages: [(Megahertz, Message); N]) -> Self {
        let mut scenario = Self(messages.to_vec());

        sort_messages_by_time_ascending(&mut scenario.0);

        scenario
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

    pub fn remove_finished_messages(&mut self) {
        self.0.retain(|(_, message)| !message.is_finished());
    }

    pub fn add_message(&mut self, frequency: Megahertz, message: Message) {
        self.0.push((frequency, message));
        sort_messages_by_time_ascending(&mut self.0);
    }
    
    pub fn iter(&self) -> std::slice::Iter<'_, (Megahertz, Message)> {
        self.0.iter()
    }

    pub fn iter_mut(
        &mut self
    ) -> std::slice::IterMut<'_, (Megahertz, Message)> {
        self.0.iter_mut()
    }
    
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
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
        let mut scenario = Self(messages.to_vec());

        sort_messages_by_time_ascending(&mut scenario.0);

        scenario
    }
}

impl<const N: usize> From<[(Megahertz, Message); N]> for MessageQueue {
    fn from(messages: [(Megahertz, Message); N]) -> Self {
        let mut scenario = Self(messages.to_vec());

        sort_messages_by_time_ascending(&mut scenario.0);

        scenario
    }
}

impl From<Scenario> for MessageQueue {
    fn from(scenario: Scenario) -> Self {
        Self(scenario.0)
    }
}

impl From<&Scenario> for MessageQueue {
    fn from(scenario: &Scenario) -> Self {
        Self(scenario.vec())
    }
}


#[cfg(test)]
mod tests {
    use super::*;


    const SOME_FREQUENCY: Megahertz = 2_000;


    fn message_vec() -> Vec<(Megahertz, Message)> {
        vec![
            (
                SOME_FREQUENCY,
                Message::new(25, MessageType::ChangeGoal(Goal::Reposition))
            ),
            (
                SOME_FREQUENCY,
                Message::new(5, MessageType::ChangeGoal(Goal::Reposition))
            ),
            (
                SOME_FREQUENCY,
                Message::new(10, MessageType::ChangeGoal(Goal::Reposition))
            )
        ]
    }


    #[test]
    fn correct_message_sort() {
        let messages = message_vec();
        let mut sorted_messages = messages.clone();

        sort_messages_by_time_ascending(&mut sorted_messages);

        assert_eq!(sorted_messages[0].1.time(), messages[1].1.time());
        assert_eq!(sorted_messages[1].1.time(), messages[2].1.time());
        assert_eq!(sorted_messages[2].1.time(), messages[0].1.time());
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
}
