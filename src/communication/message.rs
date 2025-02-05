use derive_more::{Add, Sub, Mul, Div};

use crate::mathphysics::{Point3D, Millisecond};

use super::signal::{SignalType, BASE_SIGNAL_STRENGTH_VALUE};


pub const CHANGE_GOAL_COST: MessageCost     = MessageCost(
    BASE_SIGNAL_STRENGTH_VALUE / 100.0 
);
pub const SET_DESTINATION_COST: MessageCost = MessageCost(
    BASE_SIGNAL_STRENGTH_VALUE / 50.0 
);


fn define_message_execution_cost(message_type: &MessageType) -> MessageCost {
    match message_type {
        MessageType::ChangeGoal(_) => CHANGE_GOAL_COST,
        MessageType::SetDestination(..) =>  SET_DESTINATION_COST
    }
}

fn sort_messages_by_time_ascending(messages: &mut [(SignalType, Message)]) {
    messages.sort_by_key(|(_, message)| message.time());
}


#[derive(
    Clone, Copy, Debug, Default,
    Add, Mul, PartialEq, PartialOrd
)]
pub struct MessageCost(f32);

impl MessageCost {
    pub fn new(value: f32) -> Self {
        let message_cost = if value <= 0.0 {
            0.0
        } else {
            value
        };

        Self(message_cost)
    }

    pub fn value(&self) -> f32 {
        self.0
    }
}

impl Sub for MessageCost {
    type Output = Self;
    
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.0 - rhs.0)
    }
}

impl Sub<f32> for MessageCost {
    type Output = Self;
    
    fn sub(self, rhs: f32) -> Self::Output {
        Self::new(self.0 - rhs)
    }
}

impl Div for MessageCost {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        Self(self.0 / rhs.0)
    }
}

impl Div<f32> for MessageCost {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self(self.0 / rhs)
    }
}


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

    pub fn time(&self) -> Millisecond {
        self.execution_time
    }

    pub fn cost(&self) -> MessageCost {
        self.execution_cost
    }

    pub fn message_type(&self) -> &MessageType {
        &self.message_type
    }

    pub fn message_state(&self) -> &MessageState {
        &self.message_state
    }

    pub fn process(&mut self) {
        match self.message_state {
            MessageState::Waiting => 
                self.message_state = MessageState::InProgress,
            _ => ()
        }
    }

    pub fn finish(&mut self) {
        match self.message_state {
            MessageState::Waiting | MessageState::InProgress => 
                self.message_state = MessageState::Finished,
            _ => ()
        }
    }

    pub fn is_in_progress(&self) -> bool {
        matches!(self.message_state, MessageState::InProgress)
    }

    pub fn is_finished(&self) -> bool {
        matches!(self.message_state, MessageState::Finished)
    }
}


#[derive(Clone, Default)]
pub struct Scenario(Vec<(SignalType, Message)>);

impl Scenario {
    pub fn iter(&self) -> std::slice::Iter<'_, (SignalType, Message)> {
        self.0.iter()
    }

    pub fn vec(&self) -> Vec<(SignalType, Message)> {
        self.0.clone()
    }
}

impl From<&[(SignalType, Message)]> for Scenario {
    fn from(messages: &[(SignalType, Message)]) -> Self {
        let mut scenario = Self(messages.to_vec());

        sort_messages_by_time_ascending(&mut scenario.0);

        scenario
    }
}

impl<const N: usize> From<[(SignalType, Message); N]> for Scenario {
    fn from(messages: [(SignalType, Message); N]) -> Self {
        let mut scenario = Self(messages.to_vec());

        sort_messages_by_time_ascending(&mut scenario.0);

        scenario
    }
}


#[derive(Clone, Debug, Default)]
pub struct MessageQueue(Vec<(SignalType, Message)>);

impl MessageQueue {
    pub fn new() -> Self {
        Self(Vec::new())
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn remove_finished_messages(&mut self) {
        self.0.retain(|(_, message)| !message.is_finished());
    }

    pub fn add_message(&mut self, signal_type: SignalType, message: Message) {
        self.0.push((signal_type, message));
        sort_messages_by_time_ascending(&mut self.0);
    }
    
    pub fn iter(&self) -> std::slice::Iter<'_, (SignalType, Message)> {
        self.0.iter()
    }

    pub fn iter_mut(
        &mut self
    ) -> std::slice::IterMut<'_, (SignalType, Message)> {
        self.0.iter_mut()
    }
    
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl From<&[(SignalType, Message)]> for MessageQueue {
    fn from(messages: &[(SignalType, Message)]) -> Self {
        let mut scenario = Self(messages.to_vec());

        sort_messages_by_time_ascending(&mut scenario.0);

        scenario
    }
}

impl<const N: usize> From<[(SignalType, Message); N]> for MessageQueue {
    fn from(messages: [(SignalType, Message); N]) -> Self {
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

    fn message_vec() -> Vec<(SignalType, Message)> {
        vec![
            (
                SignalType::Control,
                Message::new(25, MessageType::ChangeGoal(Goal::Reposition))
            ),
            (
                SignalType::Control,
                Message::new(5, MessageType::ChangeGoal(Goal::Reposition))
            ),
            (
                SignalType::Control,
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
