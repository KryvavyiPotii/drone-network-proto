use std::slice::{Iter, IterMut};

use crate::backend::device::IdToDelayMap;
use crate::backend::mathphysics::Megahertz;

use super::Message;


#[derive(Clone, Debug, Default)]
pub struct MessageQueue(Vec<(Megahertz, Message, IdToDelayMap)>);

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
    
    pub fn iter(
        &self
    ) -> Iter<'_, (Megahertz, Message, IdToDelayMap)> {
        self.0.iter()
    }
    
    pub fn iter_mut(
        &mut self
    ) -> IterMut<'_, (Megahertz, Message, IdToDelayMap)> {
        self.0.iter_mut()
    }
   
    pub fn add_message(
        &mut self, 
        message: Message,
        frequency: Megahertz, 
    ) {
        self.0.push((frequency, message, IdToDelayMap::default()));
        self.0.sort_by_key(|(_, message, _)| message.time());
    }

    pub fn remove_finished_messages(&mut self) {
        self.0.retain(|(_, message, _)| !message.is_finished());
    }
}

impl<'a> IntoIterator for &'a MessageQueue{
    type Item = &'a (Megahertz, Message, IdToDelayMap);
    type IntoIter = Iter<'a, (Megahertz, Message, IdToDelayMap)>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter()
    }
}

impl<'a> IntoIterator for &'a mut MessageQueue{
    type Item = &'a mut (Megahertz, Message, IdToDelayMap);
    type IntoIter = IterMut<'a, (Megahertz, Message, IdToDelayMap)>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter_mut()
    }
}

impl From<&[(Megahertz, Message)]> for MessageQueue {
    fn from(messages: &[(Megahertz, Message)]) -> Self {
        let messages: Vec<(Megahertz, Message, IdToDelayMap)> = messages
            .iter()
            .map(|(frequency, message)| 
                (*frequency, *message, IdToDelayMap::default())
            )
            .collect();

        let mut message_queue = Self(messages);

        message_queue.0.sort_by_key(|(_, message, _)| message.time());

        message_queue
    }
}

impl<const N: usize> From<[(Megahertz, Message); N]> for MessageQueue {
    fn from(messages: [(Megahertz, Message); N]) -> Self {
        let messages: Vec<(Megahertz, Message, IdToDelayMap)> = messages
            .iter()
            .map(|(frequency, message)| 
                (*frequency, *message, IdToDelayMap::default())
            )
            .collect();

        let mut message_queue = Self(messages);

        message_queue.0.sort_by_key(|(_, message, _)| message.time());

        message_queue
    }
}


#[cfg(test)]
mod tests {
    use crate::backend::message::{Task, MessageType};

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
                    MessageType::SetTask(Task::Undefined)
                )
            ),
            (
                SOME_FREQUENCY,
                Message::new(
                    1, 
                    2, 
                    5, 
                    MessageType::SetTask(Task::Undefined)
                )
            ),
            (
                SOME_FREQUENCY,
                Message::new(
                    1, 
                    3, 
                    10, 
                    MessageType::SetTask(Task::Undefined)
                )
            )
        ]
    }


    #[test]
    fn removing_finished_messages() {
        let mut messages = message_vec();

        messages[1].1.finish();

        let mut message_queue = MessageQueue::from(messages.as_slice());
        message_queue.remove_finished_messages();

        assert_eq!(message_queue.len(), 2);
        for (_, message, _) in message_queue.iter() {
            assert!(!message.is_finished());
        }
    }
    
    #[test]
    fn sort_messages_on_creation() {
        let messages = message_vec();

        let message_queue = MessageQueue::from(messages.as_slice());
        let mut queue_iter = message_queue.into_iter();

        assert_eq!(
            messages[1].1.time(),
            queue_iter.next().unwrap().1.time()
        );
        assert_eq!(
            messages[2].1.time(),
            queue_iter.next().unwrap().1.time()
        );
        assert_eq!(
            messages[0].1.time(),
            queue_iter.next().unwrap().1.time()
        );
    }

    #[test]
    fn sort_messages_while_adding() {

        let mut message_queue = MessageQueue::new();
        
        let messages = message_vec();
        for (frequency, message) in &messages {
            message_queue.add_message(*message, *frequency);
        }

        let mut queue_iter = message_queue.into_iter();

        assert_eq!(
            messages[1].1.time(),
            queue_iter.next().unwrap().1.time()
        );
        assert_eq!(
            messages[2].1.time(),
            queue_iter.next().unwrap().1.time()
        );
        assert_eq!(
            messages[0].1.time(),
            queue_iter.next().unwrap().1.time()
        );
    }
}
