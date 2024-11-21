use crate::math_physics::Coordinates3D;


#[derive(Clone, Copy, Debug)]
pub enum Goal {
    Reposition,
    Attack,
}

#[derive(Clone, Copy, Debug)]
pub enum MessageType {
    SetDestination(Option<Coordinates3D>, Goal),
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
    execution_time_in_millis: u64,
    message_type: MessageType,
    message_state: MessageState,
}

impl Message {
    pub fn new(
        execution_time_in_millis: u64,
        message_type: MessageType
    ) -> Self {
        Self { 
            execution_time_in_millis,
            message_type,
            message_state: MessageState::Waiting,
        }
    }

    pub fn time(&self) -> u64 {
        self.execution_time_in_millis
    }

    pub fn message_type(&self) -> &MessageType {
        &self.message_type
    }

    pub fn message_state(&self) -> &MessageState {
        &self.message_state
    }

    pub fn message_state_mut(&mut self) -> &mut MessageState {
        &mut self.message_state
    }
}
