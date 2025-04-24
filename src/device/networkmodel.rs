use std::collections::hash_map::Values;

use thiserror::Error;

use crate::device::{Device, DeviceId, IdToDeviceMap};
use crate::device::connections::Topology;
use crate::infection::INFECTION_DELAY;
use crate::mathphysics::{Megahertz, Point3D};
use crate::message::{Message, MessageQueue};
use crate::signal::{GPS_L1_FREQUENCY, GREEN_SIGNAL_LEVEL};


pub use cellularautomaton::CellularAutomaton;
pub use complexnetwork::ComplexNetwork;

use super::{ConnectionGraph, IdToGoalMap};


pub mod cellularautomaton;
pub mod complexnetwork;


#[derive(Error, Debug)]
pub enum UnicastMessageError {
    #[error("Message was not received")]
    NotReceived,
    #[error("Message should be sent later")]
    TooEarly,
}


fn multiply_infection_messages(
    initial_message: &Message,
    frequency: Megahertz,
    infected_device: DeviceId,
    connections: &ConnectionGraph,
    infection_messages: &mut Vec<(Megahertz, Message)>
) {
    for neighbor in connections.neighbors(infected_device) {
        let infection_message = Message::new(
            infected_device,
            neighbor,
            // TODO make infection speed depend on its type
            initial_message.time() + INFECTION_DELAY,
            // TODO make infection multiplying depend on its type
            *initial_message.message_type()
        );
        
        infection_messages.push((frequency, infection_message));
    }
}

fn enqueue_infection_messages(
    infection_messages: &Vec<(Megahertz, Message)>,
    message_queue: &mut MessageQueue,
    drone_map: &IdToDeviceMap,
) {
    let mut infected_drones = Vec::new();

    for (frequency, infection_message) in infection_messages {
        let Some(drone) = drone_map.get(
            &infection_message.destination_id()
        ) else {
            continue;
        };

        // TODO reinfection handling that depends on the infection type
        if drone.is_infected() || infected_drones.contains(&drone.id()) {
            continue;
        }

        message_queue.add_message(*frequency, *infection_message);
        
        infected_drones.push(drone.id());
    }
}

fn connect_gps_to_all_devices(device_map: &mut IdToDeviceMap) {
    device_map.set_rx_signal_level(
        &GREEN_SIGNAL_LEVEL,
        GPS_L1_FREQUENCY, 
    );
}


#[derive(Clone)]
pub enum NetworkModel {
    ComplexNetwork(ComplexNetwork),
    CellularAutomaton(CellularAutomaton),
}

impl NetworkModel {
    #[must_use]
    pub fn complex_network(&self) -> Option<&ComplexNetwork> {
        match self {
            Self::ComplexNetwork(complex_network) => Some(complex_network),
            Self::CellularAutomaton(_) => None
        }
    }
    
    #[must_use]
    pub fn complex_network_mut(&mut self) -> Option<&mut ComplexNetwork> {
        match self {
            Self::ComplexNetwork(complex_network) => Some(complex_network),
            Self::CellularAutomaton(_) => None
        }
    }

    #[must_use]
    pub fn cellular_automaton(&self) -> Option<&CellularAutomaton> {
        match self {
            Self::CellularAutomaton(cellular_automaton) => 
                Some(cellular_automaton),
            Self::ComplexNetwork(_) => None
        }
    }
    
    #[must_use]
    pub fn cellular_automaton_mut(&mut self) -> Option<&mut CellularAutomaton> {
        match self {
            Self::CellularAutomaton(cellular_automaton) => 
                Some(cellular_automaton),
            Self::ComplexNetwork(_) => None
        }
    }

    pub fn update(&mut self) {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.update(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.update()
        }
    }

    #[must_use]
    pub fn goals(&self) -> IdToGoalMap {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.goals(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.goals()
        }
    }

    #[must_use]
    pub fn command_device(&self) -> &Device {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.command_device(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.command_device()
        }
    }

    #[must_use]
    pub fn device_iter(&self) -> Values<'_, DeviceId, Device> { 
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.device_iter(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.device_iter()
        }
    }

    #[must_use]
    pub fn device_count(&self) -> usize {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.device_count(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.device_count()
        }
    }
    
    #[must_use]
    pub fn ewds(&self) -> &[Device] { 
        match self {
            Self::CellularAutomaton(cellular_automaton) => 
                cellular_automaton.ewds(),
            Self::ComplexNetwork(complex_network) => 
                complex_network.ewds()
        }
    }
}


#[derive(Clone, Copy)]
pub enum NetworkModelType {
    CellularAutomaton,
    ComplexNetwork(f32), // delay multiplier
}


#[derive(Clone)]
pub struct NetworkModelBuilder {
    network_model_type: NetworkModelType,
    command_center_id: Option<DeviceId>,
    devices: Option<Vec<Device>>,
    electronic_warfare_devices: Option<Vec<Device>>,
    destination_in_meters: Option<Point3D>,
    topology: Option<Topology>,
    scenario: Option<Vec<(Megahertz, Message)>>
}

impl NetworkModelBuilder {
    #[must_use]
    pub fn new(network_model_type: NetworkModelType) -> Self {
        Self {
            network_model_type,
            command_center_id: None,
            devices: None,
            electronic_warfare_devices: None,
            destination_in_meters: None,
            topology: None,
            scenario: None
        }
    }

    #[must_use]
    pub fn set_command_center_id(
        mut self, 
        command_center_id: DeviceId
    ) -> Self {
        self.command_center_id = Some(command_center_id);
        self
    }

    #[must_use]
    pub fn set_devices(mut self, devices: &[Device]) -> Self {
        self.devices = Some(devices.to_vec());
        self
    }

    #[must_use]
    pub fn set_electronic_warfare_devices(
        mut self, 
        electronic_warfare_devices: &[Device]
    ) -> Self {
        self.electronic_warfare_devices = Some(
            electronic_warfare_devices.to_vec()
        );
        self
    }

    #[must_use]
    pub fn set_destination(mut self, destination_in_meters: Point3D) -> Self {
        self.destination_in_meters = Some(destination_in_meters);
        self
    }

    #[must_use]
    pub fn set_topology(mut self, topology: Topology) -> Self {
        self.topology = Some(topology);
        self
    }

    #[must_use]
    pub fn set_scenario(mut self, scenario: Vec<(Megahertz, Message)>) -> Self {
        self.scenario = Some(scenario);
        self
    }

    #[must_use]
    pub fn build(self) -> NetworkModel {
        let device_map = IdToDeviceMap::from(
            self.devices
                .unwrap_or_default()
                .as_slice()
        );

        match self.network_model_type {
            NetworkModelType::CellularAutomaton => {
                let cellular_automaton = CellularAutomaton::new(
                    self.command_center_id.unwrap_or_default(),
                    device_map,
                    self.electronic_warfare_devices.unwrap_or_default(),
                    &self.scenario.unwrap_or_default(),
                    self.topology.unwrap_or_default(),
                );

                NetworkModel::CellularAutomaton(cellular_automaton)
            },
            NetworkModelType::ComplexNetwork(delay_multiplier) => {
                let complex_network = ComplexNetwork::new(
                    self.command_center_id.unwrap_or_default(),
                    device_map,
                    self.electronic_warfare_devices.unwrap_or_default(),
                    &self.scenario.unwrap_or_default(),
                    self.topology.unwrap_or_default(),
                    delay_multiplier
                );

                NetworkModel::ComplexNetwork(complex_network)
            },
        }
    }
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::device::{ConnectionGraph, DeviceBuilder, IdToDeviceMap};
    use crate::device::systems::{TRXModule, TRXSystem};
    use crate::infection::InfectionType;
    use crate::mathphysics::Meter;
    use crate::message::MessageType;
    use crate::signal::{
        GREEN_SIGNAL_STRENGTH_VALUE, NO_SIGNAL_LEVEL, SignalArea, SignalLevel, 
        WIFI_2_4GHZ_FREQUENCY
    };

    use super::*;

    
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const VERY_BIG_STRENGTH_VALUE: f32   = GREEN_SIGNAL_STRENGTH_VALUE * 1000.0;


    fn drone_tx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        let max_tx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let tx_signal_levels = HashMap::from([(
            frequency, 
            SignalLevel::from_area(
                SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap(), 
                frequency
            )
        )]);

        TRXModule::build(
            max_tx_signal_levels,
            tx_signal_levels,
        ).unwrap()
    }

    fn drone_rx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        let max_rx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let rx_signal_levels = HashMap::from([
            (frequency, NO_SIGNAL_LEVEL)
        ]);

        TRXModule::build(
            max_rx_signal_levels,
            rx_signal_levels,
        ).unwrap()   
    }
   
    fn drone_with_trx_system_set(position: Point3D) -> Device {
        DeviceBuilder::new()
            .set_global_position(position)
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module() 
                }
            )
            .build()
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn assert_all_messages_are_infections(
        messages: &Vec<(Megahertz, Message)>
    ) {
        assert!(
            messages
                .iter()
                .all(|(_, message)| 
                    matches!(message.message_type(), MessageType::Infection(_))
                )
        );
    }

    fn correct_source_and_destination_ids(
        infection_messages: &Vec<(Megahertz, Message)>, 
        source_id: DeviceId, 
        destination_id: DeviceId
    ) -> bool {
        infection_messages
            .iter()
            .any(|(_, message)| 
                message.source_id() == source_id
                && message.destination_id() == destination_id
            )
    }


    #[test]
    fn multiplying_infection_messages() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let command_center = DeviceBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let command_center_id = command_center.id();
        
        // Network topology:
        //                      D
        //                      |
        //                    (7.0)
        //                      |
        //  A -(7.0)- B -(9.0)- C
        //                      |
        //                    (7.0)
        //                      |
        //                      E
        //
        let devices = [
            command_center,
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, 7.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, -7.0, 0.0)),
        ];
        let drone1_id = devices[1].id();
        let drone2_id = devices[2].id();
        let drone3_id = devices[3].id();
        let drone4_id = devices[4].id();

        let mut connections = ConnectionGraph::new();
        connections.update(
            command_center_id, 
            &IdToDeviceMap::from(devices), 
            Topology::Mesh, 
            frequency
        );
        
        let initial_message_from_drone1 = Message::new(
            command_center_id,
            drone1_id,
            0, 
            MessageType::Infection(InfectionType::Jamming)
        );
        let mut infection_messages = Vec::new();

        multiply_infection_messages(
            &initial_message_from_drone1, 
            frequency, 
            drone1_id,
            &connections, 
            &mut infection_messages
        );

        assert_eq!(2, infection_messages.len());
        assert_all_messages_are_infections(&infection_messages);
        // Message from drone1 (B) to the command center (A).
        assert!(
            correct_source_and_destination_ids(
                &infection_messages, 
                drone1_id, 
                command_center_id
            )
        );
        // Message from drone1 (B) to drone2 (C).
        assert!(
            correct_source_and_destination_ids(
                &infection_messages, 
                drone1_id, 
                drone2_id
            )
        );
        infection_messages.clear();

        let initial_message_from_drone2 = Message::new(
            command_center_id,
            drone2_id,
            0, 
            MessageType::Infection(InfectionType::Jamming)
        );

        multiply_infection_messages(
            &initial_message_from_drone2, 
            frequency, 
            drone2_id,
            &connections, 
            &mut infection_messages
        );

        assert_eq!(3, infection_messages.len());
        assert_all_messages_are_infections(&infection_messages);
        // Message from drone2 (C) to drone1 (B).
        assert!(
            correct_source_and_destination_ids(
                &infection_messages, 
                drone2_id, 
                drone1_id
            )
        );
        // Message from drone2 (C) to drone3 (D).
        assert!(
            correct_source_and_destination_ids(
                &infection_messages, 
                drone2_id, 
                drone3_id
            )
        );
        // Message from drone2 (C) to drone4 (E).
        assert!(
            correct_source_and_destination_ids(
                &infection_messages, 
                drone2_id, 
                drone4_id
            )
        );
    }
}
