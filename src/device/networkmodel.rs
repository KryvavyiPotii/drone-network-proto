use std::collections::hash_map::Values;

use crate::device::{
    CommandCenter, Device, DeviceId, Drone, ElectronicWarfare, IdToDroneMap,
};
use crate::device::connections::Topology;
use crate::infection::INFECTION_DELAY;
use crate::mathphysics::{Megahertz, Point3D};
use crate::message::{Message, MessageQueue};


pub use cellularautomaton::CellularAutomaton;
pub use complexnetwork::ComplexNetwork;

use super::ConnectionGraph;


pub mod cellularautomaton;
pub mod complexnetwork;


fn spread_infection_messages(
    frequency: Megahertz,
    initial_message: &Message,
    connections: &ConnectionGraph,
    infection_messages: &mut Vec<(Megahertz, Message)>
) {
    let destination_id = initial_message.destination_id();

    for neighbor in connections.neighbors(destination_id) {
        let infection_message = Message::new(
            destination_id,
            neighbor,
            // TODO make infection speed depend on its type
            initial_message.time() + INFECTION_DELAY,
            // TODO make infection spreading depend on its type
            *initial_message.message_type()
        );
        
        infection_messages.push((frequency, infection_message));
    }
}

fn enqueue_infection_messages(
    message_queue: &mut MessageQueue,
    drone_map: &IdToDroneMap,
    infection_messages: &Vec<(Megahertz, Message)>
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

#[must_use]
pub fn get_drone_networks_destinations(
    drone_networks: &[NetworkModel]
) -> Vec<&Point3D> {
    drone_networks
        .iter()
        .map(NetworkModel::destination)
        .collect()
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
    pub fn destination(&self) -> &Point3D {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.destination(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.destination()
        }
    }

    #[must_use]
    pub fn command_center(&self) -> &CommandCenter {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.command_center(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.command_center()
        }
    }

    #[must_use]
    pub fn drone_iter(&self) -> Values<'_, DeviceId, Drone> { 
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.drone_iter(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.drone_iter()
        }
    }

    #[must_use]
    pub fn drone_count(&self) -> usize {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.drone_count(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.drone_count()
        }
    }
    
    #[must_use]
    pub fn ewds(&self) -> &[ElectronicWarfare] { 
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
    command_center: Option<CommandCenter>,
    drones: Option<Vec<Drone>>,
    electronic_warfare_devices: Option<Vec<ElectronicWarfare>>,
    destination_in_meters: Option<Point3D>,
    topology: Option<Topology>,
    scenario: Option<Vec<(Megahertz, Message)>>
}

impl NetworkModelBuilder {
    #[must_use]
    pub fn new(network_model_type: NetworkModelType) -> Self {
        Self {
            network_model_type,
            command_center: None,
            drones: None,
            electronic_warfare_devices: None,
            destination_in_meters: None,
            topology: None,
            scenario: None
        }
    }

    #[must_use]
    pub fn set_command_center(mut self, command_center: CommandCenter) -> Self {
        self.command_center = Some(command_center);
        self
    }

    #[must_use]
    pub fn set_drones(mut self, drones: &[Drone]) -> Self {
        self.drones = Some(drones.to_vec());
        self
    }

    #[must_use]
    pub fn set_electronic_warfare_devices(
        mut self, 
        electronic_warfare_devices: &[ElectronicWarfare]
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
        let drones = IdToDroneMap::from(
            self.drones
                .unwrap_or_default()
                .as_slice()
        );

        match self.network_model_type {
            NetworkModelType::CellularAutomaton => {
                let cellular_automaton = CellularAutomaton::new(
                    self.command_center.unwrap_or_default(),
                    drones,
                    self.electronic_warfare_devices.unwrap_or_default(),
                    &self.scenario.unwrap_or_default(),
                    self.topology.unwrap_or_default(),
                );

                NetworkModel::CellularAutomaton(cellular_automaton)
            },
            NetworkModelType::ComplexNetwork(delay_multiplier) => {
                let complex_network = ComplexNetwork::new(
                    self.command_center.unwrap_or_default(),
                    drones,
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

    use crate::device::{
        CommandCenterBuilder, ConnectionGraph, DroneBuilder, IdToDroneMap
    };
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
   
    fn drone_with_trx_system_set(position: Point3D) -> Drone {
        DroneBuilder::new()
            .set_global_position(position)
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: drone_rx_module() 
                }
            )
            .build()
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
    fn spreading_infection_messages() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build();
        
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
        let drones = [
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, 7.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, -7.0, 0.0)),
        ];
        let drone1_id = drones[0].id();
        let drone2_id = drones[1].id();
        let drone3_id = drones[2].id();
        let drone4_id = drones[3].id();

        let drone_map = IdToDroneMap::from(drones);
        
        let mut connections = ConnectionGraph::new();
        connections.update(
            &command_center, 
            &drone_map, 
            Topology::Mesh, 
            frequency
        );
        
        let initial_message_from_drone1 = Message::new(
            command_center.id(),
            drone1_id,
            0, 
            MessageType::Infection(InfectionType::Jamming)
        );
        let mut infection_messages = Vec::new();

        spread_infection_messages(
            frequency, 
            &initial_message_from_drone1, 
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
                command_center.id()
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
            command_center.id(),
            drone2_id,
            0, 
            MessageType::Infection(InfectionType::Jamming)
        );

        spread_infection_messages(
            frequency, 
            &initial_message_from_drone2, 
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
