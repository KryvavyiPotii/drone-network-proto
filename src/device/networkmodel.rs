use std::collections::{
    HashMap,
    hash_map::{Iter, IterMut, Keys, Values, ValuesMut}
};

use petgraph::{graphmap::GraphMap, visit::EdgeRef, Directed};
use rustworkx_core::{
    centrality::betweenness_centrality, 
    dictmap::DictMap, 
    shortest_path::{astar, dijkstra}, 
};
use thiserror::Error;

use crate::device::{
    CommandCenter, Device, DeviceId, Drone, ElectronicWarfare, Receiver,
    Transceiver, Transmitter
};
use crate::communication::{Message, NO_SIGNAL_LEVEL, Scenario, SignalLevel};
use crate::mathphysics::{Megahertz, Meter, Millisecond, Point3D};


pub use cellularautomaton::CellularAutomaton;
pub use complexnetwork::ComplexNetwork;


pub mod cellularautomaton;
pub mod complexnetwork;


pub type IdToLevelMap = HashMap<DeviceId, SignalLevel>;


pub trait FindSignalLevel {
    fn try_set_better_signal_level<T, TR>(
        tx: &T,
        rx: &TR,
        signal_levels: &mut IdToLevelMap,
        frequency: Megahertz
    ) 
    where
        T: Transmitter + Clone,
        TR: Transceiver;

    /// # Errors
    ///
    /// Will return Err if the first drone on found shortest path is not 
    /// present in IdToDroneMap. 
    fn try_find_best_signal_levels(
        command_center: &CommandCenter,
        drone_map: &IdToDroneMap,
        connections: &ConnectionGraph,
        frequency: Megahertz
    ) -> Result<IdToLevelMap, UpdateSignalError> {
        let mut best_signal_levels = HashMap::new();

        for drone in drone_map.drones() {
            let Ok((_, path)) = connections.find_shortest_path_from_to(
                command_center.id(), 
                drone.id()
            ) else {
                continue
            };

            let Some(first_drone) = drone_map.get(&path[1]) else {
                return Err(UpdateSignalError::MissingDrone)
            };

            Self::try_set_better_signal_level(
                command_center,
                first_drone,
                &mut best_signal_levels,
                frequency
            );
            
            // Skipping the first element to avoid reading a command center.
            // Skipping the last element to avoid reading out of bounds.
            for i in 1..(path.len() - 1) {
                let tx_id = path[i];
                let rx_id = path[i + 1];
                
                let Some(tx_drone) = drone_map.get(&tx_id) else { 
                    break 
                };
                let Some(rx_drone) = drone_map.get(&rx_id) else { 
                    break 
                };

                Self::try_set_better_signal_level(
                    tx_drone,
                    rx_drone,
                    &mut best_signal_levels,
                    frequency
                );
            }
        }
       
        Ok(best_signal_levels)
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


#[derive(Error, Debug)]
pub enum MessagePreprocessError {
    #[error("Message execution time is set to be later")]
    TooEarly,
    #[error("Message is already preprocessed")]
    AlreadyPreprocessed
}


fn try_preprocess_message(
    current_time: Millisecond,
    message: &mut Message
) -> Result<(), MessagePreprocessError> {
    if current_time < message.time() {
        return Err(MessagePreprocessError::TooEarly);
    }
    if message.is_in_progress() {
        return Err(MessagePreprocessError::AlreadyPreprocessed);
    }

    message.process();

    Ok(())
}


#[derive(Error, Debug)]
pub enum UpdateSignalError {
    #[error("Shortest Path algorithm failed to calculate path.")]
    AlgorithmError(ShortestPathError),
    #[error("Missing a drone")]
    MissingDrone,
}

impl From<ShortestPathError> for UpdateSignalError {
    fn from(error: ShortestPathError) -> Self {
        Self::AlgorithmError(error)  
    }
}


#[derive(Error, Debug)]
pub enum ShortestPathError {
    #[error("Astar algorithm failed")]
    Astar,
    #[error("Shortest path was not found")]
    NoPathFound,
    #[error("Path length is less than 2")]
    TooShortPath
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
    scenario: Option<Scenario>
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

    // TODO allow usage of the same command center for different networks.
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
    pub fn set_destination(mut self, destination_in_meters: &Point3D) -> Self {
        self.destination_in_meters = Some(*destination_in_meters);
        self
    }

    #[must_use]
    pub fn set_topology(mut self, topology: Topology) -> Self {
        self.topology = Some(topology);
        self
    }

    #[must_use]
    pub fn set_scenario(mut self, scenario: Scenario) -> Self {
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


#[derive(Clone, Debug)]
pub struct IdToDroneMap(HashMap<DeviceId, Drone>);

impl IdToDroneMap {
    #[must_use]
    pub fn get(&self, drone_id: &DeviceId) -> Option<&Drone> {
        self.0.get(drone_id)
    }
    
    #[must_use]
    pub fn get_mut(&mut self, drone_id: &DeviceId) -> Option<&mut Drone> {
        self.0.get_mut(drone_id)
    }

    #[must_use]
    pub fn ids(&self) -> Keys<'_, DeviceId, Drone> {
        self.0.keys()
    }

    #[must_use]
    pub fn drones(&self) -> Values<'_, DeviceId, Drone> {
        self.0.values()
    }
    
    #[must_use]
    pub fn drones_mut(&mut self) -> ValuesMut<'_, DeviceId, Drone> {
        self.0.values_mut()
    }

    #[must_use]
    pub fn iter(&self) -> Iter<'_, DeviceId, Drone> {
        self.0.iter()
    }

    pub fn iter_mut(&mut self) -> IterMut<'_, DeviceId, Drone> {
        self.0.iter_mut()
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }
    
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    #[must_use]
    pub fn all_rx_signal_levels(
        &self, 
        frequency: Megahertz
    ) -> IdToLevelMap {
        self.0
            .values()
            .map(|drone| (
                drone.id(),
                *drone.rx_signal_level(frequency)
            ))
            .collect()
    }

    pub fn connect_command_center(&mut self, command_center: &CommandCenter) {
        self.0
            .values_mut()
            .for_each(|drone| 
                drone.connect_command_center(command_center)
            );
    }

    pub fn update_states(&mut self) {
        self.0
            .values_mut()
            .for_each(Drone::update_state);
    }

    pub fn remove_uncontrolled_drones(&mut self, frequency: Megahertz) { 
        self.0.retain(|_, drone| drone.receives_signal(frequency));
    }
    
    pub fn set_tx_signal_levels(
        &mut self,
        signal_levels: &IdToLevelMap,
        frequency: Megahertz
    ) {
        for (id, drone) in &mut self.0 {
            let Some(signal_level) = signal_levels.get(id) else {
                continue
            };

            drone.set_tx_signal_level(frequency, *signal_level);
        }
    }

    pub fn set_rx_signal_levels(
        &mut self,
        signal_levels: &IdToLevelMap,
        frequency: Megahertz
    ) {
        for (id, drone) in &mut self.0 {
            let Some(signal_level) = signal_levels.get(id) else {
                continue
            };

            drone.set_rx_signal_level(frequency, *signal_level);
        }
    }
    
    pub fn all_receive_signals(
        &mut self, 
        signal_levels: &IdToLevelMap,
        frequency: Megahertz
    ) {
        for (id, drone) in &mut self.0 {
            let Some(signal_level) = signal_levels.get(id) else {
                continue
            };

            drone.receive_signal(frequency, *signal_level);
        }
    }

    pub fn clear_rx_signal_levels(&mut self, frequency: Megahertz) {
        self.0
            .values_mut()
            .for_each(|drone|
                drone.set_rx_signal_level(frequency, NO_SIGNAL_LEVEL)
            );
    }

    pub fn all_rx_signals_to_tx(&mut self, frequency: Megahertz) {
        let rx_signal_levels = self.all_rx_signal_levels(frequency);

        self.set_tx_signal_levels(&rx_signal_levels, frequency);
    }
}

impl<'a> IntoIterator for &'a IdToDroneMap{
    type Item = (&'a DeviceId, &'a Drone);
    type IntoIter = Iter<'a, DeviceId, Drone>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter()
    }
}

impl<'a> IntoIterator for &'a mut IdToDroneMap{
    type Item = (&'a DeviceId, &'a mut Drone);
    type IntoIter = IterMut<'a, DeviceId, Drone>;
    
    fn into_iter(self) -> Self::IntoIter {
         self.iter_mut()
    }
}

impl From<&[Drone]> for IdToDroneMap {
    fn from(drones: &[Drone]) -> Self {
       Self(
           drones 
                .iter()
                .map(|drone| (drone.id(), drone.clone()))
                .collect()
       )
    }
}

impl<const N: usize> From<[Drone; N]> for IdToDroneMap {
    fn from(drones: [Drone; N]) -> Self {
       Self(
           drones 
                .iter()
                .map(|drone| (drone.id(), drone.clone()))
                .collect()
       )
    }
}


// Currently, it considers only distances between devices and not their signal 
// levels.
#[derive(Clone, Debug)]
pub struct ConnectionGraph(GraphMap<DeviceId, Meter, Directed>);

impl ConnectionGraph {
    #[must_use]
    fn new() -> Self {
        Self(GraphMap::new())
    }

    fn add_connection(
        &mut self, 
        a: DeviceId, 
        b: DeviceId, 
        weight: Meter
    ) -> Option<Meter> {
        self.0.add_edge(a, b, weight)
    }

    fn add_device(&mut self, node: DeviceId) -> DeviceId {
        self.0.add_node(node)
    }

    fn clear(&mut self) {
        self.0.clear();
    }

    fn update(
        &mut self, 
        command_center: &CommandCenter,
        drone_map: &IdToDroneMap,
        topology: Topology,
        frequency: Megahertz
    ) {
        self.clear();
       
        // The drones in the command center area repeat the signal to each
        // other and the drones outside the area.
        // If there are no drones inside then the drones outside can not get 
        // any signal.
        // So, it is pointless to continue computation.
        if !self.try_connect_drones_directly_to_cc(
            command_center, 
            drone_map,
            frequency
        ) {
            return;
        }

        if let Topology::Mesh = topology {
            self.try_connect_drones_to_each_other(drone_map, frequency);
        }
    }
 
    fn try_connect_drones_directly_to_cc(
        &mut self,
        command_center: &CommandCenter,
        drone_map: &IdToDroneMap,
        frequency: Megahertz
    ) -> bool {
        let mut connected = false;
        let cc_node = self.add_device(command_center.id());
        
        for drone in drone_map.drones() {
            if let Some(distance) = command_center.connection_distance(
                drone,
                frequency
            ) {
                let node = self.add_device(drone.id());
                
                self.add_connection(cc_node, node, distance);
                
                connected = true;
            };
            if let Some(distance) = drone.connection_distance(
                command_center,
                frequency
            ) {
                let node = self.add_device(drone.id());
                
                self.add_connection(node, cc_node, distance);
            };
        }

        connected
    }

    fn try_connect_drones_to_each_other(
        &mut self, 
        drone_map: &IdToDroneMap,
        frequency: Megahertz
    ) {
        for (i, tx_drone) in drone_map.drones().enumerate() {
            let tx_node = self.add_device(tx_drone.id());
          
            // Loops are prohibited. 
            // Otherwise, shortest path algorithms will not function properly.
            for rx_drone in drone_map.drones().skip(i + 1) {
                let rx_node = self.add_device(rx_drone.id());
        
                if let Some(distance) = tx_drone.connection_distance(
                    rx_drone,
                    frequency
                ) {
                    self.add_connection(tx_node, rx_node, distance);
                }
                if let Some(distance) = rx_drone.connection_distance(
                    tx_drone,
                    frequency
                ) {
                    self.add_connection(rx_node, tx_node, distance);
                }
            }
        }
    }
   
    // Gives shortest distance to a device by distance between devices.
    fn single_source_dijkstra(
        &self,
        source: DeviceId
    ) -> rustworkx_core::Result<DictMap<DeviceId, f32>> {
        dijkstra(
            &self.0,
            source,
            None,
            |edge| Ok(*edge.2),
            None
        )
    }

    // Gives distance and path to a device by distance between devices.
    /// # Errors
    ///
    /// Will return `Err` if the shortest path algorithm does not find an 
    /// appropriate path.
    fn find_shortest_path_from_to(
        &self, 
        source: DeviceId,
        destination: DeviceId 
    ) -> Result<(Meter, Vec<DeviceId>), ShortestPathError> {
        let Ok(Some((distance, path))) = astar(
            &self.0,
            source,
            |finish| -> rustworkx_core::Result<bool> {
                Ok(finish == destination)
            },
            |edge| Ok(*edge.weight()),
            |_| Ok(0.0)
        ) else {
            return Err(ShortestPathError::NoPathFound);
        };

        if path.len() < 2 {
            Err(ShortestPathError::TooShortPath)
        } else {
            Ok((distance, path))
        }
    }
    
    #[must_use]
    fn diameter(&self) -> f32 {
        let shortest_paths: Vec<DictMap<DeviceId, f32>> = self.0
            .nodes()
            .map(|drone_id|
                // unwrap() is used because dijkstra() is infallible.
                self.single_source_dijkstra(drone_id).unwrap()
            )
            .collect();

        let diameter = shortest_paths
            .iter()
            .flat_map(|dictmap| dictmap.values())
            .fold(0f32, |a, &b| a.max(b));
        
        diameter
    }

    #[must_use]
    fn node_load(&self) -> Vec<Option<f64>> {
        betweenness_centrality(&self.0, true, true, 50)
    }
}


#[derive(Clone, Copy, Default)]
pub enum Topology {
    #[default]
    Star,
    Mesh,
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::communication::{
        Goal, GREEN_SIGNAL_LEVEL, MessageType, SignalArea, WIFI_2_4GHZ_FREQUENCY
    };
    use crate::device::{
        CommandCenterBuilder, DroneBuilder,
        modules::{TRXModule, TRXSystem}
    };
    
    use super::*;
    

    // This constant was introduced for testing independently from the global
    // constant.
    const DRONE_TX_CONTROL_RADIUS: f32 = 10.0;
   

    fn drone_tx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        let max_tx_signal_levels = HashMap::from([
            (frequency, GREEN_SIGNAL_LEVEL)
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
            (frequency, GREEN_SIGNAL_LEVEL)
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


    #[test]
    fn too_early_message_preprocessing() {
        let current_time = 12;
        let mut message = Message::new(
            50, 
            MessageType::ChangeGoal(Goal::Reposition)
        );

        assert!(
            matches!(
                try_preprocess_message(current_time, &mut message), 
                Err(MessagePreprocessError::TooEarly)
            )
        );
    }
    
    #[test]
    fn preprocessing_already_preprocessed_message() {
        let current_time = 0;
        let mut message = Message::new(
            current_time, 
            MessageType::ChangeGoal(Goal::Reposition)
        );
        message.process();

        assert!(
            matches!(
                try_preprocess_message(current_time, &mut message), 
                Err(MessagePreprocessError::AlreadyPreprocessed)
            )
        );
    }

    #[test]
    fn network_diameter() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        // Network 1: full mesh with edge weight 1.0.
        let command_center = CommandCenterBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(), 
                    rx_module: TRXModule::default() 
                }
            )
            .build();
        
        let drones1 = IdToDroneMap::from([
            drone_with_trx_system_set(Point3D::new(1.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(2.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(3.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(4.0, 0.0, 0.0)),
        ]);

        let mut connections = ConnectionGraph::new();
        connections.update(
            &command_center,
            &drones1,
            Topology::Mesh,
            frequency
        );

        assert_eq!(connections.diameter(), 4.0);

        // Network 2:
        // 
        // A -(7.0)- B -(7.0)- C -(7.0)- D -(7.0)- E 
        //
        let drones2 = IdToDroneMap::from([
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(21.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(28.0, 0.0, 0.0)),
        ]);
        
        connections.update(
            &command_center,
            &drones2,
            Topology::Mesh,
            frequency
        );

        assert_eq!(connections.diameter(), 28.0);
        
        // Network 3:
        //                      D
        //                      |
        //                    (7.0)
        //                      |
        //  A -(7.0)- B -(7.0)- C
        //                      |
        //                    (7.0)
        //                      |
        //                      E
        //
        let drones3 = IdToDroneMap::from([
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(21.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(21.0, 0.0, 0.0)),
        ]);
        
        connections.update(
            &command_center,
            &drones3,
            Topology::Mesh,
            frequency
        );

        assert_eq!(connections.diameter(), 21.0);
    }
}
