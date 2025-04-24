use std::collections::{BinaryHeap, HashMap};

use thiserror::Error;

use petgraph::Directed;
use petgraph::graphmap::{GraphMap, Neighbors}; 
use petgraph::visit::EdgeRef;
use rustworkx_core::centrality::betweenness_centrality;
use rustworkx_core::dictmap::DictMap;
use rustworkx_core::shortest_path::{astar, dijkstra};

use crate::device::{Device, DeviceId, STEP_DURATION};
use crate::device::{IdToDeviceMap, IdToLevelMap};
use crate::infection::{InfectionState, InfectionType};
use crate::mathphysics::{
    kmps_to_mpms, Megahertz, Meter, Millisecond, SPEED_OF_LIGHT, 
    time_in_millis_from_distance_and_speed
};


pub type DelaySnapshot = HashMap<DeviceId, u32>;

type DijkstraResultStruct = (
    Vec<DeviceId>,
    HashMap<DeviceId, Vec::<DeviceId>>,
    HashMap<DeviceId, f32>,
);

const DELAY_DISTANCE_COEFFICIENT: f32 = 1_500_000.0;


pub trait FindSignalLevel {
    fn try_set_better_signal_level(
        tx: &Device,
        rx: &Device,
        signal_levels: &mut IdToLevelMap,
        frequency: Megahertz
    ); 

    /// # Errors
    ///
    /// Will return Err if the first drone on found shortest path is not 
    /// present in `IdToDeviceMap`.
    fn try_find_best_signal_levels(
        command_device_id: DeviceId,
        device_map: &IdToDeviceMap,
        connections: &ConnectionGraph,
        frequency: Megahertz
    ) -> Result<IdToLevelMap, UpdateSignalError> {
        let Some(command_device) = device_map.get(&command_device_id) else {
            return Err(UpdateSignalError::MissingDevice);
        };
        let mut best_signal_levels = HashMap::from([
            (command_device_id, *command_device.tx_signal_level(frequency))
        ]);

        for device in device_map.devices() {
            let Ok((_, path)) = connections.find_shortest_path_from_to(
                command_device_id, 
                device.id()
            ) else {
                continue;
            };

            // Skipping the last element to avoid reading out of bounds.
            for i in 0..(path.len() - 1) {
                let tx_id = path[i];
                let rx_id = path[i + 1];
                
                let Some(tx_device) = device_map.get(&tx_id) else { 
                    break 
                };
                let Some(rx_device) = device_map.get(&rx_id) else { 
                    break 
                };

                Self::try_set_better_signal_level(
                    tx_device,
                    rx_device,
                    &mut best_signal_levels,
                    frequency
                );
            }
        }

        Ok(best_signal_levels)
    }
}


#[derive(Error, Debug)]
pub enum UpdateSignalError {
    #[error("Shortest Path algorithm failed to calculate path.")]
    AlgorithmError(ShortestPathError),
    #[error("Missing a drone")]
    MissingDevice,
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
    

// Translation of Python 3 NetworkX method `networkx.algorithms.\
// centrality.percolation._accumulate_percolation` to Rust.
// It is a helper method for `ConnectionGraph::percolation_centrality`.
#[must_use]
fn accumulate_percolation(
    mut percolation_map: HashMap<DeviceId, f32>,
    mut stack: Vec<DeviceId>,
    mut path_map: HashMap<DeviceId, Vec<DeviceId>>,
    number_of_paths_through_map: &HashMap<DeviceId, f32>,
    source: DeviceId,
    states: &HashMap<DeviceId, f32>,
    percolation_sum: f32,
) -> HashMap<DeviceId, f32> {
    let mut delta: HashMap<DeviceId, f32> = stack
        .iter()
        .map(|node| (*node, 0.0))
        .collect();

    while let Some(node) = stack.pop() {
        let node_delta = *delta
            .get(&node)
            .unwrap();
        let number_of_paths_through_node = *number_of_paths_through_map
            .get(&node)
            .unwrap();

        let coefficient = (1.0 + node_delta) / number_of_paths_through_node;

        for path_node in path_map
            .get_mut(&node)
            .unwrap()
            .iter_mut()
        {
            let path_node_delta = *delta
                .get(path_node)
                .unwrap();
            let number_of_paths_through_path_node = *number_of_paths_through_map
                .get(path_node)
                .unwrap();

            delta.insert(
                *path_node,
                path_node_delta + number_of_paths_through_path_node 
                    * coefficient
            );
        }

        if node != source {
            let source_state = *states
                .get(&source)
                .unwrap();
            let node_state = *states
                .get(&node)
                .unwrap();

            let percolation_weight = source_state 
                / (percolation_sum - node_state);
            let current_percolation = *percolation_map
                .get(&node)
                .unwrap();

            percolation_map.insert(
                node,
                current_percolation + node_delta * percolation_weight
            );
        }
    }

    percolation_map
}

#[must_use]
fn percolation_state_from_infection_state(
    infection_state: InfectionState
) -> f32 {
    match infection_state {
        InfectionState::Vulnerable => 1.0,
        InfectionState::Infected => 0.5,
        InfectionState::Patched => 0.0
    } 
}

#[must_use]
fn delay_to(distance: Meter, multiplier: f32) -> Millisecond {    
    if multiplier == 0.0 {
        return 0;
    }

    let delay = (multiplier * time_in_millis_from_distance_and_speed(
        distance * DELAY_DISTANCE_COEFFICIENT, 
        kmps_to_mpms(SPEED_OF_LIGHT) 
    )) as Millisecond;

    let reminder = delay % STEP_DURATION;
    
    delay - reminder
}


#[derive(Clone, Copy, Default)]
pub enum Topology {
    #[default]
    Star,
    Mesh,
}


// It is a helper struct for 
// `ConnectionGraph::single_source_dijkstra_path_basic`.
#[derive(Debug)]
struct HelperStruct {
    distance: Meter,
    counter: u32,
    previous_node: DeviceId,
    current_node: DeviceId
}

impl HelperStruct {
    #[must_use]
    fn new(
        distance: Meter,
        counter: u32,
        previous_node: DeviceId,
        current_node: DeviceId
    ) -> Self {
        Self {
            distance,
            counter,
            previous_node,
            current_node
        }
    }
}

impl PartialEq for HelperStruct {
    fn eq(&self, other: &Self) -> bool {
        self.distance == other.distance && self.counter == other.counter
    }
}

impl Eq for HelperStruct {}

impl PartialOrd for HelperStruct {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

// Comparison that is similar to Python's tuple comparison.
impl Ord for HelperStruct {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        let distance_comparison = self.distance
            .partial_cmp(&other.distance)
            .unwrap_or(std::cmp::Ordering::Equal);

        if let std::cmp::Ordering::Equal = distance_comparison {
            self.counter.cmp(&other.counter)
        } else {
            distance_comparison
        }
    }
}


// Currently, it considers only distances between devices while building the 
// most efficient paths. It ignores signal levels of devices.
#[derive(Clone, Debug)]
pub struct ConnectionGraph(GraphMap<DeviceId, Meter, Directed>);

impl ConnectionGraph {
    #[must_use]
    pub fn new() -> Self {
        Self(GraphMap::new())
    }

    #[must_use]
    pub fn contains_device(&self, node: DeviceId) -> bool {
        self.0.contains_node(node)
    }

    #[must_use]
    pub fn neighbors(
        &self, 
        node: DeviceId
    ) -> Neighbors<'_, DeviceId, Directed> {
        self.0.neighbors(node)
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }

    pub fn update(
        &mut self, 
        command_device_id: DeviceId,
        device_map: &IdToDeviceMap,
        topology: Topology,
        frequency: Megahertz
    ) {
        self.clear();
        
        let Some(command_device) = device_map.get(&command_device_id) else {
            return 
        };

        // The drones in the command device area forward the signal to each
        // other and the drones outside the area.
        // If there are no drones inside then the drones outside can not get 
        // any signal.
        // So, it is pointless to continue computation.
        if !self.try_connect_drones_directly_to(
            command_device, 
            device_map,
            frequency
        ) {
            return;
        }

        if let Topology::Mesh = topology {
            self.try_connect_drones_to_each_other(device_map, frequency);
        }
    }

    #[must_use]
    fn try_connect_drones_directly_to(
        &mut self,
        command_device: &Device,
        device_map: &IdToDeviceMap,
        frequency: Megahertz
    ) -> bool {
        let command_device_node = self.0.add_node(command_device.id());
        let mut connected = false;

        // Avoiding loops on a command device.
        let mut commanded_device_map = device_map.clone();
        let _ = commanded_device_map.remove(&command_device.id());
        
        for device in commanded_device_map.devices() {
            if let Some(distance) = command_device.connection_distance(
                device,
                frequency
            ) {
                let node = self.0.add_node(device.id());
                
                self.0.add_edge(command_device_node, node, distance);
                
                connected = true;
            };
            if let Some(distance) = device.connection_distance(
                command_device,
                frequency
            ) {
                let node = self.0.add_node(device.id());
                
                self.0.add_edge(node, command_device_node, distance);
            };
        }

        connected
    }

    fn try_connect_drones_to_each_other(
        &mut self, 
        device_map: &IdToDeviceMap,
        frequency: Megahertz
    ) {
        for (i, tx_drone) in device_map.devices().enumerate() {
            let tx_node = self.0.add_node(tx_drone.id());
          
            // Loops are prohibited. 
            // Otherwise, shortest path algorithms will not function properly.
            for rx_drone in device_map.devices().skip(i + 1) {
                let rx_node = self.0.add_node(rx_drone.id());
        
                if let Some(distance) = tx_drone.connection_distance(
                    rx_drone,
                    frequency
                ) {
                    self.0.add_edge(tx_node, rx_node, distance);
                }
                if let Some(distance) = rx_drone.connection_distance(
                    tx_drone,
                    frequency
                ) {
                    self.0.add_edge(rx_node, tx_node, distance);
                }
            }
        }
    }
 
    /// # Panics
    /// 
    /// Will panic if `rustworkx_core::shortest_path::dijkstra` becomes 
    /// fallible.
    #[must_use]
    pub fn delays(
        &self, 
        command_device_id: DeviceId,
        delay_multiplier: f32
    ) -> DelaySnapshot {
        let distances = self.single_source_dijkstra(
            command_device_id
        ).unwrap();

        distances
            .iter()
            .map(|(device_id, distance)| {
                let delay = delay_to(*distance, delay_multiplier);
            
                (*device_id, delay)
            })
            .collect()
    }
   
    // Gives shortest distance to a device by distance between devices.
    /// # Errors
    ///
    /// Will never fail.
    pub fn single_source_dijkstra(
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
    pub fn find_shortest_path_from_to(
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
    pub fn all_incoming_degrees(&self) -> HashMap<DeviceId, usize> {
        self.0
            .nodes()
            .map(|node| {
                let incoming_degree = self.0
                    .neighbors_directed(node, petgraph::Direction::Incoming)
                    .count();

                (node, incoming_degree)
            })
            .collect()
    }

    #[must_use]
    pub fn all_outgoing_degrees(&self) -> HashMap<DeviceId, usize> {
        self.0
            .nodes()
            .map(|node| {
                let outgoing_degree = self.0
                    .neighbors_directed(node, petgraph::Direction::Outgoing)
                    .count();

                (node, outgoing_degree)
            })
            .collect()
    }
    
    // Translation of Python 3 NetworkX method `networkx.algorithms.cluster.\
    // clustering` to Rust.
    #[must_use]
    pub fn clustering(&self) -> HashMap<DeviceId, f32> {
        /*
        td_iter = _directed_weighted_triangles_and_degree_iter(G, nodes, weight)
        clusterc = {
            v: 0 if t == 0 else t / ((dt * (dt - 1) - 2 * db) * 2)
            for v, dt, db, t in td_iter
        }

        return clusterc
        */

        todo!()
    }

    #[must_use]
    fn directed_weighted_triangles_and_degree_iter(
        &self
    ) {
        todo!()
    }
    
    /// # Panics
    /// 
    /// Will panic if `rustworkx_core::shortest_path::dijkstra` becomes 
    /// fallible.
    #[must_use]
    pub fn diameter(&self) -> f32 {
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
    pub fn node_load(&self) -> Vec<Option<f64>> {
        betweenness_centrality(&self.0, true, true, 50)
    }

    // Translation of Python 3 NetworkX method `networkx.algorithms.centrality.\
    // percolation.percolation_centrality` to Rust.
    /// # Panics
    ///
    /// Will never panic.
    #[must_use]
    pub fn percolation_centrality(
        &self, 
        command_device_id: DeviceId,
        device_map: &IdToDeviceMap,
        infection_type: &InfectionType
    ) -> HashMap<DeviceId, f32> {
        let Some(command_device) = device_map.get(&command_device_id) else {
            return HashMap::new() 
        };
        let mut percolation_map = self.0
            .nodes()
            .map(|node| (node, 0.0))
            .collect();
        let percolation_states = self.percolation_states(
            command_device, 
            device_map,
            *infection_type
        );
        let percolation_sum = percolation_states
            .values()
            .fold(0.0, |acc, x| acc + x);

        for node in self.0.nodes() {
            let (
                stack, 
                path_map, 
                number_of_paths_through_map
            ) = self.single_source_dijkstra_path_basic(node);

            percolation_map = accumulate_percolation(
                percolation_map, 
                stack, 
                path_map, 
                &number_of_paths_through_map, 
                node, 
                &percolation_states, 
                percolation_sum
            );
        }

        let coefficient = 1.0 / (self.0.node_count() as f32 - 2.0);
        percolation_map
            .values_mut() 
            .for_each(|percolation| *percolation *= coefficient); 
        
        percolation_map
    }

    #[must_use]
    fn percolation_states(
        &self,
        command_device: &Device,
        device_map: &IdToDeviceMap,
        infection_type: InfectionType
    ) -> HashMap<DeviceId, f32> {
        let mut percolation_states: HashMap<DeviceId, f32> = self.0
            .nodes()
            .map(|node| {
                let percolation_state = match device_map.get(&node) {
                    Some(drone) => {
                        percolation_state_from_infection_state(
                            *drone.infection_state(&infection_type)
                        )
                    },
                    None => 0.0
                };

                (node, percolation_state)
            })
            .collect();

        // The command center is percolated, although it can not be infected for 
        // now (version 0.5.1).
        percolation_states.insert(command_device.id(), 1.0);

        percolation_states
    }

    // Translation of Python 3 NetworkX method `networkx.algorithms.\
    // centrality.betweenness._single_source_dijkstra_path_basic` to Rust.
    // It is a helper method for `ConnectionGraph::percolation_centrality`.
    /// # Panics
    ///
    /// Will never panic.
    #[must_use]
    fn single_source_dijkstra_path_basic(
        &self, 
        source: DeviceId
    ) -> DijkstraResultStruct {
        let mut stack = Vec::new();
        let mut distance_map = HashMap::new();
        let mut path_map = HashMap::new();
        let mut number_of_paths_through_map = HashMap::new();

        for node in self.0.nodes() {
            path_map.insert(node, Vec::new());
            number_of_paths_through_map.insert(node, 0.0);
        }
        number_of_paths_through_map.insert(source, 1.0);

        let mut seen_map = HashMap::from([(source, 0.0)]);
        let mut counter = 0;
        let mut heap = BinaryHeap::new();
        
        heap.push(HelperStruct::new(0.0, counter, source, source));
        counter += 1;

        while let Some(
            HelperStruct { distance, previous_node, current_node, ..}
        ) = heap.pop() {
            if distance_map.contains_key(&current_node) {
                continue;
            }

            let through_current = *number_of_paths_through_map
                .get(&current_node)
                .unwrap();
            let through_previous = *number_of_paths_through_map
                .get(&previous_node)
                .unwrap();
            number_of_paths_through_map.insert(
                current_node, 
                through_current + through_previous
            );

            stack.push(current_node);
            distance_map.insert(current_node, distance);

            for neighbor in self.0.neighbors(current_node) {
                let distance_between_nodes = self.0
                    .edge_weight(current_node, neighbor)
                    .unwrap();
                let neighbor_distance = distance + distance_between_nodes;

                let new_or_shorter_path = if let Some(
                    previous_distance
                ) = seen_map.get(&neighbor) {
                    neighbor_distance < *previous_distance
                } else {
                    true
                };

                if !distance_map.contains_key(&neighbor) 
                    && new_or_shorter_path 
                {
                    seen_map.insert(neighbor, neighbor_distance);
                    heap.push(
                        HelperStruct::new(
                            neighbor_distance, 
                            counter, 
                            current_node, 
                            neighbor
                        )
                    );
                    counter += 1;
                    number_of_paths_through_map.insert(neighbor, 0.0); 
                    path_map.insert(neighbor, vec![current_node]);
                } else if neighbor_distance == *seen_map
                    .get(&neighbor)
                    .unwrap()
                {
                    let through_neighbor = *number_of_paths_through_map
                        .get(&neighbor)
                        .unwrap();

                    number_of_paths_through_map.insert(
                        neighbor,
                        through_current + through_neighbor
                    );

                    let neighbor_path = path_map
                        .get_mut(&neighbor)
                        .unwrap();

                    neighbor_path.push(current_node);
                }
            }
        }

        (stack, path_map, number_of_paths_through_map)
    }
}

impl Default for ConnectionGraph {
    fn default() -> Self {
        Self(GraphMap::new())
    }
}


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::message::{Message, MessageType};
    use crate::signal::{
        GREEN_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE, NO_SIGNAL_LEVEL, 
        SignalLevel, SignalArea, WIFI_2_4GHZ_FREQUENCY
    };
    use crate::device::{Device, DeviceBuilder, UNKNOWN_ID};
    use crate::device::systems::{TRXModule, TRXSystem};
    use crate::mathphysics::Point3D;
    
    use super::*;
    

    const CC_TX_CONTROL_RADIUS: Meter    = 300.0;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const VERY_BIG_STRENGTH_VALUE: f32   = GREEN_SIGNAL_STRENGTH_VALUE * 1000.0;


    fn round_with_precision(value: f32, precision: u8) -> f32 {
        let coefficient = 10.0_f32.powi(precision.into());

        (value * coefficient).round() / coefficient
    }

    fn cc_tx_module() -> TRXModule {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let max_tx_signal_levels = HashMap::from([
            (frequency, SignalLevel::from(VERY_BIG_STRENGTH_VALUE))
        ]);
        let tx_signal_levels = HashMap::from([(
            frequency, 
            SignalLevel::from_area(
                SignalArea::build(CC_TX_CONTROL_RADIUS).unwrap(),
                frequency
            )
        )]);

        TRXModule::build(
            max_tx_signal_levels,
            tx_signal_levels
        ).unwrap()
    }
    
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

    fn drone_with_trx_system_set(position: Point3D) -> Device {
        DeviceBuilder::new()
            .set_global_position(position)
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: drone_tx_module(), 
                    rx_module: drone_rx_module()
                }
            )
            .set_vulnerabilities(&[InfectionType::Jamming])
            .build()
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn simple_mesh() -> (ConnectionGraph, Vec<DeviceId>) {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        // Network:
        //                      D
        //                      |
        //                    (7.28)
        //                      |
        //  A -(7.0)- B -(9.0)- C
        //                      |
        //                    (7.28)
        //                      |
        //                      E
        //
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
        
        let devices = [
            command_center,
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, 7.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, -7.0, 0.0)),
        ];
        let device_ids: Vec<DeviceId> = devices
            .iter()
            .map(|device| device.id())
            .collect();
    
        let device_map = IdToDeviceMap::from(devices);

        let mut connections = ConnectionGraph::new();
        connections.update(
            command_center_id, 
            &device_map, 
            Topology::Mesh, 
            frequency
        );

        (connections, device_ids)
    }

    fn simple_star() -> (ConnectionGraph, Vec<DeviceId>) {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        let command_center = DeviceBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: cc_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let command_center_id = command_center.id();

        let devices = [
            command_center,
            drone_with_trx_system_set(Point3D::new(25.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(0.0, 25.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(0.0, 0.0, 25.0)),
        ];
        let device_ids: Vec<DeviceId> = devices
            .iter()
            .map(|device| device.id())
            .collect();

        let device_map = IdToDeviceMap::from(devices);

        let mut connections = ConnectionGraph::new();
        connections.update(
            command_center_id, 
            &device_map, 
            Topology::Star, 
            frequency
        );

        (connections, device_ids)
    }


    #[test]
    fn create_star_connection_graph() {
        let (connections, device_ids) = simple_star(); 
        
        let cc_id = device_ids[0];
        let drone_b_id = device_ids[1];
        let drone_c_id = device_ids[2];
        let drone_d_id = device_ids[3];

        assert_eq!(3, connections.0.edge_count());

        assert!(connections.0.contains_edge(cc_id, drone_b_id));
        assert!(connections.0.contains_edge(cc_id, drone_c_id));
        assert!(connections.0.contains_edge(cc_id, drone_d_id));
    }

    #[test]
    fn create_mesh_connection_graph() {
        let (connections, device_ids) = simple_mesh(); 

        let cc_id = device_ids[0];
        let drone_b_id = device_ids[1];
        let drone_c_id = device_ids[2];
        let drone_d_id = device_ids[3];
        let drone_e_id = device_ids[4];

        assert_eq!(8, connections.0.edge_count());
        
        assert!(connections.0.contains_edge(cc_id, drone_b_id));
        assert!(connections.0.contains_edge(drone_b_id, cc_id));
        
        assert!(connections.0.contains_edge(drone_b_id, drone_c_id));
        assert!(connections.0.contains_edge(drone_c_id, drone_b_id));
        
        assert!(connections.0.contains_edge(drone_c_id, drone_d_id));
        assert!(connections.0.contains_edge(drone_d_id, drone_c_id));

        assert!(connections.0.contains_edge(drone_c_id, drone_e_id));
        assert!(connections.0.contains_edge(drone_e_id, drone_c_id));
    }

    #[test]
    fn all_degrees_in_mesh() {
        let (connections, device_ids) = simple_mesh(); 
        
        let cc_id = device_ids[0];
        let drone_b_id = device_ids[1];
        let drone_c_id = device_ids[2];
        let drone_d_id = device_ids[3];
        let drone_e_id = device_ids[4];
        
        let expected_incoming_degrees = HashMap::from([
            (cc_id, 1),
            (drone_b_id, 2),
            (drone_c_id, 3),
            (drone_d_id, 1),
            (drone_e_id, 1),
        ]);
        let expected_outgoing_degrees = HashMap::from([
            (cc_id, 1),
            (drone_b_id, 2),
            (drone_c_id, 3),
            (drone_d_id, 1),
            (drone_e_id, 1),
        ]);

        assert_eq!(
            expected_incoming_degrees, 
            connections.all_incoming_degrees()
        );
        assert_eq!(
            expected_outgoing_degrees, 
            connections.all_outgoing_degrees()
        );
    }
    
    #[test]
    fn all_degrees_in_star() {
        let (connections, device_ids) = simple_star(); 
        
        let cc_id = device_ids[0];
        let drone_b_id = device_ids[1];
        let drone_c_id = device_ids[2];
        let drone_d_id = device_ids[3];
        
        let expected_incoming_degrees = HashMap::from([
            (cc_id, 0),
            (drone_b_id, 1),
            (drone_c_id, 1),
            (drone_d_id, 1),
        ]);
        let expected_outgoing_degrees = HashMap::from([
            (cc_id, 3),
            (drone_b_id, 0),
            (drone_c_id, 0),
            (drone_d_id, 0),
        ]);

        assert_eq!(
            expected_incoming_degrees, 
            connections.all_incoming_degrees()
        );
        assert_eq!(
            expected_outgoing_degrees, 
            connections.all_outgoing_degrees()
        );
    }

    #[test]
    fn percolation_like_in_networkx() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
 
        // Network:
        //                      D
        //                      |
        //                    (7.28)
        //                      |
        //  A -(7.0)- B -(9.0)- C
        //                      |
        //                    (7.28)
        //                      |
        //                      E
        //
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
        
        let mut devices = [
            command_center,
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, 7.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(16.0, -7.0, 0.0)),
        ];
        let drone_b_id = devices[1].id();
        let drone_c_id = devices[2].id();
        let drone_d_id = devices[3].id();
        let drone_e_id = devices[4].id();

        // In order to infect the drone we need to allow receiving.
        devices[1].set_rx_signal_level(GREEN_SIGNAL_LEVEL, frequency);
        devices[2].set_rx_signal_level(GREEN_SIGNAL_LEVEL, frequency);
        
        let infection_message_for_b = Message::new(
            UNKNOWN_ID, 
            drone_b_id, 
            0, 
            MessageType::Infection(InfectionType::Jamming)
        );
        let infection_message_for_c = Message::new(
            UNKNOWN_ID, 
            drone_c_id, 
            0, 
            MessageType::Infection(InfectionType::Jamming)
        );

        assert!(
            devices[1]
                .receive_and_process_message(
                    &infection_message_for_b,
                    frequency, 
                ).is_ok()
        );
        assert!(devices[1].is_infected());
        assert!(
            devices[2]
                .receive_and_process_message(
                    &infection_message_for_c,
                    frequency, 
                ).is_ok()
        );
        assert!(devices[2].is_infected());
        
        devices[1].set_rx_signal_level(NO_SIGNAL_LEVEL, frequency);
        devices[2].set_rx_signal_level(NO_SIGNAL_LEVEL, frequency);

        let device_map = IdToDeviceMap::from(devices);

        let mut connections = ConnectionGraph::new();
        connections.update(
            command_center_id,
            &device_map,
            Topology::Mesh,
            frequency
        );

        let percolation_map: HashMap<DeviceId, f32> = connections
            .percolation_centrality(
                command_center_id,
                &device_map,
                &InfectionType::Jamming
            )
            .iter()
            .map(|(node, percolation)| 
                (*node, round_with_precision(*percolation, 4))
            )
            .collect();
        
        let expected_percolation_map = HashMap::from([
            (command_center_id, 0.0),
            (drone_b_id, 0.5238),
            (drone_c_id, 0.8571),
            (drone_d_id, 0.0),
            (drone_e_id, 0.0),
        ]);

        assert_eq!(percolation_map, expected_percolation_map);
    }

    #[test]
    fn check_delays() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;

        let command_center = DeviceBuilder::new()
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: cc_tx_module(),
                    rx_module: TRXModule::default() 
                }
            )
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let command_center_id = command_center.id();

        let distance = 50.0;
        let drone = DeviceBuilder::new()
            .set_global_position(Point3D::new(distance, 0.0, 0.0))
            .set_trx_system(
                TRXSystem::Strength { 
                    tx_module: TRXModule::default(),
                    rx_module: drone_rx_module() 
                }
            )
            .build()
            .unwrap_or_else(|error| panic!("{}", error));
        let drone_id = drone.id();

        let mut connections = ConnectionGraph::new();
        connections.update(
            command_center_id, 
            &IdToDeviceMap::from([command_center, drone]), 
            Topology::Mesh, 
            frequency
        );

        let expected_delays = HashMap::from([
            (command_center_id, 0),
            (drone_id, 0)
        ]);
        let no_delays = connections.delays(command_center_id, 0.0);

        assert!(expected_delays.eq(&no_delays));

        let expected_delays = HashMap::from([
            (command_center_id, 0),
            (drone_id, 250)
        ]);
        let delays = connections.delays(command_center_id, 1.0);

        assert!(expected_delays.eq(&delays));
    }

    #[test]
    fn network_diameter() {
        let frequency = WIFI_2_4GHZ_FREQUENCY;
        
        // Network 1: full mesh with edge weight 1.0.
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
        
        let devices1 = IdToDeviceMap::from([
            command_center.clone(),
            drone_with_trx_system_set(Point3D::new(1.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(2.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(3.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(4.0, 0.0, 0.0)),
        ]);

        let mut connections = ConnectionGraph::new();
        connections.update(
            command_center_id,
            &devices1,
            Topology::Mesh,
            frequency
        );

        assert_eq!(connections.diameter(), 4.0);

        // Network 2:
        // 
        // A -(7.0)- B -(7.0)- C -(7.0)- D -(7.0)- E 
        //
        let devices2 = IdToDeviceMap::from([
            command_center.clone(),
            drone_with_trx_system_set(Point3D::new(7.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(14.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(21.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(28.0, 0.0, 0.0)),
        ]);
        
        connections.update(
            command_center_id,
            &devices2,
            Topology::Mesh,
            frequency
        );

        assert_eq!(connections.diameter(), 28.0);
        
        // Network 3:
        //                      D
        //                      |
        //                    (7.28)
        //                      |
        //  A -(7.0)- B -(9.0)- C
        //                      |
        //                    (7.28)
        //                      |
        //                      E
        //
        let (connections, _) = simple_mesh();

        let diameter3 = round_with_precision(connections.diameter(), 2);

        assert_eq!(diameter3, 21.28);
    }
}
