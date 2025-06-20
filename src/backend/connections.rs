use std::collections::{BinaryHeap, HashMap};

use rustworkx_core::distancemap::DistanceMap;
use thiserror::Error;

use petgraph::Directed;
use petgraph::graphmap::{GraphMap, Neighbors}; 
use petgraph::visit::EdgeRef;
use rustworkx_core::centrality::betweenness_centrality;
use rustworkx_core::dictmap::DictMap;
use rustworkx_core::shortest_path::{astar, dijkstra};

use super::device::{
    Device, DeviceId, IdToDelayMap, IdToDeviceMap, BROADCAST_ID
};
use super::mathphysics::{delay_to, Megahertz, Meter, Position};
use super::malware::Malware;

use percolation::{
    HelperStruct, accumulate_percolation, percolation_state_from_infection_state
}; 


mod percolation;


type DijkstraResultStruct = (
    Vec<DeviceId>,
    HashMap<DeviceId, Vec::<DeviceId>>,
    HashMap<DeviceId, f32>,
);

const ERROR_MARGIN: f32 = 0.01;


#[derive(Error, Debug)]
pub enum ShortestPathError {
    #[error("Shortest path was not found")]
    NoPathFound,
    #[error("Path length is less than 2")]
    PathTooShort
}
    

fn unicast_delay_map_from_outside_network(
    destination_id: DeviceId,
    source_device: &Device,
    device_map: &IdToDeviceMap,
    delay_multiplier: f32,
) -> IdToDelayMap {
    let Some(destination_device) = device_map.get(&destination_id) else {
        return IdToDelayMap::new();
    };
    
    let delay = delay_to(
        source_device.distance_to(destination_device), 
        delay_multiplier
    );

    IdToDelayMap::from([(destination_id, delay)])
}


#[derive(Clone, Copy, Debug, Default)]
pub enum Topology {
    #[default]
    Star,
    Mesh,
}


#[derive(Clone, Debug, Default)]
pub struct ConnectionGraph {
    graph_map: GraphMap<DeviceId, Meter, Directed>,
    topology: Topology,
}

impl ConnectionGraph {
    #[must_use]
    pub fn new(topology: Topology) -> Self {
        Self { 
            graph_map: GraphMap::new(),
            topology
        }
    }

    #[must_use]
    pub fn neighbors(
        &self, 
        node: DeviceId
    ) -> Neighbors<'_, DeviceId, Directed> {
        self.graph_map.neighbors(node)
    }

    // Currently, it considers only distances between devices while building the 
    // most efficient paths. It ignores signal levels of devices.
    pub fn update(
        &mut self, 
        command_device_id: DeviceId,
        device_map: &IdToDeviceMap,
        frequency: Megahertz
    ) {
        self.graph_map.clear();
        
        let Some(command_device) = device_map.get(&command_device_id) else {
            return 
        };

        match self.topology {
            Topology::Star => 
                self.create_star(command_device, device_map, frequency),
            Topology::Mesh => 
                self.create_mesh(device_map, frequency),
        }
    }

    fn create_star(
        &mut self,
        central_device: &Device,
        device_map: &IdToDeviceMap,
        frequency: Megahertz
    ) {
        for (device_id, device) in device_map {
            // Loops are prohibited. Otherwise, shortest path algorithms will 
            // not function properly.
            if *device_id == central_device.id() {
                continue;
            }

            let distance = central_device.distance_to(device);

            if central_device.transmits_at(distance, frequency) {
                self.graph_map.add_edge(
                    central_device.id(), 
                    *device_id, 
                    distance
                );
            }
            if device.transmits_at(distance, frequency) {
                self.graph_map.add_edge(
                    *device_id, 
                    central_device.id(), 
                    distance
                );
            }
        }
    }

    fn create_mesh(
        &mut self, 
        device_map: &IdToDeviceMap,
        frequency: Megahertz
    ) {
        for (tx_id, tx) in device_map {
            for (rx_id, rx) in device_map {
                // Loops are prohibited. Otherwise, shortest path algorithms 
                // will not function properly.
                if tx_id == rx_id {
                    continue;
                }

                let distance = tx.distance_to(rx);
        
                if tx.transmits_at(distance, frequency) {
                    self.graph_map.add_edge(*tx_id, *rx_id, distance);
                }
                if rx.transmits_at(distance, frequency) {
                    self.graph_map.add_edge(*rx_id, *tx_id, distance);
                }
            }
        }
    }
    
    #[must_use]
    pub fn delay_map(
        &self,
        source_device: &Device,
        destination_id: DeviceId,
        device_map: &IdToDeviceMap,
        delay_multiplier: f32,
    ) -> IdToDelayMap {
        if self.graph_map.contains_node(source_device.id()) {
            self.delay_map_from_inside_network(
                source_device.id(), 
                destination_id,
                delay_multiplier
            )
        } else {
            self.delay_map_from_outside_network(
                source_device, 
                destination_id,
                device_map,
                delay_multiplier
            )        
        }
    }

    fn delay_map_from_inside_network(
        &self,
        source: DeviceId,
        destination: DeviceId,
        delay_multiplier: f32,
    ) -> IdToDelayMap {
        if destination == BROADCAST_ID {
            return self.broadcast_delay_map_from_inside_network(
                source, 
                delay_multiplier
            );
        }

        self.unicast_delay_map_from_inside_network(
            source, 
            destination, 
            delay_multiplier
        )
    }
    
    fn unicast_delay_map_from_inside_network(
        &self, 
        source: DeviceId,
        destination: DeviceId,
        delay_multiplier: f32
    ) -> IdToDelayMap {
        let distance_map = self
            .dijkstra(source, Some(destination))
            .unwrap();
        
        let distance = distance_map
            .get_item(destination)
            .unwrap();

        IdToDelayMap::from([
            (destination, delay_to(*distance, delay_multiplier))
        ])
    }
 
    fn broadcast_delay_map_from_inside_network(
        &self, 
        source: DeviceId,
        delay_multiplier: f32
    ) -> IdToDelayMap {
        let distance_map = self.dijkstra(source, None).unwrap();

        distance_map
            .iter()
            .map(|(device_id, distance)| {
                let delay = delay_to(*distance, delay_multiplier);
            
                (*device_id, delay)
            })
            .collect()
    }

    fn delay_map_from_outside_network(
        &self,
        source_device: &Device,
        destination_id: DeviceId,
        device_map: &IdToDeviceMap,
        delay_multiplier: f32,
    ) -> IdToDelayMap {
        if destination_id == BROADCAST_ID {
            return self.broadcast_delay_map_from_outside_network(
                destination_id, 
                source_device, 
                device_map, 
                delay_multiplier
            );
        } 

        unicast_delay_map_from_outside_network(
            destination_id, 
            source_device, 
            device_map, 
            delay_multiplier
        )
    }

    fn broadcast_delay_map_from_outside_network(
        &self,
        destination_id: DeviceId,
        source_device: &Device,
        device_map: &IdToDeviceMap,
        delay_multiplier: f32,
    ) -> IdToDelayMap {
        self.graph_map
            .nodes()
            .filter_map(|device_id| {
                let destination_device = device_map.get(&destination_id)?; 
                
                let delay = delay_to(
                    source_device.distance_to(destination_device), 
                    delay_multiplier
                );

                Some((device_id, delay))
            })
            .collect()
    }

    // Gives shortest distance to a device by distance between devices.
    /// # Errors
    ///
    /// Will never fail.
    pub fn dijkstra(
        &self,
        source: DeviceId,
        destination: Option<DeviceId>,
    ) -> rustworkx_core::Result<DictMap<DeviceId, f32>> {
        dijkstra(
            &self.graph_map,
            source,
            destination,
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
            &self.graph_map,
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
            return Err(ShortestPathError::PathTooShort);
        } 
        
        Ok((distance, path))
    }

    #[must_use]
    pub fn all_incoming_degrees(&self) -> HashMap<DeviceId, usize> {
        self.graph_map
            .nodes()
            .map(|node| {
                let incoming_degree = self.graph_map
                    .neighbors_directed(node, petgraph::Direction::Incoming)
                    .count();

                (node, incoming_degree)
            })
            .collect()
    }

    #[must_use]
    pub fn all_outgoing_degrees(&self) -> HashMap<DeviceId, usize> {
        self.graph_map
            .nodes()
            .map(|node| {
                let outgoing_degree = self.graph_map
                    .neighbors_directed(node, petgraph::Direction::Outgoing)
                    .count();

                (node, outgoing_degree)
            })
            .collect()
    }
    
    /// # Panics
    /// 
    /// Will panic if `rustworkx_core::shortest_path::dijkstra` becomes 
    /// fallible.
    #[must_use]
    pub fn diameter(&self) -> f32 {
        let shortest_paths: Vec<DictMap<DeviceId, f32>> = self.graph_map
            .nodes()
            .map(|drone_id| self.dijkstra(drone_id, None).unwrap())
            .collect();

        shortest_paths
            .iter()
            .flat_map(|dictmap| dictmap.values())
            .fold(0f32, |a, &b| a.max(b))
    }

    #[must_use]
    pub fn betweenness_centrality(&self) -> Vec<Option<f64>> {
        betweenness_centrality(&self.graph_map, true, true, 50)
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
        malware: &Malware
    ) -> HashMap<DeviceId, f32> {
        let Some(command_device) = device_map.get(&command_device_id) else {
            return HashMap::new() 
        };
        let mut percolation_map = self.graph_map
            .nodes()
            .map(|node| (node, 0.0))
            .collect();
        let percolation_states = self.percolation_states(
            command_device, 
            device_map,
            malware
        );
        let percolation_sum = percolation_states
            .values()
            .fold(0.0, |acc, x| acc + x);

        for node in self.graph_map.nodes() {
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

        let coefficient = 1.0 / (self.graph_map.node_count() as f32 - 2.0);
        percolation_map
            .values_mut() 
            .for_each(|percolation| *percolation *= coefficient); 
        
        percolation_map
    }

    fn percolation_states(
        &self,
        command_device: &Device,
        device_map: &IdToDeviceMap,
        malware: &Malware
    ) -> HashMap<DeviceId, f32> {
        let mut percolation_states: HashMap<DeviceId, f32> = self.graph_map
            .nodes()
            .map(|node| {
                let percolation_state = match device_map.get(&node) {
                    Some(drone) => {
                        let infection_state = *drone.infection_state(malware);

                        percolation_state_from_infection_state(
                            infection_state
                        )
                    },
                    None        => 0.0
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
    fn single_source_dijkstra_path_basic(
        &self, 
        source: DeviceId
    ) -> DijkstraResultStruct {
        let mut stack = Vec::new();
        let mut distance_map = HashMap::new();
        let mut path_map = HashMap::new();
        let mut number_of_paths_through_map = HashMap::new();

        for node in self.graph_map.nodes() {
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

            for neighbor in self.graph_map.neighbors(current_node) {
                let distance_between_nodes = self.graph_map
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
                } else {
                    let neighbor_seen = *seen_map
                        .get(&neighbor)
                        .unwrap();

                    if (neighbor_distance - neighbor_seen).abs() 
                        > ERROR_MARGIN 
                    {
                        continue;
                    }

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


#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::backend::CONTROL_FREQUENCY;
    use crate::backend::device::{Device, DeviceBuilder};
    use crate::backend::device::systems::{
        PowerSystem, TRXModule, TRXSystem, TRXSystemType
    };
    use crate::backend::malware::{Malware, MalwareType};
    use crate::backend::mathphysics::{Point3D, PowerUnit};
    use crate::backend::message::{Message, MessageType};
    use crate::backend::signal::{
        GREEN_SIGNAL_LEVEL, GREEN_SIGNAL_STRENGTH_VALUE, NO_SIGNAL_LEVEL, 
        SignalLevel, SignalArea,
    };
    
    use super::*;
    

    const CC_TX_CONTROL_RADIUS: Meter    = 300.0;
    const DEVICE_MAX_POWER: PowerUnit    = 1_000;
    const DRONE_TX_CONTROL_RADIUS: Meter = 10.0;
    const UNKNOWN_ID: DeviceId           = 0;
    const VERY_BIG_STRENGTH_VALUE: f32   = GREEN_SIGNAL_STRENGTH_VALUE * 1000.0;
    

    fn device_power_system() -> PowerSystem {
        PowerSystem::build(DEVICE_MAX_POWER, DEVICE_MAX_POWER)
            .unwrap_or_else(|error| panic!("{}", error))
    }

    fn round_with_precision(value: f32, precision: u8) -> f32 {
        let coefficient = 10.0_f32.powi(precision.into());

        (value * coefficient).round() / coefficient
    }

    fn cc_tx_module() -> TRXModule {
        let frequency = CONTROL_FREQUENCY;

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
        let frequency = CONTROL_FREQUENCY;
        
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
        let frequency = CONTROL_FREQUENCY;
        
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
        let trx_system = TRXSystem::new(
            TRXSystemType::Strength,
            drone_tx_module(), 
            drone_rx_module()
        );
        
        DeviceBuilder::new()
            .set_real_position(position)
            .set_power_system(device_power_system())
            .set_trx_system(trx_system)
            .build()
    }

    fn simple_mesh() -> (ConnectionGraph, Vec<DeviceId>) {
        let frequency = CONTROL_FREQUENCY;
        
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
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    drone_tx_module(), 
                    drone_rx_module()
                )
            )
            .build();
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

        let mut connections = ConnectionGraph::new(Topology::Mesh);
        connections.update(
            command_center_id, 
            &device_map, 
            frequency
        );

        (connections, device_ids)
    }

    fn simple_star() -> (ConnectionGraph, Vec<DeviceId>) {
        let frequency = CONTROL_FREQUENCY;
        
        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    cc_tx_module(),
                    drone_rx_module()
                )
            )
            .build();
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

        let mut connections = ConnectionGraph::new(Topology::Star);
        connections.update(
            command_center_id, 
            &device_map, 
            frequency
        );

        (connections, device_ids)
    }

    fn jamming_malware(jammed_frequency: Megahertz) -> Malware {
        Malware::new(
            0, 
            MalwareType::Jamming(jammed_frequency),
            false,
        )
    }


    #[test]
    fn create_star_connection_graph() {
        let (connections, device_ids) = simple_star(); 
        
        let cc_id = device_ids[0];
        let drone_b_id = device_ids[1];
        let drone_c_id = device_ids[2];
        let drone_d_id = device_ids[3];

        assert_eq!(3, connections.graph_map.edge_count());

        assert!(connections.graph_map.contains_edge(cc_id, drone_b_id));
        assert!(connections.graph_map.contains_edge(cc_id, drone_c_id));
        assert!(connections.graph_map.contains_edge(cc_id, drone_d_id));
    }

    #[test]
    fn create_mesh_connection_graph() {
        let (connections, device_ids) = simple_mesh(); 

        let cc_id = device_ids[0];
        let drone_b_id = device_ids[1];
        let drone_c_id = device_ids[2];
        let drone_d_id = device_ids[3];
        let drone_e_id = device_ids[4];

        assert_eq!(8, connections.graph_map.edge_count());
        
        assert!(connections.graph_map.contains_edge(cc_id, drone_b_id));
        assert!(connections.graph_map.contains_edge(drone_b_id, cc_id));
        
        assert!(connections.graph_map.contains_edge(drone_b_id, drone_c_id));
        assert!(connections.graph_map.contains_edge(drone_c_id, drone_b_id));
        
        assert!(connections.graph_map.contains_edge(drone_c_id, drone_d_id));
        assert!(connections.graph_map.contains_edge(drone_d_id, drone_c_id));

        assert!(connections.graph_map.contains_edge(drone_c_id, drone_e_id));
        assert!(connections.graph_map.contains_edge(drone_e_id, drone_c_id));
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
        let frequency = CONTROL_FREQUENCY;
        let malware   = jamming_malware(frequency);
 
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
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    drone_tx_module(), 
                    TRXModule::default() 
                )
            )
            .build();
        let command_center_id = command_center.id();
        
        let drone_trx_system = TRXSystem::new( 
            TRXSystemType::Strength,
            drone_tx_module(), 
            drone_rx_module()
        );
        
        let vulnerable_drone_builder = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(drone_trx_system)
            .set_vulnerabilities(&[malware]);
            
        let mut devices = [
            command_center,
            vulnerable_drone_builder
                .clone()
                .set_real_position(Point3D::new(7.0, 0.0, 0.0))
                .build(),
            vulnerable_drone_builder
                .clone()
                .set_real_position(Point3D::new(14.0, 0.0, 0.0))
                .build(),
            vulnerable_drone_builder
                .clone()
                .set_real_position(Point3D::new(16.0, 7.0, 0.0))
                .build(),
            vulnerable_drone_builder
                .clone()
                .set_real_position(Point3D::new(16.0, -7.0, 0.0))
                .build(),
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
            MessageType::Malware(malware)
        );
        let infection_message_for_c = Message::new(
            UNKNOWN_ID, 
            drone_c_id, 
            0, 
            MessageType::Malware(malware)
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

        let mut connections = ConnectionGraph::new(Topology::Mesh);
        connections.update(
            command_center_id,
            &device_map,
            frequency
        );

        let percolation_map: HashMap<DeviceId, f32> = connections
            .percolation_centrality(
                command_center_id,
                &device_map,
                &malware
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
        let frequency = CONTROL_FREQUENCY;

        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    cc_tx_module(),
                    TRXModule::default() 
                )
            )
            .build();
        let command_center_id = command_center.id();

        let distance = 50.0;
        let drone = DeviceBuilder::new()
            .set_real_position(Point3D::new(distance, 0.0, 0.0))
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    TRXModule::default(),
                    drone_rx_module() 
                )
            )
            .build();
        let drone_id = drone.id();
        let device_map = &IdToDeviceMap::from([command_center, drone]);

        let mut connections = ConnectionGraph::new(Topology::Mesh);
        connections.update(
            command_center_id, 
            &device_map,            
            frequency
        );

        let expected_delays = HashMap::from([
            (command_center_id, 0),
            (drone_id, 0)
        ]);
        let no_delays = connections.delay_map(
            device_map.get(&command_center_id).unwrap(), 
            BROADCAST_ID,
            &device_map,
            0.0
        );

        assert!(expected_delays.eq(&no_delays));

        let expected_delays = HashMap::from([
            (command_center_id, 0),
            (drone_id, 250)
        ]);
        let delays = connections.delay_map(
            device_map.get(&command_center_id).unwrap(), 
            BROADCAST_ID,
            &device_map,
            1_500_000.0
        );

        assert!(expected_delays.eq(&delays));
    }

    #[test]
    fn network_diameter() {
        let frequency = CONTROL_FREQUENCY;
        
        // Network 1: full mesh with edge weight 1.0.
        let command_center = DeviceBuilder::new()
            .set_power_system(device_power_system())
            .set_trx_system(
                TRXSystem::new( 
                    TRXSystemType::Strength,
                    drone_tx_module(), 
                    TRXModule::default() 
                )
            )
            .build();
        let command_center_id = command_center.id();
        
        let devices1 = IdToDeviceMap::from([
            command_center.clone(),
            drone_with_trx_system_set(Point3D::new(1.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(2.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(3.0, 0.0, 0.0)),
            drone_with_trx_system_set(Point3D::new(4.0, 0.0, 0.0)),
        ]);

        let mut connections = ConnectionGraph::new(Topology::Mesh);
        connections.update(
            command_center_id,
            &devices1,
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
