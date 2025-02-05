use std::collections::hash_map::Values;

use petgraph::{graphmap::GraphMap, Directed};
use rustworkx_core::{
    centrality::betweenness_centrality, 
    dictmap::DictMap, 
    shortest_path::{astar, dijkstra}, 
};

use super::*;

use cellularautomaton::{CellularAutomaton, CellularAutomatonRWD};
use complexnetwork::{ComplexNetwork, ComplexNetworkRWD};


pub use cellularautomaton::{
    CellularAutomatonBuilder, 
    CellularAutomatonRWDBuilder
};
pub use complexnetwork::{
    ComplexNetworkBuilder, 
    ComplexNetworkRWDBuilder
};


pub mod cellularautomaton;
pub mod complexnetwork;


pub fn get_drone_networks_destinations(
    drone_networks: &[NetworkModel]
) -> Vec<&Point3D> {
    drone_networks
        .iter()
        .map(|drone_network| drone_network.destination())
        .collect()
}

fn try_find_best_signal_levels<FC, F>(
    command_center: &CommandCenter,
    drones: &IdToDroneMap,
    connections: &ConnectionGraph,
    signal_type: &SignalType,
    try_set_best_signal_level_with_cc: FC,
    try_set_best_signal_level: F
) -> Result<HashMap<DeviceId, SignalLevel>, &'static str>
where
// Two functions were used instead of Box<_> because the Receiver and 
// Transmitter trait objects cannot be built.
    FC: Fn( 
        &CommandCenter,
        &Drone,
        &mut HashMap<DeviceId, SignalLevel>,
        &SignalType
    ),
    F: Fn( 
        &Drone,
        &Drone,
        &mut HashMap<DeviceId, SignalLevel>,
        &SignalType
    )
{
    let mut best_signal_levels = HashMap::new();

    for drone in drones.drones() {
        let path = match connections.find_shortest_path_from_to(
            command_center.id(),
            drone.id()
        ) {
            Some((_, shortest_path)) => shortest_path,
            None => continue
        };

        if path.len() < 2 {
            continue;
        }

        let first_drone = match drones.get(&path[1]) {
            Some(drone) => drone,
            None => return Err("Missing a drone")
        };

        try_set_best_signal_level_with_cc(
            command_center, 
            first_drone,
            &mut best_signal_levels,
            signal_type
        );

        // Skipping last element to avoid reading out of bounds.
        for i in 1..(path.len() - 1) {
            let tx_id = path[i];
            let rx_id = path[i + 1];
            
            let tx_drone = match drones.get(&tx_id) {
                Some(drone) => drone,
                None => break
            };
            let rx_drone = match drones.get(&rx_id) {
                Some(drone) => drone,
                None => break
            };

            try_set_best_signal_level(
                tx_drone,
                rx_drone,
                &mut best_signal_levels,
                signal_type
            );
        }
    }

    Ok(best_signal_levels)
}


pub enum NetworkModel {
    ComplexNetwork(ComplexNetwork),
    CellularAutomaton(CellularAutomaton),
}

impl NetworkModel {
    pub fn update(&mut self) {
        match self {
            Self::CellularAutomaton(cellular_automaton) => {
                cellular_automaton.update()},
            Self::ComplexNetwork(complex_network) => {
                complex_network.update()
            }
        }
    }

    pub fn destination(&self) -> &Point3D {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                &cellular_automaton.destination(),
            Self::ComplexNetwork(complex_network) =>
                &complex_network.destination()
        }
    }

    pub fn command_center(&self) -> &CommandCenter {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.command_center(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.command_center()
        }
    }

    pub fn drone_iter(&self) -> Values<'_, DeviceId, Drone> { 
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.drone_iter(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.drone_iter()
        }
    }

    pub fn drone_count(&self) -> usize {
        match self {
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.drone_count(),
            Self::ComplexNetwork(complex_network) =>
                complex_network.drone_count()
        }
    }
    
    pub fn rwds(&self) -> Vec<RWDType> { 
        match self {
            Self::CellularAutomaton(cellular_automaton) => 
                cellular_automaton.rwds(),
            Self::ComplexNetwork(complex_network) => 
                complex_network.rwds()
        }
    }
}


// Temporary interface for accessing network RWDs.
pub enum RWDType<'a> {
    CellularAutomaton(&'a CellularAutomatonRWD),
    ComplexNetwork(&'a ComplexNetworkRWD)
}

impl<'a> RWDType<'a> {
    pub fn position(&self) -> &Point3D {
        match self {
            Self::CellularAutomaton(rwd) => rwd.position(),
            Self::ComplexNetwork(rwd) => rwd.position()
        }
    }

    pub fn tx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        match self {
            Self::CellularAutomaton(rwd) => rwd.tx_signal_level(signal_type),
            Self::ComplexNetwork(rwd) => rwd.tx_signal_level(signal_type)
        }
    }

    pub fn area(&self, signal_type: &SignalType) -> SignalArea {
        match self {
            Self::CellularAutomaton(rwd) => rwd.area(signal_type),
            Self::ComplexNetwork(rwd) => rwd.area(signal_type)
        }
    }
}



#[derive(Clone, Debug)]
struct ConnectionGraph(GraphMap<DeviceId, Meter, Directed>);

impl ConnectionGraph {
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

    fn contains_device(&self, id: DeviceId) -> bool {
        self.0.contains_node(id)
    }

    fn clear(&mut self) {
        self.0.clear();
    }

    fn update(
        &mut self, 
        command_center: &CommandCenter,
        drones: &IdToDroneMap,
        topology: Topology
    ) {
        self.clear();
       
        // The drones in the command center area repeat the signal to each
        // other and the drones outside the area.
        // If there are no drones inside then the drones outside can not get 
        // any signal.
        // So, it is pointless to continue computation.
        if !self.try_connect_drones_directly_to_cc(command_center, drones) {
            return;
        }

        if let Topology::Mesh = topology {
            self.try_connect_drones_to_each_other(drones);
        }
    }
 
    fn try_connect_drones_directly_to_cc(
        &mut self,
        command_center: &CommandCenter,
        drones: &IdToDroneMap
    ) -> bool {
        let mut connected = false;
        let cc_node = self.add_device(command_center.id());
        
        for drone in drones.drones() {
            if let Some(distance) = command_center.connection_distance(
                drone,
                &SignalType::Control
            ) {
                let node = self.add_device(drone.id());
                
                self.add_connection(cc_node, node, distance);
                
                connected = true;
            };
            if let Some(distance) = drone.connection_distance(
                command_center,
                &SignalType::Control
            ) {
                let node = self.add_device(drone.id());
                
                self.add_connection(node, cc_node, distance);
            };
        }

        connected
    }

    fn try_connect_drones_to_each_other(&mut self, drones: &IdToDroneMap) {
        for tx_drone in drones.drones() {
            let tx_node = self.add_device(tx_drone.id());
           
            // Iterating twice without skipping to cover the case when drones
            // have different broadcast areas. In this case, drone A may be
            // able to broadcast signal to drone B, but B cannot broadcast
            // signal to A.
            for rx_drone in drones.drones() {
                let rx_node = self.add_device(rx_drone.id());
        
                if let Some(distance) = tx_drone.connection_distance(
                    rx_drone,
                    &SignalType::Control
                ) {
                    self.add_connection(tx_node, rx_node, distance);
                }
                if let Some(distance) = rx_drone.connection_distance(
                    tx_drone,
                    &SignalType::Control
                ) {
                    self.add_connection(rx_node, tx_node, distance);
                }
            }
        }
    }
    
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

    fn find_shortest_path_from_to(
        &self, 
        source: DeviceId,
        destination: DeviceId 
    ) -> Option<(f32, Vec<DeviceId>)> {
        astar(
            &self.0,
            source,
            |finish| -> rustworkx_core::Result<bool> {
                Ok(finish == destination)
            },
            |edge| Ok(*edge.2),
            |_| Ok(0.0)
        ).expect("Failed to find the shortest path")
    }

    fn diameter(&self) -> Option<f32> {
        let shortest_paths: Vec<DictMap<DeviceId, f32>> = self.0
            .nodes()
            .map(|drone_id|
                self
                    .single_source_dijkstra(drone_id)
                    .unwrap()
            )
            .collect();

        let diameter = shortest_paths
            .iter()
            .flat_map(|dictmap| dictmap.values())
            .fold(0f32, |a, &b| a.max(b));
        
        Some(diameter)
    }

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
    use super::*;
    
    const DRONE_TX_CONTROL_RADIUS: f32 = 10.0;

    fn drone_with_trx_system_set(position: Point3D) -> Drone {
        DroneBuilder::new()
            .set_global_position(position)
            .set_tx_module(
                TRXModule::build(
                    AntennaType::Strength,
                    HashMap::from([(SignalType::Control, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(
                        SignalType::Control, 
                        SignalLevel::from(
                            SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap()
                        )
                    )]),
                ).unwrap()
            )
            .set_rx_module(
                TRXModule::build(
                    AntennaType::Strength,
                    HashMap::from([(SignalType::Control, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(SignalType::Control, NO_SIGNAL_LEVEL)]),
                ).unwrap()
            )
            .build()
    }

    #[test]
    fn network_diameter() {
        // Network 1: full mesh with edge weight 1.0.
        let command_center = CommandCenterBuilder::new()
            .set_tx_module(
                TRXModule::build(
                    AntennaType::Strength,
                    HashMap::from([(SignalType::Control, GREEN_SIGNAL_LEVEL)]),
                    HashMap::from([(
                        SignalType::Control, 
                        SignalLevel::from(
                            SignalArea::build(DRONE_TX_CONTROL_RADIUS).unwrap()
                        )
                    )])
                ).unwrap()
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
            Topology::Mesh
        );

        assert_eq!(connections.diameter().unwrap(), 4.0);

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
            Topology::Mesh
        );

        assert_eq!(connections.diameter().unwrap(), 28.0);
        
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
            Topology::Mesh
        );

        assert_eq!(connections.diameter().unwrap(), 21.0);
    }
}
