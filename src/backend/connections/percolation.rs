use std::collections::HashMap;

use crate::backend::device::DeviceId;
use crate::backend::mathphysics::Meter;
use crate::backend::malware::InfectionState;


// Translation of Python 3 NetworkX method `networkx.algorithms.\
// centrality.percolation._accumulate_percolation` to Rust.
// It is a helper method for `ConnectionGraph::percolation_centrality`.
#[must_use]
pub fn accumulate_percolation(
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
pub fn percolation_state_from_infection_state(
    infection_state: InfectionState
) -> f32 {
    match infection_state {
        InfectionState::Vulnerable => 1.0,
        InfectionState::Infected   => 0.5,
        InfectionState::Patched    => 0.0
    } 
}


// It is a helper struct for 
// `ConnectionGraph::single_source_dijkstra_path_basic`.
#[derive(Debug)]
pub struct HelperStruct {
    pub distance: Meter,
    pub counter: u32,
    pub previous_node: DeviceId,
    pub current_node: DeviceId
}

impl HelperStruct {
    #[must_use]
    pub fn new(
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
