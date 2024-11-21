use std::collections::hash_map::Values;

use super::*;
use cellularautomaton::CellularAutomaton;
use complexnetwork::ComplexNetwork;


pub mod cellularautomaton;
pub mod complexnetwork;


#[derive(Clone, Copy)]
pub enum Topology {
    Star,
    Mesh,
}

pub enum RadarWarfareDeviceType {
    ComplexNetwork(complexnetwork::RadarWarfareDevice),
    CellularAutomaton(cellularautomaton::RadarWarfareDevice),
}

impl RadarWarfareDeviceType {
    pub fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        match self {
            Self::CellularAutomaton(rwd) => rwd.tx_signal_levels(),
            Self::ComplexNetwork(rwd) => rwd.tx_signal_levels(),
        }
    }

    pub fn position(&self) -> Coordinates3D {
        match self {
            Self::CellularAutomaton(rwd) => rwd.position(),
            Self::ComplexNetwork(rwd) => rwd.position(),
        }
    }

    pub fn area(&self) -> &SignalAreaType {
        match self {
            Self::CellularAutomaton(rwd) => rwd.area(),
            Self::ComplexNetwork(rwd) => rwd.area(),
        }
    }
}

pub enum DroneNetwork {
    ComplexNetwork(ComplexNetwork),
    CellularAutomaton(CellularAutomaton),
}

impl DroneNetwork {
    pub fn update_network(&mut self) {
        match self {
            Self::ComplexNetwork(complex_network) =>
                complex_network.update_network(),
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.update_network(),
        }
    }

    pub fn destination(&self) -> &Coordinates3D {
        match self {
            Self::ComplexNetwork(complex_network) =>
                &complex_network.destination(),
            Self::CellularAutomaton(cellular_automaton) =>
                &cellular_automaton.destination(),
        }
    }

    pub fn command_center(&self) -> &CommandCenter {
        match self {
            Self::ComplexNetwork(complex_network) =>
                complex_network.command_center(),
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.command_center(),
        }
    }

    pub fn command_center_mut(&mut self) -> &mut CommandCenter {
        match self {
            Self::ComplexNetwork(complex_network) =>
                complex_network.command_center_mut(),
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.command_center_mut(),
        }
    }
    
    pub fn drone_iter(&self) -> Values<'_, u32, Drone> { 
        match self {
            Self::ComplexNetwork(complex_network) =>
                complex_network.drone_iter(),
            Self::CellularAutomaton(cellular_automaton) =>
                cellular_automaton.drone_iter(),
        }
    }

    // TODO find a way to return an iterator without cloning
    pub fn radar_warfare_devices(&self) -> Vec<RadarWarfareDeviceType> { 
        match self {
            Self::ComplexNetwork(complex_network) => {
                complex_network
                    .rwd_iter()
                    .map(|rwd| RadarWarfareDeviceType::ComplexNetwork(
                        rwd.clone()
                    ))
                    .collect()
            },
            Self::CellularAutomaton(cellular_automaton) => {
                cellular_automaton
                    .rwd_iter()
                    .map(|rwd| RadarWarfareDeviceType::CellularAutomaton(
                        rwd.clone()
                    ))
                    .collect()
            }
        }
    }
}
