use std::collections::hash_map::Values;

use crate::backend::mathphysics::{Megahertz, Point3D};
use crate::backend::message::Message;

use super::{Device, DeviceId, IdToDeviceMap, IdToTaskMap};
use super::connections::Topology;

use attack::AttackerDevice;


pub use stateful::StatefulModel;
pub use stateless::StatelessModel;


pub mod attack;
pub mod stateful;
pub mod stateless;
pub mod msgproc;


#[derive(Clone)]
pub enum NetworkModel {
    Stateful(StatefulModel),
    Stateless(StatelessModel),
}

impl NetworkModel {
    #[must_use]
    pub fn stateful_model(&self) -> Option<&StatefulModel> {
        match self {
            Self::Stateful(stateful_model) => Some(stateful_model),
            Self::Stateless(_)             => None
        }
    }
    
    #[must_use]
    pub fn stateful_model_mut(&mut self) -> Option<&mut StatefulModel> {
        match self {
            Self::Stateful(stateful_model) => Some(stateful_model),
            Self::Stateless(_)             => None
        }
    }

    #[must_use]
    pub fn stateless_model(&self) -> Option<&StatelessModel> {
        match self {
            Self::Stateful(_)                => None,
            Self::Stateless(stateless_model) => Some(stateless_model),
        }
    }
    
    #[must_use]
    pub fn stateless_model_mut(&mut self) -> Option<&mut StatelessModel> {
        match self {
            Self::Stateful(_)                => None,
            Self::Stateless(stateless_model) => Some(stateless_model),
        }
    }

    pub fn update(&mut self) {
        match self {
            Self::Stateful(stateful_model)   => stateful_model.update(),
            Self::Stateless(stateless_model) => stateless_model.update(),
        }
    }

    #[must_use]
    pub fn device_tasks(&self) -> IdToTaskMap {
        match self {
            Self::Stateful(stateful_model)   => stateful_model.device_tasks(),
            Self::Stateless(stateless_model) => stateless_model.device_tasks(),
        }
    }

    #[must_use]
    pub fn command_device(&self) -> &Device {
        match self {
            Self::Stateful(stateful_model)   => 
                stateful_model.command_device(),
            Self::Stateless(stateless_model) => 
                stateless_model.command_device(),
        }
    }

    #[must_use]
    pub fn device_iter(&self) -> Values<'_, DeviceId, Device> { 
        match self {
            Self::Stateful(stateful_model)   => stateful_model.device_iter(),
            Self::Stateless(stateless_model) => stateless_model.device_iter(),
        }
    }

    #[must_use]
    pub fn device_count(&self) -> usize {
        match self {
            Self::Stateful(stateful_model)   => 
                stateful_model.device_count(),
            Self::Stateless(stateless_model) => 
                stateless_model.device_count(),
        }
    }
    
    #[must_use]
    pub fn attacker_devices(&self) -> &[AttackerDevice] { 
        match self {
            Self::Stateful(stateful_model)   => 
                stateful_model.attacker_devices(),
            Self::Stateless(stateless_model) => 
                stateless_model.attacker_devices(),
        }
    }
}


#[derive(Clone, Copy)]
pub enum NetworkModelType {
    Stateful,
    Stateless(f32), // delay multiplier
}


#[derive(Clone)]
pub struct NetworkModelBuilder {
    network_model_type: NetworkModelType,
    command_center_id: Option<DeviceId>,
    devices: Option<Vec<Device>>,
    attacker_devices: Option<Vec<AttackerDevice>>,
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
            attacker_devices: None,
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
    pub fn set_attacker_devices(
        mut self, 
        attacker_devices: &[AttackerDevice]
    ) -> Self {
        self.attacker_devices = Some(attacker_devices.to_vec());
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
        match self.network_model_type {
            NetworkModelType::Stateful                    =>
                self.build_stateful_model(),
            NetworkModelType::Stateless(delay_multiplier) =>
                self.build_stateless_model(delay_multiplier),
        }
    }

    fn build_stateful_model(self) -> NetworkModel {
        let device_map = IdToDeviceMap::from(
            self.devices
                .unwrap_or_default()
                .as_slice()
        );

        let stateful_model = StatefulModel::new(
            self.command_center_id.unwrap_or_default(),
            device_map,
            self.attacker_devices.unwrap_or_default(),
            &self.scenario.unwrap_or_default(),
            self.topology.unwrap_or_default(),
        );

        NetworkModel::Stateful(stateful_model)
    }

    fn build_stateless_model(self, delay_multiplier: f32) -> NetworkModel {
        let device_map = IdToDeviceMap::from(
            self.devices
                .unwrap_or_default()
                .as_slice()
        );
        
        let stateless_model = StatelessModel::new(
            self.command_center_id.unwrap_or_default(),
            device_map,
            self.attacker_devices.unwrap_or_default(),
            &self.scenario.unwrap_or_default(),
            self.topology.unwrap_or_default(),
            delay_multiplier
        );

        NetworkModel::Stateless(stateless_model)
    }
}
