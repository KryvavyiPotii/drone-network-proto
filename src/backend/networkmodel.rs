use std::collections::hash_map::Values;

use super::connections::Topology;
use super::device::{Device, DeviceId, IdToDeviceMap, IdToTaskMap};
use super::mathphysics::Megahertz;
use super::message::Message;

use attack::AttackerDevice;
use gps::GPS;


pub use stateful::StatefulModel;
pub use stateless::StatelessModel;


pub mod attack;
pub mod gps;
pub mod msgproc;
pub mod stateful;
pub mod stateless;


fn get_any_device<'a>(
    device_id: DeviceId,
    device_map: &'a IdToDeviceMap,
    attacker_devices: &'a [AttackerDevice],
    gps: &'a GPS,
) -> Option<&'a Device> {
    if device_id == gps.device().id() {
        return Some(gps.device());
    }
    if let Some(device) = device_map.get(&device_id) {
        return Some(device);
    }
    if let Some(attacker_device) = attacker_devices
        .iter()
        .find(|attacker_device| attacker_device.device().id() == device_id)
    {
        return Some(attacker_device.device());
    }

    None
}


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
    Stateless,
}


#[derive(Clone)]
pub struct NetworkModelBuilder {
    network_model_type: NetworkModelType,
    command_center_id: Option<DeviceId>,
    devices: Option<Vec<Device>>,
    attacker_devices: Option<Vec<AttackerDevice>>,
    gps: Option<GPS>,
    topology: Option<Topology>,
    scenario: Option<Vec<(Megahertz, Message)>>,
    delay_multiplier: Option<f32>,
}

impl NetworkModelBuilder {
    #[must_use]
    pub fn new(network_model_type: NetworkModelType) -> Self {
        Self {
            network_model_type,
            command_center_id: None,
            devices: None,
            attacker_devices: None,
            gps: None,
            topology: None,
            scenario: None,
            delay_multiplier: None,
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
    pub fn set_gps(mut self, gps: GPS) -> Self {
        self.gps = Some(gps);
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
    pub fn set_delay_multiplier(mut self, delay_multiplier: f32) -> Self {
        self.delay_multiplier = Some(delay_multiplier);
        self
    }

    #[must_use]
    pub fn build(self) -> NetworkModel {
        match self.network_model_type {
            NetworkModelType::Stateful  => self.build_stateful_model(),
            NetworkModelType::Stateless => self.build_stateless_model(),
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
            self.gps.unwrap_or_default(),
            &self.scenario.unwrap_or_default(),
            self.topology.unwrap_or_default(),
            self.delay_multiplier.unwrap_or_default(),
        );

        NetworkModel::Stateful(stateful_model)
    }

    fn build_stateless_model(self) -> NetworkModel {
        let device_map = IdToDeviceMap::from(
            self.devices
                .unwrap_or_default()
                .as_slice()
        );
        
        let stateless_model = StatelessModel::new(
            self.command_center_id.unwrap_or_default(),
            device_map,
            self.attacker_devices.unwrap_or_default(),
            self.gps.unwrap_or_default(),
            &self.scenario.unwrap_or_default(),
            self.topology.unwrap_or_default(),
            self.delay_multiplier.unwrap_or_default(),
        );

        NetworkModel::Stateless(stateless_model)
    }
}
