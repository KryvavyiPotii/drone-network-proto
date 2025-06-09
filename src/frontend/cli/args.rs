use clap::ArgMatches;

use crate::backend::connections::Topology;
use crate::backend::device::systems::TRXSystemType;
use crate::backend::malware::{Malware, MalwareType};
use crate::backend::mathphysics::Millisecond;
use crate::backend::CONTROL_FREQUENCY;
use crate::frontend::{MALWARE_INFECTION_DELAY, MALWARE_SPREADS};
use crate::frontend::examples::{Example, DEVICE_MAX_POWER};
use crate::frontend::renderer::{Pixel, PlotResolution};

use super::{GeneralConfig, ModelConfig, RenderConfig};


pub const ARG_DELAY_MULTIPLIER: &str = "delay multiplier";
pub const ARG_DISPLAY_DELAYLESS_NETWORK: &str   = "display delayless network";
pub const ARG_DISPLAY_MALWARE_PROPAGATION: &str = "display malware propagation";
pub const ARG_DRONE_COUNT: &str      = "drone count";
pub const ARG_EXPERIMENT_TITLE: &str = "experiment title";
pub const ARG_MALWARE_TYPE: &str     = "malware type";
pub const ARG_NETWORK_TOPOLOGY: &str = "network topology";
pub const ARG_SIM_TIME: &str         = "simulation time";
pub const ARG_TRX_SYSTEM: &str       = "trx system";
pub const ARG_PLOT_CAPTION: &str     = "plot caption";
pub const ARG_PLOT_HEIGHT: &str      = "plot height";
pub const ARG_PLOT_WIDTH: &str       = "plot width";

pub const EXP_COMMAND_DELAYS: &str    = "delays";
pub const EXP_GPS_AND_CONTROL: &str   = "control";
pub const EXP_GPS_ONLY: &str          = "gps";
pub const EXP_GPS_SPOOFING: &str      = "gpsspoof";
pub const EXP_SIGNAL_COLOR: &str      = "signals";
pub const EXP_SIGNAL_LOSS: &str       = "signalloss";
pub const EXP_MALWARE_INFECTION: &str = "malware";

pub const MAL_DOS: &str       = "dos";
pub const MAL_INDICATOR: &str = "indicator";
pub const MAL_JAMMING: &str   = "jamming";

pub const TOPOLOGY_BOTH: &str = "both";
pub const TOPOLOGY_MESH: &str = "mesh";
pub const TOPOLOGY_STAR: &str = "star";

pub const TRX_BOTH: &str     = "both";
pub const TRX_COLOR: &str    = "color";
pub const TRX_STRENGTH: &str = "strength";

pub const DEFAULT_DELAY_MULTIPLIER: &str = "0.0";
pub const DEFAULT_DRONE_COUNT: &str  = "100";
pub const DEFAULT_PLOT_CAPTION: &str = "";
pub const DEFAULT_PLOT_HEIGHT: &str  = "300";
pub const DEFAULT_PLOT_WIDTH: &str   = "400";
pub const DEFAULT_SIM_TIME: &str     = "15000";


pub fn handle_arguments(matches: &ArgMatches) {
    let Some(experiment_title) = matches.get_one::<String>(
        ARG_EXPERIMENT_TITLE
    ) else {
        return;
    };

    let plot_resolution = PlotResolution::new(
        plot_width(matches), 
        plot_height(matches)
    );

    let model_config = ModelConfig::new(
        trx_system_type(matches), 
        topology(matches),
        drone_count(matches),
        delay_multiplier(matches),
        malware(matches),
    );   
    let render_config = RenderConfig::new(
        plot_caption(matches), 
        plot_resolution, 
        display_delayless_network(matches), 
        display_malware_propagation(matches),
        simulation_time(matches),
    );
    let general_config = GeneralConfig::new(model_config, render_config);
    
    let example = match experiment_title.as_str() {
        EXP_COMMAND_DELAYS    => Example::CommandDelays,
        EXP_GPS_AND_CONTROL   => Example::GPSAndControlEWDs,
        EXP_GPS_ONLY          => Example::GPSEWD,
        EXP_GPS_SPOOFING      => Example::GPSSpoofing,
        EXP_MALWARE_INFECTION => Example::MalwareInfection, 
        EXP_SIGNAL_COLOR      => Example::SignalColor, 
        EXP_SIGNAL_LOSS       => Example::SignalLossResponse,
        _ => return
    };

    example.execute(&general_config);
}

fn plot_width(matches: &ArgMatches) -> Pixel {
    *matches
        .get_one::<Pixel>(ARG_PLOT_WIDTH)
        .unwrap()
}

fn plot_height(matches: &ArgMatches) -> Pixel {
    *matches
        .get_one::<Pixel>(ARG_PLOT_HEIGHT)
        .unwrap()
}

fn simulation_time(matches: &ArgMatches) -> Millisecond {
    *matches
        .get_one::<Millisecond>(ARG_SIM_TIME)
        .unwrap()
}

fn plot_caption(matches: &ArgMatches) -> &str {
    matches
        .get_one::<String>(ARG_PLOT_CAPTION)
        .unwrap()
}

fn display_delayless_network(matches: &ArgMatches) -> bool {
    *matches
        .get_one::<bool>(ARG_DISPLAY_DELAYLESS_NETWORK)
        .unwrap()
}

fn display_malware_propagation(matches: &ArgMatches) -> bool {
    *matches
        .get_one::<bool>(ARG_DISPLAY_MALWARE_PROPAGATION)
        .unwrap()
}

fn trx_system_type(matches: &ArgMatches) -> Option<TRXSystemType> {
    let trx_system_type = match matches
        .get_one::<String>(ARG_TRX_SYSTEM) 
        .unwrap()
        .as_str() 
    {
        TRX_COLOR    => TRXSystemType::Color,
        TRX_STRENGTH => TRXSystemType::Strength,
        _ => return None,
    };

    Some(trx_system_type)
}

fn drone_count(matches: &ArgMatches) -> usize {
    *matches
        .get_one::<usize>(ARG_DRONE_COUNT)
        .unwrap()
}

fn topology(matches: &ArgMatches) -> Option<Topology> {
    let topology = match matches
        .get_one::<String>(ARG_NETWORK_TOPOLOGY)
        .unwrap()
        .as_str()
    {
        TOPOLOGY_STAR => Topology::Star,
        TOPOLOGY_MESH => Topology::Mesh,
        _ => return None,
    };

    Some(topology)
}

fn delay_multiplier(matches: &ArgMatches) -> f32 {
    *matches
        .get_one::<f32>(ARG_DELAY_MULTIPLIER)
        .unwrap()
}

fn malware(matches: &ArgMatches) -> Option<Malware> {
    let malware_type_name = matches.get_one::<String>(ARG_MALWARE_TYPE)?;
        
    let malware_type = match malware_type_name.as_str() {
        MAL_DOS => MalwareType::DoS(DEVICE_MAX_POWER),
        MAL_INDICATOR => MalwareType::Indicator,
        MAL_JAMMING => MalwareType::Jamming(CONTROL_FREQUENCY),
        _ => return None,
    };

    let malware = Malware::new(
        MALWARE_INFECTION_DELAY, 
        malware_type, 
        MALWARE_SPREADS
    );

    Some(malware)
}
