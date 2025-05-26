use clap::{Arg, ArgAction, ArgMatches, Command};

use crate::backend::device::Topology;
use crate::backend::device::networkmodel::NetworkModelType;

use super::examples;


const ARG_DISPLAY_DELAYLESS_NETWORK: &str = "display delayless network";
const ARG_EXAMPLE_NUMBER: &str   = "example number";
const ARG_EXPERIMENT_TITLE: &str = "experiment title";
const ARG_INFECTION_TYPE: &str   = "infection type";
const ARG_NETWORK_MODEL: &str    = "network model";
const ARG_NETWORK_TOPOLOGY: &str = "network topology";
const ARG_SC_MOVEMENT: &str      = "signal color movement";
const ARG_PLOT_CAPTION: &str     = "plot caption";

const EXP_COMMAND_DELAYS: &str  = "delays";
const EXP_DOS: &str             = "dos";
const EXP_GPS_AND_CONTROL: &str = "control";
const EXP_GPS_ONLY: &str        = "gps";
const EXP_GPS_SPOOFING: &str    = "gpsspoof";
const EXP_SIGNAL_COLOR: &str    = "signals";
const EXP_INFECTION: &str       = "infection";

const INF_INDICATOR: &str = "indicator";
const INF_JAMMING: &str   = "jamming";

const NM_STATEFUL: &str    = "sf";
const NM_STATELESS: &str   = "sl";
const SLNM_DELAY_MULTIPLIER: &str = "delay multiplier";

const TOPOLOGY_MESH: &str = "mesh";
const TOPOLOGY_STAR: &str = "star";

const DEFAULT_DELAY_MULTIPLIER: &str = "0.0";
const DEFAULT_PLOT_CAPTION:     &str = "";


pub fn cli() {
    let matches = Command::new("drone_network")
        .version("0.13.2")
        .about("Models drone networks.")
        .arg(
            Arg::new(ARG_PLOT_CAPTION)
                .short('c')
                .long("caption")
                .default_value(DEFAULT_PLOT_CAPTION)
                .help("Set the plot caption")
        )
        .arg(
            Arg::new(ARG_EXAMPLE_NUMBER)
                .short('e')
                .long("example")
                .conflicts_with_all([
                    ARG_DISPLAY_DELAYLESS_NETWORK,
                    ARG_EXPERIMENT_TITLE,
                    ARG_INFECTION_TYPE,
                    ARG_NETWORK_MODEL,
                    ARG_NETWORK_TOPOLOGY,
                    ARG_SC_MOVEMENT,
                    ARG_PLOT_CAPTION,
                ])
                .value_parser(clap::value_parser!(u8))
                .help("Run an experiment by its number")
        )
        .arg(
            Arg::new(ARG_EXPERIMENT_TITLE)
                .short('x')
                .long("experiment")
                .requires(ARG_NETWORK_MODEL)
                .requires(ARG_NETWORK_TOPOLOGY)
                .requires_if(EXP_INFECTION, ARG_INFECTION_TYPE)
                .value_parser([
                    EXP_COMMAND_DELAYS,
                    EXP_DOS,
                    EXP_GPS_AND_CONTROL,
                    EXP_GPS_ONLY,
                    EXP_GPS_SPOOFING,
                    EXP_INFECTION,
                    EXP_SIGNAL_COLOR
                ])
                .help("Choose experiment title")
        )
        .arg(
            Arg::new(ARG_NETWORK_MODEL)
                .short('m')
                .long("network-model")
                .value_parser([NM_STATELESS, NM_STATEFUL])
                .help("Choose network model")
        )
        .arg(
            Arg::new(SLNM_DELAY_MULTIPLIER)
                .short('d')
                .long("delay-multiplier")
                .value_parser(clap::value_parser!(f32))
                .requires_if(NM_STATELESS, ARG_NETWORK_MODEL)
                .default_value(DEFAULT_DELAY_MULTIPLIER)
                .help("Set communication delay for complex network model")
        )
        .arg(
            Arg::new(ARG_NETWORK_TOPOLOGY)
                .short('t')
                .long("topology")
                .value_parser([TOPOLOGY_MESH, TOPOLOGY_STAR])
                .help("Choose network topology")
        )
        .arg(
            Arg::new(ARG_DISPLAY_DELAYLESS_NETWORK)
                .long("display-delayless")
                .action(ArgAction::SetTrue)
                .help("Show the same network model without delays as well")
        )
        .arg(
            Arg::new(ARG_SC_MOVEMENT)
                .long("dynamic")
                .action(ArgAction::SetTrue)
                .help("Show signal colors when drones are moving")
        )
        .arg(
            Arg::new(ARG_INFECTION_TYPE)
                .short('i')
                .long("infection")
                .value_parser([INF_INDICATOR, INF_JAMMING])
                .help("Choose infection type")
        )
        .arg_required_else_help(true)
        .get_matches();

    handle_arguments(&matches);
}

fn handle_arguments(matches: &ArgMatches) {
    if let Some(example_number) = matches.get_one::<u8>(ARG_EXAMPLE_NUMBER) {
        run_example_by_number(*example_number);
        return;
    }
    
    let Some(experiment_title) = matches
        .get_one::<String>(ARG_EXPERIMENT_TITLE)
    else {
        return;
    };

    let plot_caption = matches
        .get_one::<String>(ARG_PLOT_CAPTION)
        .unwrap();
    let display_delayless_network = *matches
        .get_one::<bool>(ARG_DISPLAY_DELAYLESS_NETWORK)
        .unwrap();
    let network_model = match matches
        .get_one::<String>(ARG_NETWORK_MODEL)
        .unwrap()
        .as_str()
    {
        NM_STATEFUL  => { 
            NetworkModelType::Stateful
        },
        NM_STATELESS => {
            let delay_multiplier = matches
                .get_one::<f32>(SLNM_DELAY_MULTIPLIER)
                .unwrap();

            NetworkModelType::Stateless(*delay_multiplier)
        },
        _ => return,
    };
    let topology = match matches
        .get_one::<String>(ARG_NETWORK_TOPOLOGY)
        .unwrap()
        .as_str()
    {
        TOPOLOGY_MESH => Topology::Mesh,
        TOPOLOGY_STAR => Topology::Star,
        _ => return,
    };

    let config = Config::new(
        plot_caption, 
        display_delayless_network, 
        network_model, 
        topology
    );

    match experiment_title.as_str() {
        EXP_COMMAND_DELAYS  => examples::command_delay(&config),
        EXP_DOS             => examples::dos(&config),
        EXP_GPS_AND_CONTROL => examples::gps_and_control(&config),
        EXP_GPS_ONLY        => examples::gps_only(&config),
        EXP_GPS_SPOOFING    => examples::gps_spoofing(&config),
        EXP_INFECTION       => {
            if INF_INDICATOR == matches
                .get_one::<String>(ARG_INFECTION_TYPE)
                .unwrap()
                .as_str()
            {
                examples::infection(&config);
            } else {
                examples::jamming_infection(&config);
            }
        }
        EXP_SIGNAL_COLOR    => {
            if *matches
                .get_one::<bool>(ARG_SC_MOVEMENT)
                .unwrap()
            {
                examples::signal_color_dynamic(&config);
            } else {
                examples::signal_color(&config);
            }
        }
        _ => ()
    }
}

fn run_example_by_number(example_number: u8) {
    match example_number {
        1  => examples::gps_only(
            &Config::new(
                "Stateless Model (Star)",
                false,
                NetworkModelType::Stateless(0.0),
                Topology::Star
            )
        ),
        2  => examples::gps_and_control(
            &Config::new(
                "Stateless Model (Star)",
                false,
                NetworkModelType::Stateless(0.0),
                Topology::Star
            )
        ),
        3  => examples::command_delay(
            &Config::new(
                "Stateless Model (Star)",
                true,
                NetworkModelType::Stateless(1.0),
                Topology::Star
            )
        ),
        4  => examples::signal_color(
            &Config::new(
                "Stateless Model (Star)",
                false,
                NetworkModelType::Stateless(0.0),
                Topology::Star
            )
        ),
        5  => examples::infection(
            &Config::new(
                "Stateless Model (Star)",
                false,
                NetworkModelType::Stateless(1.0),
                Topology::Star
            )
        ),
        6  => examples::dos(
            &Config::new(
                "Stateless Model (Star)",
                false,
                NetworkModelType::Stateless(1.0),
                Topology::Star
            )
        ),
        7  => examples::gps_only(
            &Config::new(
                "Stateless Model (Mesh)",
                false,
                NetworkModelType::Stateless(0.0),
                Topology::Mesh
            )
        ),
        8  => examples::gps_and_control(
            &Config::new(
                "Stateless Model (Mesh)",
                false,
                NetworkModelType::Stateless(0.0),
                Topology::Mesh
            )
        ),
        9  => examples::command_delay(
            &Config::new(
                "Stateless Model (Mesh)",
                true,
                NetworkModelType::Stateless(1.0),
                Topology::Mesh
            )
        ),
        10 => examples::signal_color(
            &Config::new(
                "Stateless Model (Mesh)",
                false,
                NetworkModelType::Stateless(0.0),
                Topology::Mesh
            )
        ),
        11 => examples::infection(
            &Config::new(
                "Stateless Model (Mesh)",
                false,
                NetworkModelType::Stateless(1.0),
                Topology::Mesh
            )
        ),
        12 => examples::dos(
            &Config::new(
                "Stateless Model (Mesh)",
                false,
                NetworkModelType::Stateless(1.0),
                Topology::Mesh
            )
        ),
        13 => examples::gps_only(
            &Config::new(
                "Stateful Model (Star)",
                false,
                NetworkModelType::Stateful,
                Topology::Star
            )
        ),
        14 => examples::gps_and_control(
            &Config::new(
                "Stateful Model (Star)",
                false,
                NetworkModelType::Stateful,
                Topology::Star
            )
        ),
        15 => examples::signal_color(
            &Config::new(
                "Stateful Model (Star)",
                false,
                NetworkModelType::Stateful,
                Topology::Star
            )
        ),
        16 => examples::infection(
            &Config::new(
                "Stateful Model (Star)",
                false,
                NetworkModelType::Stateful,
                Topology::Star
            )
        ),
        17 => examples::dos(
            &Config::new(
                "Stateful Model (Star)",
                false,
                NetworkModelType::Stateful,
                Topology::Star
            )
        ),
        18 => examples::gps_only(
            &Config::new(
                "Stateful Model (Mesh)",
                false,
                NetworkModelType::Stateful,
                Topology::Mesh
            )
        ),
        19 => examples::gps_and_control(
            &Config::new(
                "Stateful Model (Mesh)",
                false,
                NetworkModelType::Stateful,
                Topology::Mesh
            )
        ),
        20 => examples::signal_color(
            &Config::new(
                "Stateful Model (Mesh)",
                false,
                NetworkModelType::Stateful,
                Topology::Mesh
            )
        ),
        21 => examples::infection(
            &Config::new(
                "Stateful Model (Mesh)",
                false,
                NetworkModelType::Stateful,
                Topology::Mesh
            )
        ),
        22 => examples::dos(
            &Config::new(
                "Stateful Model (Mesh)",
                false,
                NetworkModelType::Stateful,
                Topology::Mesh
            )
        ),
        23 => examples::signal_loss_response(
            &Config::new(
                "Stateless Model (Mesh)",
                false,
                NetworkModelType::Stateless(0.0),
                Topology::Mesh
            )
        ),
        24 => examples::signal_loss_response(
            &Config::new(
                "Stateful Model (Mesh)",
                false,
                NetworkModelType::Stateful,
                Topology::Mesh
            )
        ),
        _ => ()
    }
}


pub enum AntennaType {
    Color,
    Strength
}


pub struct Config {
    pub plot_caption: String,
    pub display_delayless_network: bool, 
    pub network_model: NetworkModelType,
    pub topology: Topology,
}

impl Config {
    #[must_use]
    pub fn new(
        plot_caption: &str,
        display_delayless_network: bool,
        network_model: NetworkModelType,
        topology: Topology
    ) -> Self {
        Self {
            plot_caption: plot_caption.to_string(),
            display_delayless_network,
            network_model,
            topology
        }
    }

    #[must_use]
    pub fn antenna(&self) -> AntennaType {
        match self.network_model {
            NetworkModelType::Stateful     => AntennaType::Color,
            NetworkModelType::Stateless(_) => AntennaType::Strength,
        }
    }

    #[must_use]
    pub fn delay_multiplier(&self) -> f32 {
        match self.network_model {
            NetworkModelType::Stateful                    => 0.0,
            NetworkModelType::Stateless(delay_multiplier) => delay_multiplier,
        }
    }
}
