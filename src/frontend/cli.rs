use clap::{Arg, ArgAction, ArgMatches, Command};

use crate::backend::connections::Topology;
use crate::backend::device::systems::TRXSystemType;
use crate::backend::mathphysics::Millisecond;

use super::examples;


const ARG_DELAY_MULTIPLIER: &str = "delay multiplier";
const ARG_DISPLAY_DELAYLESS_NETWORK: &str = "display delayless network";
const ARG_EXAMPLE_NUMBER: &str   = "example number";
const ARG_EXPERIMENT_TITLE: &str = "experiment title";
const ARG_INFECTION_TYPE: &str   = "infection type";
const ARG_NETWORK_TOPOLOGY: &str = "network topology";
const ARG_SC_MOVEMENT: &str      = "signal color movement";
const ARG_SIM_TIME: &str         = "simulation time";
const ARG_TRX_SYSTEM: &str       = "trx system";
const ARG_PLOT_CAPTION: &str     = "plot caption";
const ARG_PLOT_HEIGHT: &str      = "plot height";
const ARG_PLOT_WIDTH: &str       = "plot width";

const EXP_COMMAND_DELAYS: &str  = "delays";
const EXP_DOS: &str             = "dos";
const EXP_GPS_AND_CONTROL: &str = "control";
const EXP_GPS_ONLY: &str        = "gps";
const EXP_GPS_SPOOFING: &str    = "gpsspoof";
const EXP_SIGNAL_COLOR: &str    = "signals";
const EXP_INFECTION: &str       = "infection";

const INF_INDICATOR: &str = "indicator";
const INF_JAMMING: &str   = "jamming";

const TOPOLOGY_MESH: &str = "mesh";
const TOPOLOGY_STAR: &str = "star";

const TRX_COLOR: &str    = "color";
const TRX_STRENGTH: &str = "strength";

const DEFAULT_DELAY_MULTIPLIER: &str = "0.0";
const DEFAULT_PLOT_CAPTION: &str = "";
const DEFAULT_PLOT_HEIGHT: &str  = "300";
const DEFAULT_PLOT_WIDTH: &str   = "400";
const DEFAULT_SIM_TIME: &str     = "15000";


pub fn cli() {
    let matches = Command::new("drone_network")
        .version("0.15.0")
        .about("Models drone networks.")
        .arg(
            Arg::new(ARG_PLOT_CAPTION)
                .short('c')
                .long("caption")
                .default_value(DEFAULT_PLOT_CAPTION)
                .help("Set the plot caption")
        )
        .arg(
            Arg::new(ARG_PLOT_WIDTH)
                .long("width")
                .requires(ARG_PLOT_HEIGHT)
                .value_parser(clap::value_parser!(u16))
                .default_value(DEFAULT_PLOT_WIDTH)
                .help("Set the plot width")
        )
        .arg(
            Arg::new(ARG_PLOT_HEIGHT)
                .long("height")
                .requires(ARG_PLOT_WIDTH)
                .value_parser(clap::value_parser!(u16))
                .default_value(DEFAULT_PLOT_HEIGHT)
                .help("Set the plot height")
        )
        .arg(
            Arg::new(ARG_SIM_TIME)
                .long("time")
                .value_parser(clap::value_parser!(Millisecond))
                .default_value(DEFAULT_SIM_TIME)
                .help("Set the simulation time")
        )
        .arg(
            Arg::new(ARG_EXAMPLE_NUMBER)
                .short('e')
                .long("example")
                .conflicts_with_all([
                    ARG_DISPLAY_DELAYLESS_NETWORK,
                    ARG_EXPERIMENT_TITLE,
                    ARG_INFECTION_TYPE,
                    ARG_NETWORK_TOPOLOGY,
                    ARG_SC_MOVEMENT,
                    ARG_TRX_SYSTEM,
                    ARG_PLOT_CAPTION,
                ])
                .value_parser(clap::value_parser!(u8))
                .help("Run an experiment by its number")
        )
        .arg(
            Arg::new(ARG_EXPERIMENT_TITLE)
                .short('x')
                .long("experiment")
                .requires(ARG_TRX_SYSTEM)
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
            Arg::new(ARG_TRX_SYSTEM)
                .long("trx")
                .value_parser([TRX_COLOR, TRX_STRENGTH])
                .help("Choose device TRX system type")
        )
        .arg(
            Arg::new(ARG_DELAY_MULTIPLIER)
                .short('d')
                .long("delay-multiplier")
                .value_parser(clap::value_parser!(f32))
                .default_value(DEFAULT_DELAY_MULTIPLIER)
                .help("Set signal transmission delay multiplier")
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
                .help(
                    format!(
                        "Show the same network model without delays as \
                        well (\"{EXP_COMMAND_DELAYS}\" experiment)" 
                    )
                )
        )
        .arg(
            Arg::new(ARG_SC_MOVEMENT)
                .long("dynamic")
                .action(ArgAction::SetTrue)
                .help(
                    format!(
                        "Show signal colors when drones are moving \
                        (\"{EXP_SIGNAL_COLOR}\" experiment)"
                    )
                )
        )
        .arg(
            Arg::new(ARG_INFECTION_TYPE)
                .short('i')
                .long("infection")
                .value_parser([INF_INDICATOR, INF_JAMMING])
                .help(
                    format!(
                        "Choose infection type \
                        (\"{EXP_INFECTION}\" experiment)"
                    )
                )
        )
        .arg_required_else_help(true)
        .get_matches();

    handle_arguments(&matches);
}

fn handle_arguments(matches: &ArgMatches) {
    let plot_width = *matches
        .get_one::<u16>(ARG_PLOT_WIDTH)
        .unwrap();
    let plot_height = *matches
        .get_one::<u16>(ARG_PLOT_HEIGHT)
        .unwrap();
    let simulation_time = *matches
        .get_one::<Millisecond>(ARG_SIM_TIME)
        .unwrap();
    
    if let Some(example_number) = matches.get_one::<u8>(ARG_EXAMPLE_NUMBER) {
        run_example_by_number(
            *example_number,
            (plot_width, plot_height),
            simulation_time,
        );
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
    let trx_system_type = match matches
        .get_one::<String>(ARG_TRX_SYSTEM)
        .unwrap()
        .as_str()
    {
        TRX_COLOR    => TRXSystemType::Color,
        TRX_STRENGTH => TRXSystemType::Strength,
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
    let delay_multiplier = matches
        .get_one::<f32>(ARG_DELAY_MULTIPLIER)
        .unwrap();

    let config = Config::new(
        plot_caption, 
        (plot_width, plot_height),
        simulation_time,
        display_delayless_network, 
        trx_system_type, 
        topology,
        *delay_multiplier,
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

fn run_example_by_number(
    example_number: u8,
    plot_resolution: (u16, u16),
    simulation_time: Millisecond,
) {
    match example_number {
        1  => examples::gps_only(
            &Config::new(
                "Strength Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Star,
                0.0,
            )
        ),
        2  => examples::gps_and_control(
            &Config::new(
                "Strength Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Star,
                0.0,
            )
        ),
        3  => examples::command_delay(
            &Config::new(
                "Strength Model (Star)",
                plot_resolution,
                simulation_time,
                true,
                TRXSystemType::Strength,
                Topology::Star,
                1_000_000.0,
            )
        ),
        4  => examples::signal_color(
            &Config::new(
                "Strength Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Star,
                0.0,
            )
        ),
        5  => examples::infection(
            &Config::new(
                "Strength Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Star,
                1_000_000.0,
            )
        ),
        6  => examples::dos(
            &Config::new(
                "Strength Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Star,
                10_000_000.0,
            )
        ),
        7  => examples::gps_only(
            &Config::new(
                "Strength Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Mesh,
                0.0,
            )
        ),
        8  => examples::gps_and_control(
            &Config::new(
                "Strength Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Mesh,
                0.0,
            )
        ),
        9  => examples::command_delay(
            &Config::new(
                "Strength Model (Mesh)",
                plot_resolution,
                simulation_time,
                true,
                TRXSystemType::Strength,
                Topology::Mesh,
                1_000_000.0,
            )
        ),
        10 => examples::signal_color(
            &Config::new(
                "Strength Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Mesh,
                0.0,
            )
        ),
        11 => examples::infection(
            &Config::new(
                "Strength Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Mesh,
                1_000_000.0,
            )
        ),
        12 => examples::dos(
            &Config::new(
                "Strength Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Mesh,
                10_000_000.0,
            )
        ),
        13 => examples::gps_only(
            &Config::new(
                "Color Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Star,
                0.0,
            )
        ),
        14 => examples::gps_and_control(
            &Config::new(
                "Color Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Star,
                0.0,
            )
        ),
        15 => examples::command_delay(
            &Config::new(
                "Color Model (Star)",
                plot_resolution,
                simulation_time,
                true,
                TRXSystemType::Color,
                Topology::Star,
                1_000_000.0,
            )
        ),
        16 => examples::signal_color(
            &Config::new(
                "Color Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Star,
                0.0,
            )
        ),
        17 => examples::infection(
            &Config::new(
                "Color Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Star,
                1_000_000.0,
            )
        ),
        18 => examples::dos(
            &Config::new(
                "Color Model (Star)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Star,
                10_000_000.0,
            )
        ),
        19 => examples::gps_only(
            &Config::new(
                "Color Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Mesh,
                0.0,
            )
        ),
        20 => examples::gps_and_control(
            &Config::new(
                "Color Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Mesh,
                0.0,
            )
        ),
        21 => examples::command_delay(
            &Config::new(
                "Color Model (Mesh)",
                plot_resolution,
                simulation_time,
                true,
                TRXSystemType::Color,
                Topology::Mesh,
                1_000_000.0,
            )
        ),
        22 => examples::signal_color(
            &Config::new(
                "Color Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Mesh,
                0.0,
            )
        ),
        23 => examples::infection(
            &Config::new(
                "Color Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Mesh,
                1_000_000.0,
            )
        ),
        24 => examples::dos(
            &Config::new(
                "Color Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Mesh,
                10_000_000.0,
            )
        ),
        25 => examples::signal_loss_response(
            &Config::new(
                "Strength Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Strength,
                Topology::Mesh,
                0.0,
            )
        ),
        26 => examples::signal_loss_response(
            &Config::new(
                "Color Model (Mesh)",
                plot_resolution,
                simulation_time,
                false,
                TRXSystemType::Color,
                Topology::Mesh,
                0.0
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
    pub plot_resolution: (u16, u16),
    pub simulation_time: Millisecond,
    pub display_delayless_network: bool, 
    pub trx_system_type: TRXSystemType,
    pub topology: Topology,
    pub delay_multiplier: f32,
}

impl Config {
    #[must_use]
    pub fn new(
        plot_caption: &str,
        plot_resolution: (u16, u16),
        simulation_time: Millisecond,
        display_delayless_network: bool,
        trx_system_type: TRXSystemType,
        topology: Topology,
        delay_multiplier: f32
    ) -> Self {
        Self {
            plot_caption: plot_caption.to_string(),
            plot_resolution,
            simulation_time,
            display_delayless_network,
            trx_system_type,
            topology,
            delay_multiplier
        }
    }
}
