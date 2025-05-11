use std::io::Write;

use env_logger::{Builder, Target};

use crate::cli::cli;

pub mod cli;
pub mod device;
pub mod examples;
pub mod infection;
pub mod mathphysics;
pub mod message;
pub mod signal;
pub mod simulation;


fn configure_logging() {
    Builder::new()
        .format(|buf, record| 
            writeln!(
                buf,
                "{} {} - {}", 
                chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
                record.level(), 
                record.args()
            )
        )
        .filter(None, log::LevelFilter::Trace)
        .target(Target::Stdout)
        .init();
}

fn main() {
    configure_logging();
    cli();
}
