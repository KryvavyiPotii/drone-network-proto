use std::io::Write;

use env_logger::{Builder, Target};

use crate::frontend::cli::cli;


pub mod backend;
pub mod frontend;


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
