use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

use log::{info, trace};

use crate::device::STEP_DURATION;
use crate::device::networkmodel::NetworkModel;
use crate::mathphysics::Millisecond;

use self::renderer::PlottersRenderer;


pub mod renderer;


pub const START_TIME: Millisecond = 0;
pub const END_TIME: Millisecond   = 15000;


fn set_ctrlc_handler() -> Arc<AtomicBool> {
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc_async::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    running
}

fn log_drone_count(drone_networks: &[NetworkModel]) -> String {
    let mut output = String::new();

    for (i, network) in drone_networks.iter().enumerate() {
        let entry = format!("Drone count {}: {}, ", i, network.drone_count());
        
        output.push_str(&entry); 
    }

    // Remove trailing comma and space.
    output.truncate(output.len() - 2);

    output
}


pub struct Simulation<'a> {
    current_time_in_millis: u32,
    end_time_in_millis: u32,
    drone_networks: Vec<NetworkModel>,
    renderer: PlottersRenderer<'a>
}

impl<'a> Simulation<'a> {
    #[must_use]
    pub fn new(
        end_time_in_millis: u32,
        drone_networks: Vec<NetworkModel>,
        renderer: PlottersRenderer<'a>
    ) -> Self {
        Self {
            current_time_in_millis: START_TIME,
            end_time_in_millis,
            drone_networks,
            renderer
        }
    }

    /// # Panics
    ///
    /// Will panic if an error occurs during rendering. 
    pub fn run(&mut self) {
        let running = set_ctrlc_handler(); 

        let begin = self.current_time_in_millis;
        let end = self.end_time_in_millis;

        for _ in (begin..end).step_by(STEP_DURATION as usize) {
            if !running.load(std::sync::atomic::Ordering::SeqCst) { 
                info!(
                    "TERM, Simulation, Time: {}, {}", 
                    self.current_time_in_millis,
                    log_drone_count(&self.drone_networks)
                );
                return;
            }

            self.drone_networks
                .iter_mut()
                .for_each(NetworkModel::update);
            
            self.renderer
                .draw_drone_networks(&self.drone_networks)
                .unwrap();
                        
            trace!(
                "Current time: {}, {}", 
                self.current_time_in_millis,
                log_drone_count(&self.drone_networks)
            );
            self.current_time_in_millis += STEP_DURATION;
        }

        info!(
            "Simulation finished at {} millis",
            self.current_time_in_millis
        );
    }
}
