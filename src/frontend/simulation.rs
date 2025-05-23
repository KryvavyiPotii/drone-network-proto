use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

use log::{info, trace};

use crate::backend::device::STEP_DURATION;
use crate::backend::device::networkmodel::NetworkModel;
use crate::backend::mathphysics::Millisecond;

use renderer::PlottersRenderer;


pub mod renderer;


fn set_ctrlc_handler() -> Arc<AtomicBool> {
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc_async::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    running
}

fn log_device_count(network_models: &[NetworkModel]) -> String {
    let mut output = String::new();

    for (i, network_model) in network_models.iter().enumerate() {
        let entry = format!(
            "Device count {}: {}, ", 
            i, 
            network_model.device_count()
        );
        
        output.push_str(&entry); 
    }

    // Remove trailing comma and space.
    output.truncate(output.len() - 2);

    output
}


pub struct Simulation<'a> {
    current_time: Millisecond,
    end_time: Millisecond,
    network_models: Vec<NetworkModel>,
    renderer: PlottersRenderer<'a>
}

impl<'a> Simulation<'a> {
    #[must_use]
    pub fn new(
        end_time: Millisecond,
        network_models: Vec<NetworkModel>,
        renderer: PlottersRenderer<'a>
    ) -> Self {
        Self {
            current_time: 0,
            end_time,
            network_models,
            renderer
        }
    }

    /// # Panics
    ///
    /// Will panic if an error occurs during rendering. 
    pub fn run(&mut self) {
        let running = set_ctrlc_handler(); 

        let end = self.end_time;

        info!("Output filename: {}", self.renderer.output_filename());
        for _ in (0..end).step_by(STEP_DURATION as usize) {
            if !running.load(std::sync::atomic::Ordering::SeqCst) { 
                info!(
                    "TERM, Simulation, Time: {}, {}", 
                    self.current_time,
                    log_device_count(&self.network_models)
                );
                return;
            }

            self.network_models
                .iter_mut()
                .for_each(NetworkModel::update);
            
            self.renderer
                .render(&self.network_models)
                .unwrap();
                        
            trace!(
                "Current time: {}, {}", 
                self.current_time,
                log_device_count(&self.network_models)
            );
            self.current_time += STEP_DURATION;
        }

        info!("Simulation finished at {} millis", self.current_time);
    }
}
