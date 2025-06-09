use log::{info, trace};

use crate::backend::ITERATION_TIME;
use crate::backend::networkmodel::NetworkModel;
use crate::backend::mathphysics::Millisecond;

use super::renderer::PlottersRenderer;


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


pub struct ModelPlayer<'a> {
    current_time: Millisecond,
    end_time: Millisecond,
    network_models: Vec<NetworkModel>,
    renderer: PlottersRenderer<'a>
}

impl<'a> ModelPlayer<'a> {
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
    pub fn play(&mut self) {
        info!("Output filename: {}", self.renderer.output_filename());

        for _ in (0..self.end_time).step_by(ITERATION_TIME as usize) {
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

            self.current_time += ITERATION_TIME;
        }

        info!("Simulation finished at {} millis", self.current_time);
    }
}
