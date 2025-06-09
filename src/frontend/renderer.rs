use full_palette::GREY;
use plotters::coord::Shift;
use plotters::coord::ranged3d::Cartesian3d;
use plotters::coord::types::RangedCoordf64;
use plotters::prelude::*;
use thiserror::Error;

use crate::backend::ITERATION_TIME;
use crate::backend::mathphysics::Point3D;
use crate::backend::message::Task;
use crate::backend::networkmodel::NetworkModel;
use crate::backend::networkmodel::attack::AttackerDevice;

use primitives::{
    attacker_device_primitive_on_all_frequencies, command_device_primitive, 
    destination_primitive, device_primitive
};

pub use plotcfg::{
    Axes3DRanges, CameraAngle, DeviceColoring, Pixel, PlottersUnit, 
    PlottersPoint3D, PlotResolution, meters_to_pixels 
};

use plotcfg::{font_size, PLOT_MARGIN};


mod plotcfg;
mod primitives;


type PlottersChartContext<'a> = ChartContext<
    'a, 
    BitMapBackend<'a>, 
    Cartesian3d<RangedCoordf64, RangedCoordf64, RangedCoordf64>
>;


const FONT: &str = "sans-serif";


fn network_models_destinations(
    network_models: &[NetworkModel]
) -> Vec<Point3D> {
    let mut destinations = Vec::new();

    for network_model in network_models {
        let task_vec: Vec<Task> = network_model
            .device_tasks()
            .values()
            .copied()
            .collect();

        for task in task_vec {
            match task {
                Task::Attack(destination) 
                    | Task::Reconnect(destination)
                    | Task::Reposition(destination) => 
                    destinations.push(destination),
                Task::Undefined => (),
            }
        }
    }

    destinations
}


#[derive(Debug, Error)]
pub enum RenderError {
    #[error("Color count does not match network count")]
    NotMatchingColorNumber
}


pub struct PlottersRenderer<'a> {
    output_filename: String,
    caption: String,
    plot_resolution: PlotResolution,
    font_size: Pixel,
    axes_ranges: Axes3DRanges,
    camera_angle: CameraAngle,
    device_colorings: Vec<DeviceColoring>,
    area: DrawingArea<BitMapBackend<'a>, Shift>, 
}

impl<'a> PlottersRenderer<'a> {
    /// # Panics
    ///
    /// Will panic if an error occurs during bitmap backend creation. 
    #[must_use]
    pub fn new(
        output_filename: &str,
        caption: &str,
        plot_resolution: PlotResolution,
        axes_ranges: Axes3DRanges,
        device_colorings: &[DeviceColoring],
        camera_angle: CameraAngle,
    ) -> Self {
        let font_size = font_size(plot_resolution);
        let area      = BitMapBackend::gif(
            output_filename, 
            plot_resolution.into(),
            ITERATION_TIME
                .try_into()
                .expect("Failed to convert i32 to u32")
        )
            .expect("Failed to create `BitMapBackend`")
            .into_drawing_area();

        Self {
            output_filename: output_filename.to_string(),
            caption: caption.to_string(),
            plot_resolution,
            font_size,
            axes_ranges,
            camera_angle,
            device_colorings: device_colorings.to_vec(),
            area,
        }
    }

    #[must_use]
    pub fn output_filename(&self) -> String {
        self.output_filename.clone()
    }

    /// # Errors
    ///
    /// Will return `Err` if the number of networks does not match the number of
    /// colors.
    /// 
    /// # Panics
    ///
    /// Will panic if an error occurs during drawing.
    pub fn render(
        &mut self, 
        network_models: &[NetworkModel]
    ) -> Result<(), RenderError> {
        if self.device_colorings.len() != network_models.len() {
            return Err(RenderError::NotMatchingColorNumber);
        }
        
        self.area
            .fill(&WHITE)
            .expect("Failed to fill an area");
        
        let mut chart_context = self.chart_context();

        self.draw_chart(&mut chart_context);
        self.draw_network_models(network_models, &mut chart_context);

        self.area
            .present()
            .expect("Failed to finalize drawing");

        Ok(())
    }
    
    fn chart_context(&self) -> PlottersChartContext<'a> {
        let mut chart_builder = ChartBuilder::on(&self.area);

        if !self.caption.is_empty() {
            chart_builder.caption(
                &self.caption, 
                (FONT, self.font_size)
            );
        }

        chart_builder
            .margin(PLOT_MARGIN)
            .build_cartesian_3d(
                self.axes_ranges.x(),
                self.axes_ranges.y(),
                self.axes_ranges.z(),
            )
            .expect("Failed to create a chart")
    }

    fn draw_network_models(
        &self,
        network_models: &[NetworkModel],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        let destinations = network_models_destinations(network_models);

        self.draw_destinations(&destinations, chart_context);
        self.draw_command_devices(network_models, chart_context);
        self.draw_devices(network_models, chart_context);
        
        for network_model in network_models {
            self.draw_attacker_devices(
                network_model.attacker_devices(), 
                chart_context
            );
        }
    }

    fn draw_chart(&self, chart_context: &mut PlottersChartContext<'a>) {
        chart_context 
            .with_projection(|mut p| {
                p.pitch = self.camera_angle.pitch();
                p.yaw = self.camera_angle.yaw();
                p.into_matrix()
            })
            .configure_axes()
            .axis_panel_style(GREY.mix(0.1))
            .label_style((FONT, self.font_size / 2))
            .draw()
            .expect("Failed to draw a chart");
    }
    
    fn draw_destinations(
        &self, 
        destinations: &[Point3D],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        let destination_primitives = destinations
            .iter()
            .map(|destination| 
                destination_primitive(
                    destination, 
                    self.plot_resolution
                )
            );

        chart_context
            .draw_series(destination_primitives)
            .expect("Failed to draw destination points");
    }
    
    fn draw_command_devices(
        &self, 
        network_models: &[NetworkModel],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        let network_model_primitives = network_models
            .iter()
            .filter_map(|network_model| {
                let command_device = network_model.command_device()?;
                let primitive = command_device_primitive(
                    command_device, 
                    self.plot_resolution
                );

                Some(primitive)
            });

        chart_context
            .draw_series(network_model_primitives)
            .expect("Failed to draw destination points");
    }

    fn draw_devices(
        &self, 
        network_models: &[NetworkModel],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        for (network_model, coloring) in network_models
            .iter()
            .zip(self.device_colorings.iter())
        {
            let device_primitives = network_model
                .device_iter()
                .map(|device|
                    device_primitive(
                        device, 
                        *coloring, 
                        self.plot_resolution
                    )
                );

            chart_context.draw_series(device_primitives).unwrap();
        }
    }

    fn draw_attacker_devices(
        &self, 
        attacker_devices: &[AttackerDevice],
        chart_context: &mut PlottersChartContext<'a>
    ) {
        let attacker_device_primitives = attacker_devices
            .iter()
            .flat_map(|attacker_device| {
                attacker_device_primitive_on_all_frequencies(
                    attacker_device, 
                    self.plot_resolution
                )
            });

        chart_context
            .draw_series(attacker_device_primitives)
            .expect("Failed to draw control EWDs");
    }
}
