use full_palette::{GREEN_400, GREY, ORANGE, PINK, RED_400, YELLOW_700};
use plotters::prelude::*;
use plotters::style::RGBColor;

use crate::backend::{CONTROL_FREQUENCY, DESTINATION_RADIUS};
use crate::backend::device::Device;
use crate::backend::mathphysics::{Megahertz, Meter, Point3D, Position};
use crate::backend::networkmodel::attack::{AttackerDevice, AttackType};
use crate::backend::signal::{SignalLevel, GPS_L1_FREQUENCY};

use super::{
    DeviceColoring, PlotResolution, meters_to_pixels, 
    plotters_point_from_point3d
};


const COMMAND_CENTER_RADIUS: Meter = 5.0;

const CIRCLE_SIZE_COEF: u32 = 400;

const PLOTTERS_DESTINATION_COLOR: RGBColor    = YELLOW;
const PLOTTERS_COMMAND_CENTER_COLOR: RGBColor = GREEN;


type PlottersCircle = Circle<(f64, f64, f64), u32>; 


pub fn destination_primitive( 
    destination: &Point3D,
    plot_resolution: PlotResolution
) -> PlottersCircle {
    let point  = plotters_point_from_point3d(destination);
    let radius = meters_to_pixels(
        DESTINATION_RADIUS,
        plot_resolution
    );

    Circle::new(point, radius, PLOTTERS_DESTINATION_COLOR)
}

pub fn command_device_primitive(
    command_device: &Device,
    plot_resolution: PlotResolution
) -> PlottersCircle {
    let point  = plotters_point_from_point3d(command_device.position());
    let radius = meters_to_pixels(
        COMMAND_CENTER_RADIUS,
        plot_resolution
    );  
    
    Circle::new(point, radius, PLOTTERS_COMMAND_CENTER_COLOR)
}

pub fn device_primitive(
    drone: &Device,
    coloring: DeviceColoring,
    plot_resolution: PlotResolution
) -> PlottersCircle {
    let point = plotters_point_from_point3d(drone.position());
    let color = get_drone_color(drone, coloring);
    let style = Into::<ShapeStyle>::into(color).filled();
    let size  = if plot_resolution.height() < CIRCLE_SIZE_COEF {
        1  
    } else {
        plot_resolution.height() / CIRCLE_SIZE_COEF
    };

    Circle::new(point, size, style)
}

pub fn attacker_device_primitive(
    attacker_device: &AttackerDevice,
    frequency: Megahertz,
    plot_resolution: PlotResolution
) -> PlottersCircle {
    let radius = attacker_device
        .device()
        .area(frequency)
        .radius();
    let device_position = attacker_device.device().position();
    let spoofs_gps = matches!(
        attacker_device.attack_type(), 
        AttackType::GPSSpoofing(_)
    );

    let point = plotters_point_from_point3d(device_position);
    let attacker_device_coverage = meters_to_pixels(radius, plot_resolution);
    let area_color = match frequency {
        GPS_L1_FREQUENCY if spoofs_gps => ORANGE,
        GPS_L1_FREQUENCY               => RED,
        CONTROL_FREQUENCY              => BLUE,
        _                              => GREY
    };

    Circle::new(point, attacker_device_coverage, area_color)
}

fn get_drone_color(drone: &Device, coloring: DeviceColoring) -> RGBColor {
    match coloring {
        DeviceColoring::Infection            => {
            color_by_infection(drone.is_infected())
        },
        DeviceColoring::Signal               => {
            let signal_level = drone.rx_signal_level(CONTROL_FREQUENCY);
            
            color_by_signal(*signal_level)
        },
        DeviceColoring::SingleColor(r, g, b) => {
            RGBColor(r, g, b)
        }
    }
}

fn color_by_infection(infected: bool) -> RGBColor {
    if infected {
        PINK
    } else {
        BLACK
    }
}

fn color_by_signal(signal_level: SignalLevel) -> RGBColor {
    if signal_level.is_green() {
        GREEN_400
    } else if signal_level.is_yellow() {
        YELLOW_700
    } else if signal_level.is_red() {
        RED_400
    } else {
        BLACK
    }
}
