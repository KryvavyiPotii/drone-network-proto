use std::f32::NAN;

pub use point::Point3D;
pub use vector::Vector3D;
pub use unit::*;


pub mod point;
pub mod vector;
pub mod unit;


pub const INVALID_POSITION: Point3D = Point3D { x: NAN, y: NAN, z: NAN};


pub fn equation_of_motion_1d(
    start_position: f32,
    velocity: f32,
    time_in_secs: f32
) -> f32 {
    velocity.mul_add(time_in_secs, start_position)
}

pub fn equation_of_motion_3d(
    start_position: &Point3D,
    velocity: &Point3D,
    time_in_secs: f32
) -> Point3D {
    Point3D::new(
        equation_of_motion_1d(start_position.x, velocity.x, time_in_secs),
        equation_of_motion_1d(start_position.y, velocity.y, time_in_secs),
        equation_of_motion_1d(start_position.z, velocity.z, time_in_secs),
    )
}


pub trait Position {
    fn position(&self) -> &Point3D;

    fn distance_to<P: Position>(&self, other: &P) -> f32 {
        let vector = Vector3D::new(*self.position(), *other.position());
        
        vector.size()
    }

    fn cmp_by_distance_to<P: Position>(
        &self, 
        other: &P, 
        destination: &P
    ) -> std::cmp::Ordering {
        let distance_x = self.distance_to(destination);
        let distance_y = other.distance_to(destination);
        
        distance_x
            .partial_cmp(&distance_y)
            .expect("Failed to compare f32 values")
    }
}
