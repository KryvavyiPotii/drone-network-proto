use super::ITERATION_TIME;

pub use point::Point3D;
pub use unit::*;
pub use vector::Vector3D;


pub mod point;
pub mod unit;
pub mod vector;


pub const INVALID_POSITION: Point3D = Point3D { 
    x: f32::NAN, y: f32::NAN, z: f32::NAN
};


#[must_use]
pub fn delay_to(distance: Meter, multiplier: f32) -> Millisecond {    
    if multiplier == 0.0 {
        return 0;
    }

    let delay = time_in_millis_from_distance_and_speed(
        distance * multiplier as Meter,
        kmps_to_mpms(SPEED_OF_LIGHT) 
    );
    let reminder = delay % ITERATION_TIME;
    
    delay - reminder
}

#[must_use]
pub fn equation_of_motion_1d(
    start_position: Meter,
    velocity: MeterPerSecond,
    time: Second
) -> Meter {
    velocity.mul_add(time, start_position)
}

#[must_use]
pub fn equation_of_motion_3d(
    start_position: &Point3D,
    velocity: &Point3D,
    time: Second
) -> Point3D {
    Point3D::new(
        equation_of_motion_1d(start_position.x, velocity.x, time),
        equation_of_motion_1d(start_position.y, velocity.y, time),
        equation_of_motion_1d(start_position.z, velocity.z, time),
    )
}


pub trait Position {
    fn position(&self) -> &Point3D;

    fn distance_to<P: Position>(&self, other: &P) -> f32 {
        let vector = Vector3D::new(*self.position(), *other.position());
        
        vector.size()
    }

    /// # Panics
    ///
    /// Will panic if distances are not comparable.
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

#[cfg(test)]
mod tests {
    use super::*;

    const ORIGIN: Point3D = Point3D { x: 0.0, y: 0.0, z: 0.0 };

    #[test]
    fn distance_to_another_point() {
        let some_point = Point3D::new(5.0, 0.0, 0.0);

        assert_eq!(0.0, ORIGIN.distance_to(&ORIGIN));
        assert_eq!(5.0, ORIGIN.distance_to(&some_point));
    }

    #[test]
    fn comparison_by_distance() {
        let point_a = Point3D::new(5.0, 0.0, 0.0);
        let point_b = Point3D::new(0.0, -5.0, 0.0);

        assert_eq!(
            7.0,
            point_a.distance_to(&point_b).round()
        );
        assert_eq!(
            ORIGIN.distance_to(&point_a),
            point_a.distance_to(&ORIGIN)
        );
        assert_eq!(
            ORIGIN.distance_to(&point_a),
            point_b.distance_to(&ORIGIN)
        );
    }
}
