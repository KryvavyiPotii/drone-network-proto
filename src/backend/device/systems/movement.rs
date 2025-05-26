use thiserror::Error;

use crate::backend::mathphysics::{MeterPerSecond, Point3D, Vector3D};


#[derive(Error, Debug)]
pub enum MovementSystemBuildError {
    #[error("Maximum speed is negative")]
    NegativeMaxSpeed
}


// By default the system can not move, because its maximum speed is 0.0.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MovementSystem {
    position_in_meters: Point3D,
    max_speed: MeterPerSecond,
    velocity_in_mps: Vector3D,
}

impl MovementSystem {
    /// # Errors
    ///
    /// Will return `Err` if `max_speed` is negative. 
    pub fn build(
        max_speed: MeterPerSecond
    ) -> Result<Self, MovementSystemBuildError> {
        if max_speed < 0.0 {
            return Err(MovementSystemBuildError::NegativeMaxSpeed);
        }

        Ok(
            Self {
                // Upon creation the system does not know its position.
                // The position should be provided by GPS (from TRXSystem).
                position_in_meters: Point3D::default(),
                max_speed,
                velocity_in_mps: Vector3D::default()
            }
        )
    }
    
    #[must_use]
    pub fn position(&self) -> &Point3D {
        &self.position_in_meters
    }
    
    #[must_use]
    pub fn max_speed(&self) -> MeterPerSecond {
        self.max_speed
    }

    #[must_use]
    pub fn velocity(&self) -> &Vector3D {
        &self.velocity_in_mps
    }

    #[must_use]
    pub fn is_disabled(&self) -> bool {
        self.max_speed == 0.0
    }
    
    pub fn set_position(&mut self, position_in_meters: Point3D) {
        self.position_in_meters = position_in_meters;
    }
    
    pub fn set_velocity(&mut self, velocity_in_mps: Vector3D) {
        if self.is_disabled() {
            return;
        }

        self.velocity_in_mps = velocity_in_mps;
        self.velocity_in_mps.truncate(self.max_speed);
    }
    
    pub fn set_direction(&mut self, destination_in_metres: Point3D) {
        if self.is_disabled() {
            return;
        }
        
        self.velocity_in_mps = Vector3D::new(
            self.position_in_meters,
            destination_in_metres
        );
        
        self.velocity_in_mps.scale_to(self.max_speed);
    }
}


#[cfg(test)]
mod tests {
    use super::*;


    #[test]
    fn default_movement_system_does_not_function() {
        let default_movement_system = MovementSystem::default();

        assert_eq!(default_movement_system.max_speed(), 0.0);
    }

    #[test]
    fn building_movement_system_with_negative_max_speed() {
        let result = MovementSystem::build(-5.0);

        assert!(
            matches!(result, Err(MovementSystemBuildError::NegativeMaxSpeed))
        );
    }

    #[test]
    fn setting_velocity() {
        let max_speed = 5.0;
        let mut movement_system = MovementSystem::build(max_speed)
            .unwrap();

        assert_eq!(*movement_system.velocity(), Vector3D::default());

        let normal_velocity = Vector3D::new(
            Point3D::default(), 
            Point3D::new(max_speed / 2.0, 0.0, 0.0)
        );
        movement_system.set_velocity(normal_velocity);
        
        assert_eq!(*movement_system.velocity(), normal_velocity);

        let too_high_velocity = Vector3D::new(
            Point3D::default(), 
            Point3D::new(max_speed * 2.0, 0.0, 0.0)
        );
        movement_system.set_velocity(too_high_velocity);

        assert_eq!(
            *movement_system.velocity(), 
            Vector3D::new(
                Point3D::default(),
                Point3D::new(max_speed, 0.0, 0.0)
            )
        );
    }
}
