use derive_more::{Add, Sub, Mul, Div, MulAssign, DivAssign};

use super::point::Point3D;


#[derive(
    Copy, Clone, 
    PartialEq, Add, Sub, Mul, Div, MulAssign, DivAssign, 
    Debug, Default
)]
pub struct Vector3D {
    pub initial_point: Point3D,
    pub terminal_point: Point3D
}

impl Vector3D {
    #[must_use]
    pub fn new(initial_point: Point3D, terminal_point: Point3D) -> Self {
        Self { initial_point, terminal_point }
    }

    #[must_use]
    pub fn displacement(&self) -> Point3D {
        self.terminal_point - self.initial_point
    }

    #[must_use]
    pub fn size(&self) -> f32 {
        let displacement = self.displacement();
        
        (
            displacement.x.powi(2)
            + displacement.y.powi(2)
            + displacement.z.powi(2)
        ).sqrt()
    }

    pub fn normalize(&mut self) {
        let vector_size = self.size();

        if vector_size != 0.0 {
            *self /= vector_size;
        }
    }

    pub fn truncate(&mut self, truncated_size: f32) { 
        if truncated_size < 0.0 || truncated_size >= self.size() {
            return; 
        } else if truncated_size == 0.0 {
            *self = Self::default();
        } 

        self.normalize();

        *self *= truncated_size;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_vector_as_default() {
        let default_vector = Vector3D::default();

        assert_eq!(default_vector.initial_point.x, 0.0);
        assert_eq!(default_vector.initial_point.y, 0.0);
        assert_eq!(default_vector.initial_point.z, 0.0);
        assert_eq!(default_vector.terminal_point.x, 0.0);
        assert_eq!(default_vector.terminal_point.y, 0.0);
        assert_eq!(default_vector.terminal_point.z, 0.0);
    }

    #[test]
    fn normalizing_zero_vector() {
        let mut zero_vector = Vector3D::default();

        zero_vector.normalize();

        assert!(!zero_vector.initial_point.x.is_nan());
        assert!(!zero_vector.initial_point.y.is_nan());
        assert!(!zero_vector.initial_point.z.is_nan());
        assert!(!zero_vector.terminal_point.x.is_nan());
        assert!(!zero_vector.terminal_point.y.is_nan());
        assert!(!zero_vector.terminal_point.z.is_nan());
    }

    #[test]
    fn correct_vector_truncation() {
        let vector_size = 5.0;
        let original_vector = Vector3D::new(
            Point3D::default(),
            Point3D::new(vector_size, 0.0, 0.0)
        );
        
        let negative_truncated_size = -1.0;
        let mut not_truncated_vector = original_vector;
        
        not_truncated_vector.truncate(negative_truncated_size);
        assert_eq!(not_truncated_vector, original_vector);

        let too_big_truncated_size = vector_size * 2.0;
        let mut not_truncated_vector = original_vector;

        not_truncated_vector.truncate(too_big_truncated_size);
        assert_eq!(not_truncated_vector, original_vector);

        let zero_truncated_size = 0.0;
        let mut not_truncated_vector = original_vector;
        
        not_truncated_vector.truncate(zero_truncated_size);
        assert_eq!(not_truncated_vector, Vector3D::default());

        let normal_truncated_size = 2.0;
        let mut truncated_vector = original_vector;

        truncated_vector.truncate(normal_truncated_size);
        assert_eq!(
            truncated_vector, 
            Vector3D::new(
                Point3D::default(),
                Point3D::new(2.0, 0.0, 0.0)
            )
        );
    }
}
