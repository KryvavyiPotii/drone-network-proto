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
        if truncated_size < self.size() {
            self.scale_to(truncated_size);
        }
    }

    pub fn scale_to(&mut self, scaled_size: f32) {
        if scaled_size < 0.0 {
            return; 
        } else if scaled_size == 0.0 {
            *self = Self::default();
        }
        
        self.normalize();

        *self *= scaled_size;
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
        
        let negative_size = -1.0;
        let mut not_truncated_vector = original_vector;
        
        not_truncated_vector.truncate(negative_size);
        assert_eq!(not_truncated_vector, original_vector);

        let bigger_size = vector_size * 2.0;
        let mut not_truncated_vector = original_vector;

        not_truncated_vector.truncate(bigger_size);
        assert_eq!(not_truncated_vector, original_vector);

        let zero_size = 0.0;
        let mut not_truncated_vector = original_vector;
        
        not_truncated_vector.truncate(zero_size);
        assert_eq!(not_truncated_vector, Vector3D::default());

        let smaller_size = vector_size / 2.0;
        let mut truncated_vector = original_vector;

        truncated_vector.truncate(smaller_size);
        assert_eq!(
            truncated_vector, 
            Vector3D::new(
                Point3D::default(),
                Point3D::new(smaller_size, 0.0, 0.0)
            )
        );
    }

    #[test]
    fn correct_vector_scaling() {
        let vector_size = 5.0;
        let original_vector = Vector3D::new(
            Point3D::default(),
            Point3D::new(vector_size, 0.0, 0.0)
        );
        
        let negative_size = -1.0;
        let mut not_scaled_vector = original_vector;
        
        not_scaled_vector.scale_to(negative_size);
        assert_eq!(not_scaled_vector, original_vector);

        let bigger_size = vector_size * 2.0;
        let mut bigger_vector = original_vector;

        bigger_vector.scale_to(bigger_size);
        assert_eq!(
            bigger_vector, 
            Vector3D::new(
                Point3D::default(),
                Point3D::new(bigger_size, 0.0, 0.0)
            )
        );

        let zero_size = 0.0;
        let mut defaulted_vector = original_vector;
        
        defaulted_vector.scale_to(zero_size);
        assert_eq!(defaulted_vector, Vector3D::default());

        let smaller_size = vector_size / 2.0;
        let mut smaller_vector = original_vector;

        smaller_vector.scale_to(smaller_size);
        assert_eq!(
            smaller_vector, 
            Vector3D::new(
                Point3D::default(),
                Point3D::new(smaller_size, 0.0, 0.0)
            )
        );
    }
}
