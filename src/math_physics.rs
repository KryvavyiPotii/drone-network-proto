pub fn equation_of_motion_1d(
    start_position: f32,
    velocity: f32,
    time_in_secs: f32
) -> f32 {
    velocity.mul_add(time_in_secs, start_position)
}

pub fn equation_of_motion_3d(
    start_position: &Coordinates3D,
    velocity: &Coordinates3D,
    time_in_secs: f32
    ) -> Coordinates3D {
    Coordinates3D::new(
        equation_of_motion_1d(start_position.x, velocity.x, time_in_secs),
        equation_of_motion_1d(start_position.y, velocity.y, time_in_secs),
        equation_of_motion_1d(start_position.z, velocity.z, time_in_secs),
    )
}


pub trait Position {
    fn position(&self) -> Coordinates3D;

    fn distance_to<U>(&self, other: &U) -> f32
    where
        U: Position,
    {
        self.position().point_vector_to(&other.position()).size()
    }

    fn cmp_by_distance_to<U, T>(&self, other: &U, destination: &T)
        -> std::cmp::Ordering
    where
        U: Position,
        T: Position,
    {
        let distance_x = self.distance_to(destination);
        let distance_y = other.distance_to(destination);
        
        distance_x.partial_cmp(&distance_y)
            .expect("Failed to compare f32 values")
    }
}


#[derive(Copy, Clone, Debug)]
pub struct Coordinates3D { 
    pub x: f32, 
    pub y: f32, 
    pub z: f32, 
}

impl Coordinates3D {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn coordinates(&self) -> (f32, f32, f32) {
        (self.x, self.y, self.z)
    }

    pub fn set_coordinates(&mut self, coordinates: (f32, f32, f32)) {
        self.x = coordinates.0;
        self.y = coordinates.1;
        self.z = coordinates.2;
    }

    pub fn size(&self) -> f32 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    pub fn truncate_vector_size(&mut self, truncation_size: f32) { 
        let vector_size: f32 = self.size();
        
        if truncation_size > 0.0 && vector_size > truncation_size {
            self.x = self.x * truncation_size / vector_size;
            self.y = self.y * truncation_size / vector_size;
            self.z = self.z * truncation_size / vector_size;
        }
    }

    pub fn point_vector_to(&self, other: &Coordinates3D) -> Self {
        Self::new(
            other.x - self.x,
            other.y - self.y,
            other.z - self.z
        )
    }
}

impl From<(f32, f32, f32)> for Coordinates3D {
    fn from(value: (f32, f32, f32)) -> Self {
        Self::new(value.0, value.1, value.2)
    }
}

impl Position for Coordinates3D {
    fn position(&self) -> Self {
        *self
    }
}
