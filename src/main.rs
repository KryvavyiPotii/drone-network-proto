const MAX_HEIGHT: f32    = 5.0;
const MAX_SPEED: f32     = 25.0;
const MAX_TIME: f32      = 10.0;
const STEP_DURATION: f32 = 0.5;
const RWD_RADIUS: f32    = MAX_HEIGHT;


struct Coordinates {
    x: f32,
    y: f32,
    z: f32,
}

impl Coordinates {
    fn size(&self) -> f32 {
        (self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0))
            .sqrt()
    }

    fn limit_speed(&mut self, speed: f32) {
        let size: f32 = self.size();

        if speed > 0.0 && size > speed {
            self.x = self.x * speed / size;
            self.y = self.y * speed / size;
            self.z = self.z * speed / size;
        }
    }

}


struct CommandCenter {
    position: Coordinates,
}


struct Drone {
    max_speed: f32
    position: Coordinates,
    velocity: Coordinates,
    acceleration: Coordinates,
    direction: Coordinates,
    //infection_state: i8,
    suppressed: bool,
}

impl Drone {
    fn set_destination(&mut self, destination: &Coordinates) {
        self.velocity.x = destination.x - self.position.x,
        self.velocity.y = destination.y - self.position.y,
        self.velocity.z = destination.z - self.position.z,
        
        self.velocity.limit_speed(max_speed);
        // TODO change acceleration
    }
    
    fn update_position(&mut self, time: f32) {
        self.position.x += self.velocity.x * time +
            self.acceleration.x * time.powf(2.0) / 2;
        self.position.y += self.velocity.y * time +
            self.acceleration.y * time.powf(2.0) / 2;
        self.position.z += self.velocity.z * time +
            self.acceleration.z * time.powf(2.0) / 2;
    }

}


struct RadarWarfareDevice {
    position: Coordinates,
    radar_radius: f32,
}

impl RadarWarfareDevice {
    fn in_radius(&self, drone: &Drone) -> bool {
        let drone_position = &drone.position;

        let distance: f32 = (
            (drone_position.x - self.position.x).powf(2.0) +
            (drone_position.y - self.position.y).powf(2.0) +
            (drone_position.z - self.position.z).powf(2.0)
            ).sqrt();

        if distance <= self.radar_radius { true }
        else { false }
    }
   
    fn suppress(&self, drone: &mut Drone) -> bool {
        drone.suppressed = self.in_radius(drone);
        
        drone.suppressed
    }

}


struct World {
    command_center: CommandCenter,
    drones: Drone,
    rwds: RadarWarfareDevice,
    current_time: f32,
    end_time: f32,
}


fn main() {
    let mut drone = Drone {
        max_speed: MAX_SPEED,
        position: Coordinates {
            x: 150.3,
            y: 50.6,
            z: 25.5, 
        },
        velocity: Coordinates { x: 0.0, y: 0.0, z: 0.0, },
        acceleration: Coordinates { x: 0.0, y: 0.0, z: 0.0, },
        direction: Coordinates {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        //infection_state: 0,
        suppressed: false,
    }; 
    let rwd = RadarWarfareDevice {
        position: Coordinates {
            x: 0.0,
            y: 5.0,
            z: 2.0,
        },
        radar_radius: RWD_RADIUS,
    };

    println!("Drone position:\n\tx = {}, y = {}, z = {}",
             drone.position.x, drone.position.y, drone.position.z);
    println!("Drone velocity:\n\tx = {}, y = {}, z = {}",
             drone.velocity.x, drone.velocity.y, drone.velocity.z);
    println!("RWD position:\n\tx = {}, y = {}, z = {}",
             rwd.position.x, rwd.position.y, rwd.position.z); 

    println!("Flight started!"); 
    let mut time: f32 = 0.0;
    while time < MAX_TIME {
        drone.update_position(STEP_DURATION);

        println!("Drone position:\n\tx = {}, y = {}, z = {}",
             drone.position.x, drone.position.y, drone.position.z);
        
        if rwd.suppress(&mut drone) {
            println!("Suppressed by RWD.");
            
            velocity.z = 0.0;
            velocity.limit_speed(MAX_SPEED);
            println!("Drone velocity:\n\tx = {}, y = {}, z = {}",
                     velocity.x, velocity.y, velocity.z);    
        }
        else {
            println!("Not suppressed by RWD.");
            
            velocity.x = destination_position.x - drone.position.x;
            velocity.y = destination_position.y - drone.position.y;
            velocity.z = destination_position.z - drone.position.z;
            velocity.limit_speed(MAX_SPEED);
            println!("Drone velocity:\n\tx = {}, y = {}, z = {}",
                     velocity.x, velocity.y, velocity.z);
        }

        time += STEP_DURATION;
    } 
}
