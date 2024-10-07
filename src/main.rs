use std::collections::HashMap;

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


struct GPS {
    drones_positions: HashMap<u32, Coordinates>,
}

impl GPS {
    fn connect_drone(&mut self, drone: &Drone) {
        self.drones_positions.insert(drone.id, drone.position.clone());
    }
    
    fn update_position(&mut self, drone: &Drone, time: f32) {
        match self.drones_positions.get(&drone.id) {
            Some(drone_position) => {
                let position = drone_position.clone();

                position.x = drone.velocity.x * time +
                    drone.acceleration.x * time.powf(2.0) / 2.0;
                position.y += drone.velocity.y * time +
                    drone.acceleration.y * time.powf(2.0) / 2.0;
                position.z += drone.velocity.z * time +
                    drone.acceleration.z * time.powf(2.0) / 2.0;
            
                self.drones_positions.insert(drone.id, position);
                return
            },
            None => return,
        }

    }

}


struct Drone {
    id: u32,
    max_speed: f32,
    position: Coordinates,
    velocity: Coordinates,
    acceleration: Coordinates,
    destination: Coordinates,
    //infection_state: i8,
    command_center: CommandCenter,
    command_center_connection: bool,
    gps: GPS,
    gps_connection: bool,
}

impl Drone {
    fn set_destination(&mut self, destination: &Coordinates) {
        self.velocity.x = destination.x - self.position.x;
        self.velocity.y = destination.y - self.position.y;
        self.velocity.z = destination.z - self.position.z;
        
        self.velocity.limit_speed(self.max_speed);
        // TODO change acceleration
    }
    
    fn update_position(&mut self, time: f32) {
        if self.gps_connection {
            self.velocity.z = 0.0;
            self.acceleration.z = 0.0;
        }
        else {
            self.set_destination(&self.destination);
        }

        self.position.x += self.velocity.x * time +
            self.acceleration.x * time.powf(2.0) / 2.0;
        self.position.y += self.velocity.y * time +
            self.acceleration.y * time.powf(2.0) / 2.0;
        self.position.z += self.velocity.z * time +
            self.acceleration.z * time.powf(2.0) / 2.0;
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
        
        if !drone.suppressed {
            drone.velocity.x = drone.destination.x - drone.position.x;
            drone.velocity.y = drone.destination.y - drone.position.y;
            drone.velocity.z = drone.destination.z - drone.position.z;
            drone.velocity.limit_speed(MAX_SPEED); 
        }
        
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
            
            drone.velocity.z = 0.0;
            drone.velocity.limit_speed(MAX_SPEED);
            println!("Drone velocity:\n\tx = {}, y = {}, z = {}",
                     velocity.x, velocity.y, velocity.z);    
        }
        else {
            println!("Not suppressed by RWD.");
            
            println!("Drone velocity:\n\tx = {}, y = {}, z = {}",
                     drone.velocity.x, drone.velocity.y, drone.velocity.z);
        }

        time += STEP_DURATION;
    }
}
