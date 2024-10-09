const MAX_HEIGHT: f32    = 5.0;
const MAX_SPEED: f32     = 25.0;
const MAX_TIME: f32      = 10.0;
const STEP_DURATION: f32 = 0.5;
const RWD_RADIUS: f32    = MAX_HEIGHT;

#[derive(Clone)]
struct Coordinates3D { 
    x: f32, 
    y: f32, 
    z: f32, 
}

impl Coordinates3D {
    fn set_coordinates(&mut self, x: f32, y: f32, z: f32) {
        self.x = x;
        self.y = y;
        self.z = z;
    }

    fn size(&self) -> f32 {
        (self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0))
            .sqrt()
    }

    fn truncate_vector_size(&mut self, truncation_size: f32) { 
        let vector_size: f32 = self.size();
        
        if truncation_size > 0.0 && vector_size > truncation_size {
            self.x = self.x * truncation_size / vector_size;
            self.y = self.y * truncation_size / vector_size;
            self.z = self.z * truncation_size / vector_size;
        }
    }
}


enum SignalLevel {
    Green,
    Yellow,
    Red,
    Black,
}


#[derive(Clone)]
struct CommandCenter {
    position: Coordinates3D,
}


struct Drone {
    max_speed: f32,
    global_position: Coordinates3D,
    position: Coordinates3D,
    velocity: Coordinates3D,
    acceleration: Coordinates3D,
    destination: Coordinates3D,
    // TODO infection_state
    command_center: CommandCenter,
    // TODO network or command center connection state
    gps_connection: bool,
}

impl Drone {
    fn set_destination(&mut self, destination: Option<&Coordinates3D>) {
        match destination {
            Some(coordinates) => self.destination.set_coordinates(
                coordinates.x,
                coordinates.y,
                coordinates.z
            ),
            None => (),
        }

        self.velocity.set_coordinates(
            self.destination.x - self.position.x,
            self.destination.y - self.position.y,
            self.destination.z - self.position.z
        );
        
        self.velocity.truncate_vector_size(self.max_speed);
        // TODO change acceleration
    }
    
    fn update_position(&mut self, time: f32) {
        self.global_position.x += self.velocity.x * time +
            self.acceleration.x * time.powf(2.0) / 2.0;
        self.global_position.y += self.velocity.y * time +
            self.acceleration.y * time.powf(2.0) / 2.0;
        self.global_position.z += self.velocity.z * time +
            self.acceleration.z * time.powf(2.0) / 2.0;
        
        if self.gps_connection {
            self.position.set_coordinates(
                self.global_position.x,
                self.global_position.y,
                self.global_position.z
            ); 
        }
    }
}

enum SuppressionFrequencyType {
    GPS
    // TODO Control
}

enum SuppressionAreaType {
    Dome(f32)
    // TODO Rifle
}

struct RadarWarfareDevice {
    position: Coordinates3D,
    frequency: SuppressionFrequencyType,
    area: SuppressionAreaType,
}

impl RadarWarfareDevice {
    fn in_area(&self, drone: &Drone) -> bool {
        let drone_position = &drone.global_position;

        match self.area {
            SuppressionAreaType::Dome(radius) => {
                let distance: f32 = (
                    (drone_position.x - self.position.x).powf(2.0) +
                    (drone_position.y - self.position.y).powf(2.0) +
                    (drone_position.z - self.position.z).powf(2.0)
                    ).sqrt();
                
                if distance <= radius { true }
                else { false }
            },
            _ => false
        }

    }
   
    fn suppress(&self, drone: &mut Drone) -> bool {
        if self.in_area(drone) {
            match self.frequency {
                SuppressionFrequencyType::GPS => {
                    drone.gps_connection = false;
                    true
                },
                _ => false
            }
        }
        else {
            false
        }
    }

}


struct World {
    command_center: CommandCenter,
    drones: Vec<Drone>,
    radar_warfare_devices: Vec<RadarWarfareDevice>,
    current_time: f32,
    end_time: f32,
}

impl World {
    fn simulate(&mut self) {
        println!("Time\tEntity\tId\tData\tX\tY\tZ");
        for (i, drone) in self.drones.iter_mut().enumerate() {
            drone.set_destination(None);
            println!("{}\tdrone\t{}\tposition\t{}\t{}\t{}",
                self.current_time,
                i,
                &drone.global_position.x,
                &drone.global_position.y,
                &drone.global_position.z
            );
            println!("{}\tdrone\t{}\tvelocity\t{}\t{}\t{}",
                self.current_time,
                i,
                &drone.velocity.x,
                &drone.velocity.y,
                &drone.velocity.z
            );
        }

        for (j, rwd) in self.radar_warfare_devices.iter().enumerate() {
            println!("{}\trwd\t{}\tposition\t{}\t{}\t{}",
                self.current_time,
                j,
                &rwd.position.x,
                &rwd.position.y,
                &rwd.position.z
            );
        }

        while self.current_time < self.end_time {
            for (i, drone) in self.drones.iter_mut().enumerate() {
                for (j, rwd) in self.radar_warfare_devices.iter().enumerate() {
                    if rwd.suppress(drone) {
                        println!("RWD{j} suppressed Drone{i}!");
                    }
                }

                drone.update_position(STEP_DURATION);
                
                println!("{}\tdrone\t{}\tposition\t{}\t{}\t{}",
                    self.current_time,
                    i,
                    &drone.global_position.x,
                    &drone.global_position.y,
                    &drone.global_position.z
                );
                println!("{}\tdrone\t{}\tvelocity\t{}\t{}\t{}",
                    self.current_time,
                    i,
                    &drone.velocity.x,
                    &drone.velocity.y,
                    &drone.velocity.z
                );

                self.current_time += STEP_DURATION;
            }
        }
    }
}

fn main() {
    let drone_command_center = CommandCenter {
        position: Coordinates3D {
            x: 200.0,
            y: 100.0,
            z: 0.0,
        }
    };

    let drone = Drone {
        max_speed: MAX_SPEED,
        global_position: Coordinates3D {
            x: 150.3,
            y: 50.6,
            z: 25.5, 
        },
        position: Coordinates3D {
            x: 150.3,
            y: 50.6,
            z: 25.5, 
        },
        velocity: Coordinates3D { x: 0.0, y: 0.0, z: 0.0, },
        acceleration: Coordinates3D { x: 0.0, y: 0.0, z: 0.0, },
        destination: Coordinates3D {
            x: 5.0,
            y: 10.0,
            z: 2.0,
        },
        // TODO infection_state: 0,
        command_center: drone_command_center.clone(),
        gps_connection: true,
        
    };

    let radar_warfare_device = RadarWarfareDevice {
        position: Coordinates3D {
            x: 0.0,
            y: 5.0,
            z: 2.0,
        },
        frequency: SuppressionFrequencyType::GPS,
        area: SuppressionAreaType::Dome(RWD_RADIUS),
    };

    let mut world = World {
        command_center: drone_command_center.clone(),
        drones: vec![drone],
        radar_warfare_devices: vec![radar_warfare_device],
        current_time: 0.0,
        end_time: MAX_TIME,    
    };

    world.simulate();
}
