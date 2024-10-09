use std::{thread, time};

const MAX_HEIGHT_IN_METRES: f32          = 50.0;
const MAX_SPEED_IN_METRES_PER_S: f32     = 25.0;
const MAX_TIME_IN_MILLIS: u64            = 10000;
const STEP_DURATION_IN_MILLIS: u64       = 250;
const RWD_RADIUS_IN_METRES: f32          = 20.0;
const DESTINATION_RADIUS_IN_METRES: f32  = 5.0;

#[derive(Clone)]
struct Coordinates3D { 
    x: f32, 
    y: f32, 
    z: f32, 
}

impl Coordinates3D {
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

    fn distance(&self, destination: &Coordinates3D) -> f32 {
        let difference = Coordinates3D {
            x: destination.x - self.x,
            y: destination.y - self.y,
            z: destination.z - self.z 
        };

        difference.size() 
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
    network_connection: bool,
    gps_connection: bool,
}

impl Drone {
    fn set_destination(&mut self, destination: Option<&Coordinates3D>) {
        match destination {
            Some(coordinates) => {
                self.destination.x = coordinates.x;
                self.destination.y = coordinates.y;
                self.destination.z = coordinates.z;
            },
            None => (),
        }

        self.velocity.x = self.destination.x - self.position.x;
        self.velocity.y = self.destination.y - self.position.y;
        self.velocity.z = self.destination.z - self.position.z;
        
        self.velocity.truncate_vector_size(self.max_speed);
        // TODO change acceleration
    }
    
    fn update_position(&mut self, time_in_millis: u64) {
        let time_in_secs = time_in_millis as f32 / 1000.0;

        self.global_position.x += self.velocity.x * time_in_secs +
            self.acceleration.x * time_in_secs.powf(2.0) / 2.0;
        self.global_position.y += self.velocity.y * time_in_secs +
            self.acceleration.y * time_in_secs.powf(2.0) / 2.0;
        self.global_position.z += self.velocity.z * time_in_secs +
            self.acceleration.z * time_in_secs.powf(2.0) / 2.0;
        
        if self.gps_connection {
            self.set_destination(None);

            self.position.x = self.global_position.x;
            self.position.y = self.global_position.y;
            self.position.z = self.global_position.z;
            
            self.reached_destination();
        }
        else {
            self.velocity.z = 0.0;
            self.acceleration.z = 0.0;

            self.velocity.truncate_vector_size(self.max_speed);
            // TODO truncate acceleration
        }
    }

    fn reached_destination(&mut self) {
        let distance_to_destination = self.position.distance(&self.destination); 
        
        if distance_to_destination <= DESTINATION_RADIUS_IN_METRES {
            self.network_connection = false;
        }
    }
}

enum SuppressionFrequencyType {
    GPS,
    Control
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
            }
        }

    }
   
    fn suppress(&self, drone: &mut Drone) -> bool {
        if self.in_area(drone) {
            match self.frequency {
                SuppressionFrequencyType::GPS => {
                    drone.gps_connection = false;
                    true
                },
                SuppressionFrequencyType::Control => {
                    drone.network_connection = false;
                    true
                },
            }
        }
        else { false }
    }

}


struct World {
    command_center: CommandCenter,
    drones: Vec<Drone>,
    radar_warfare_devices: Vec<RadarWarfareDevice>,
    current_time_in_millis: u64,
    end_time_in_millis: u64,
}

impl World {
    fn simulate(&mut self) {
        let step_duration = time::Duration::from_millis(STEP_DURATION_IN_MILLIS);

        println!("Time\tEntity\tId\tData\tX\tY\tZ");
        for (i, drone) in self.drones.iter_mut().enumerate() {
            drone.set_destination(None);
            println!("{}\tdrone\t{}\tposition\t{}\t{}\t{}",
                self.current_time_in_millis,
                i,
                &drone.global_position.x,
                &drone.global_position.y,
                &drone.global_position.z
            );
            println!("{}\tdrone\t{}\tvelocity\t{}\t{}\t{}",
                self.current_time_in_millis,
                i,
                &drone.velocity.x,
                &drone.velocity.y,
                &drone.velocity.z
            );
        }

        for (j, rwd) in self.radar_warfare_devices.iter().enumerate() {
            println!("{}\trwd\t{}\tposition\t{}\t{}\t{}",
                self.current_time_in_millis,
                j,
                &rwd.position.x,
                &rwd.position.y,
                &rwd.position.z
            );
        }

        while self.current_time_in_millis < self.end_time_in_millis {
            for (i, drone) in self.drones.iter_mut().enumerate() {
                let mut gps_suppressed: bool = false;
                let mut network_connection_suppressed: bool = false;

                drone.update_position(STEP_DURATION_IN_MILLIS);
                
                if drone.network_connection {
                    println!("{}\tdrone\t{}\tposition\t{}\t{}\t{}",
                        self.current_time_in_millis,
                        i,
                        &drone.global_position.x,
                        &drone.global_position.y,
                        &drone.global_position.z
                    );
                    println!("{}\tdrone\t{}\tvelocity\t{}\t{}\t{}",
                        self.current_time_in_millis,
                        i,
                        &drone.velocity.x,
                        &drone.velocity.y,
                        &drone.velocity.z
                    );
                }
                else {
                    println!("[INFO] Drone{} reached destination", i);
                    // TODO remove drone from network
                    continue;
                }
                
                for (j, rwd) in self.radar_warfare_devices.iter().enumerate() {
                    if rwd.suppress(drone) &&
                        (!gps_suppressed || !network_connection_suppressed) {
                        match rwd.frequency {
                            SuppressionFrequencyType::GPS =>
                                gps_suppressed = true,
                            SuppressionFrequencyType::Control => 
                                network_connection_suppressed = true
                        }
                        println!("[INFO] RWD{j} suppressed Drone{i}!");
                    }
                }

                if !gps_suppressed {
                    drone.gps_connection = true;
                }
                if !network_connection_suppressed {
                    drone.network_connection = true;
                }
            }

            thread::sleep(step_duration);

            self.current_time_in_millis += STEP_DURATION_IN_MILLIS;
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
        max_speed: MAX_SPEED_IN_METRES_PER_S,
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
        network_connection: true,
        gps_connection: true,
        
    };

    let radar_warfare_device = RadarWarfareDevice {
        position: Coordinates3D {
            x: 0.0,
            y: 5.0,
            z: 2.0,
        },
        frequency: SuppressionFrequencyType::GPS,
        area: SuppressionAreaType::Dome(RWD_RADIUS_IN_METRES),
    };

    let mut world = World {
        command_center: drone_command_center.clone(),
        drones: vec![drone],
        radar_warfare_devices: vec![radar_warfare_device],
        current_time_in_millis: 0,
        end_time_in_millis: MAX_TIME_IN_MILLIS,    
    };

    world.simulate();
}
