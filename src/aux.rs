fn read_coordinates(coordinates: &mut Coordinates) {
    let mut input = String::new();
    println!("Enter x coordinate: ");
    std::io::stdin()
        .read_line(&mut input)
        .expect("Failed to read line.");
    let x: f32 = input.trim().parse().expect("Please type a number!");
    coordinates.x = x;

    let mut input = String::new();
    println!("Enter y coordinate: ");
    std::io::stdin()
        .read_line(&mut input)
        .expect("Failed to read line.");
    let y: f32 = input.trim().parse().expect("Please type a number!");
    coordinates.y = y;

    let mut input = String::new();
    println!("Enter z coordinate: ");
    std::io::stdin()
        .read_line(&mut input)
        .expect("Failed to read line.");
    let z: f32 = input.trim().parse().expect("Please type a number!");
    coordinates.z = z;
}
