
extern crate rand;

use point::Point;

pub mod body;
pub mod shape;
pub mod collisions;
pub mod polygon;
pub mod circle;

use std::f64;

const DT : f64 = 0.01;
const GRAVITY : f64 = 10.0;
// const GRAVITY : f64 = 0.0;
const GRAVITY_DIR : Point = Point{x: 0.0, y: 1.0};
const BAUMGARTE_FACTOR : f64 = 10.0;
const NUM_ITERATIONS: usize = 1;
const COLLISION_MARGIN: f64 = 0.1;
// const COLLISION_MARGIN: f64 = 0.0;

pub struct Simulation {
    pub bodies : Vec<body::Body>,
    pub collision_handler : collisions::CollisionHandler
}

impl Simulation {
    pub fn timestep(&mut self) {
        self.handle_gravity();
        self.collision_handler.find_collisions(&mut self.bodies);
        for _ in 0..NUM_ITERATIONS {
            self.collision_handler.resolve_collisions(&mut self.bodies);
        }
        self.integrate();
    }

    pub fn integrate(&mut self) {
        for body in self.bodies.iter_mut() {
            body.timestep(DT);
        }
    }

    fn handle_gravity(&mut self) {
        for body in self.bodies.iter_mut() {
            apply_gravity(body);
        }
    }

    pub fn new(bodies: Vec<body::Body>) -> Simulation {
        Simulation{
            bodies: bodies,
            collision_handler : collisions::CollisionHandler::new()
        }
    }
}

fn apply_gravity(body : &mut body::Body) {
    // let force = GRAVITY * body.mass * Point{x: 0.0, y: 1.0};
    // let force = GRAVITY * body.mass * (Point::new(0.0, 0.0) - body.pos);
    let force = GRAVITY * body.mass * GRAVITY_DIR;
    body.apply_force(force);
}

pub fn test_collision_1() -> Simulation {
    let mut bodies : Vec<body::Body> = vec![];
    bodies.push(body::get_rectangle(Point::new(0.5, 0.5), 1.0, 1.0, 1.0));
    bodies.push(body::get_rectangle(Point::new(0.5, 2.5), 1.0, 1.0, 0.0));
    bodies[0].apos = 1.0;
    let mut sim = Simulation::new(bodies);
    sim
}

pub fn test_collision_2() -> Simulation {
    let mut bodies : Vec<body::Body> = vec![];
    let num_polygons = 10;
    for i in 0..num_polygons {
        let x = 0.1 + (i as f64) * 0.3;
        let y = -5.0 + (i as f64) * 1.3;
        let mass = 1.0;
        let radius = 0.5;
        bodies.push(body::get_regular_polygon(Point::new(x, y), radius, 3+i, mass));
    }
    bodies.push(body::get_rectangle(Point::new(0.0, 10.0), 30.0, 3.0, 0.0));
    let mut sim = Simulation::new(bodies);
    sim
}

pub fn test_collision_3() -> Simulation {
    let mut bodies : Vec<body::Body> = vec![];
    let num_polygons = 20;
    for i in 0..num_polygons {
        let x = 0.1 + (i as f64) * 0.1;
        let y = 0.0 - (i as f64) * 2.3;
        let mass = 1.0;
        // let radius = 0.707;
        let radius = 1.5;
        bodies.push(body::get_regular_polygon(Point::new(x, y), radius, 3+i%3, mass));
    }
    bodies.push(body::get_rectangle(Point::new(0.0, 10.0), 30.0, 1.0, 0.0));
    bodies.push(body::get_rectangle(Point::new(-5.0, 0.0), 1.0, 30.0, 0.0));
    bodies.push(body::get_rectangle(Point::new(5.0, 0.0), 1.0, 30.0, 0.0));
    let mut sim = Simulation::new(bodies);
    sim
}

pub fn test_collision_4() -> Simulation {
    let mut bodies : Vec<body::Body> = vec![];
    let num_boxes = 20;
    for i in 0..num_boxes {
        bodies.push(body::get_rectangle(Point::new(0.5, 0.5 - 1.4 * (i as f64)), 1.0, 1.0, 1.0));
    }
    bodies.push(body::get_rectangle(Point::new(0.5, 4.5), 5.0, 5.0, 0.0));
    let mut sim = Simulation::new(bodies);
    sim
}
