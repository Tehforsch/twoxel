
extern crate rand;

use point::Point;

pub mod body;
pub mod shape;
pub mod collisions;
pub mod polygon;
pub mod circle;

use std::f64;

const DT : f64 = 0.01;
const GRAVITY : f64 = 0.0;

pub struct Simulation {
    pub bodies : Vec<body::Body>,
    collision_handler : collisions::CollisionHandler
}

impl Simulation {
    pub fn timestep(&mut self) {
        self.handle_gravity();
        self.collision_handler.find_collisions(&mut self.bodies);
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
    let force = GRAVITY * body.mass * Point{x: 0.0, y: 1.0};
    body.apply_force(force);
}

pub fn initialize_sim() -> Simulation {
    let mut bodies : Vec<body::Body> = vec![];
    let num_circles = 0;
    for i in 0..num_circles {
        let x = 300.0;
        let y = 300.0;
        let mass = 1.0;
        let radius = 100.0;
        bodies.push(body::get_circle(Point{x: x, y: y}, mass, radius));
    }
    let num_polygons = 2;
    for i in 0..num_polygons {
        let x = 300.0;
        let y = 300.0;
        let mass = 1.0;
        let radius = 100.0;
        let num_vertices = 4;
        let mut points = vec![];
        for i in 0..num_vertices {
            points.push(Point{
                x: x + radius * (2.0 * f64::consts::PI * (i as f64) / (num_vertices as f64)).cos(),
                y: y + radius * (2.0 * f64::consts::PI * (i as f64) / (num_vertices as f64)).sin()
            });
        }
        bodies.push(body::get_polygon(Point{x:x, y:y}, points, mass));
    }
    let mut sim = Simulation::new(bodies);
    sim
}
