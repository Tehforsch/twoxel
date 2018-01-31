
extern crate rand;

use point::Point;

pub mod body;
pub mod shape;
pub mod collisions;

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
    let num_bodies = 2;
    for i in 0..num_bodies {
        let x = 300.0;
        let y = 300.0;
        let mass = 1.0;
        let radius = 100.0;
        bodies.push(body::get_circle(Point{x: x, y: y}, mass, radius));
    }
    let mut sim = Simulation::new(bodies);
    sim
}
