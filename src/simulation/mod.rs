
extern crate rand;

use point::Point;

pub mod body;
pub mod shape;
pub mod collisions;
pub mod polygon;
pub mod circle;

use std::f64;

const DT : f64 = 0.01;
const GRAVITY : f64 = 1.0;

pub struct Simulation {
    pub bodies : Vec<body::Body>,
    pub collision_handler : collisions::CollisionHandler
}

impl Simulation {
    pub fn timestep(&mut self) {
        self.handle_gravity();
        // self.collision_handler.find_collisions(&mut self.bodies);
        self.collision_handler.resolve_collisions(&mut self.bodies);
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
    let force = GRAVITY * body.mass * Point::new(0.0, 1.0);
    body.apply_force(force);
}

pub fn test_collision_1() -> Simulation {
    let mut bodies : Vec<body::Body> = vec![];
    let vertices_1 = [
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(1.0, 1.0),
        Point::new(0.0, 1.0)
    ];
    let vertices_2 = [
        Point::new(0.0, 3.0),
        Point::new(1.0, 3.0),
        Point::new(1.0, 4.0),
        Point::new(0.0, 4.0)
    ];
    bodies.push(body::get_polygon(vertices_1.to_vec(), 1.0, false));
    bodies.push(body::get_polygon(vertices_2.to_vec(), 1.0, true));
    bodies[0].apos = 1.0;
    let mut sim = Simulation::new(bodies);
    sim
}

pub fn test_collision_2() -> Simulation {
    let mut bodies : Vec<body::Body> = vec![];
    let num_polygons = 3;
    for i in 0..num_polygons {
        let x = 0.1;
        let y = 0.1 + (i as f64) * 0.3;
        // let mass = 1.0;
        let mass = if (i == 0) { 1.0 } else { 1.0 };
        let radius = 1.0;
        let num_vertices = 3 + i;
        let mut points = vec![];
        for j in 0..num_vertices {
            let angle = 2.0 * f64::consts::PI * (0.5 + 0.2 * i as f64 + j as f64) / (num_vertices as f64);
            points.push(Point{
                x: x + radius * angle.cos(),
                y: y + radius * angle.sin()
            });
        }
        bodies.push(body::get_polygon(points, mass, false));
    }
    let vertices = [
        Point::new(-5.0, 10.0),
        Point::new(5.0, 10.0),
        Point::new(5.0, 11.0),
        Point::new(-5.0, 11.0)
    ];
    bodies.push(body::get_polygon(vertices.to_vec(), 1.0, true));
    let mut sim = Simulation::new(bodies);
    sim
}

