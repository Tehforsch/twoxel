use simulation::body::Body;
use simulation::Simulation;

use simulation::shape;
use simulation::Point;

pub struct CollisionHandler {
    collisions: Vec<Collision>
}

#[derive(Debug)]
pub struct Collision {
    pos: Point,
    depth: f64
}

impl CollisionHandler{
    pub fn new() -> CollisionHandler {
        CollisionHandler {
            collisions: vec![]
        }
    }
    pub fn find_collisions(&mut self, bodies: &mut Vec<Body>) {
        self.collisions = vec![];
        // let mut slice = &mut bodies[..];
        // let length = slice.len();
        // for i in 1..length {
        //     let (mut first, second) = slice.split_at_mut(i);
        //     let first_length = first.len();
        //     let mut body1 = &mut first[first_length-1];
        //     for mut body2 in second {
        //         find_collision(body1, body2);
        //     }
        // }
        let slice = &bodies[..];
        let length = slice.len();
        let mut collision = None;
        for i in 1..length {
            let (first, second) = slice.split_at(i);
            let first_length = first.len();
            let body1 = & first[first_length-1];
            for body2 in second {
                collision = find_collision(body1, body2);
            }
        }
        match collision {
            Some(c) => { self.collisions.push(c); }
            None => {}
        }
        println!("{:?}", self.collisions)
    }
}

fn find_collision(body1: &Body, body2: &Body) -> Option<Collision> {
    match body1.shape {
        shape::Shape::Circle(ref circle1) => {
            match body2.shape {
                shape::Shape::Circle(ref circle2) => { circle_circle(body1.pos, body2.pos, &circle1, &circle2) }
                _ => None
            }
        }
        _ => None
    }
}

fn circle_circle(pos1: Point, pos2: Point, circle1: &shape::Circle, circle2: &shape::Circle) -> Option<Collision> {
    let distance = (pos1 - pos2).norm();
    let depth = distance - (circle1.radius + circle2.radius);
    println!("{}", depth);
    if depth < 0.0 {
        Some(Collision { 
            pos: pos1.middle(pos2) ,
            depth: depth
        })
    }
    else {
        None
    }
}
