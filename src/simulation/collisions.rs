use simulation::body::Body;
use simulation::Simulation;

use simulation::shape::Shape;
use simulation::polygon::Polygon;
use simulation::circle::Circle;
use point::Point;

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
        Shape::Circle(ref circle1) => {
            match body2.shape {
                Shape::Circle(ref circle2) => { circle_circle(&circle1, &circle2) }
                _ => None
            }
        }
        Shape::Polygon(ref polygon1) => {
            match body2.shape {
                Shape::Polygon(ref polygon2) => { polygon_polygon(&polygon1, &polygon2) }
                _ => None
            }
        }
        _ => None
    }
}

fn circle_circle(circle1: &Circle, circle2: &Circle) -> Option<Collision> {
    let distance = (circle1.pos - circle2.pos).norm();
    let depth = distance - (circle1.radius + circle2.radius);
    if depth < 0.0 {
        Some(Collision { 
            pos: circle1.pos.middle(circle2.pos) ,
            depth: depth
        })
    }
    else {
        None
    }
}

fn polygon_polygon(polygon1: &Polygon, polygon2: &Polygon) -> Option<Collision> {
    for edge in polygon1.get_edges().iter().chain(polygon2.get_edges().iter()) {
        let projection1 = polygon1.project(*edge);
        let projection2 = polygon2.project(*edge);
        if projection1[1] < projection2[0] || projection2[1] < projection1[0] {
            return None
        }
    }
    Some(Collision{ pos:Point{x:0.0, y:0.0}, depth: 0.0 })
}
