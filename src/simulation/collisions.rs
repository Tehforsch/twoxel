use simulation::body::Body;
use simulation::Simulation;

use simulation::shape::Shape;
use simulation::polygon::Polygon;
use simulation::circle::Circle;
use point::Point;

pub struct CollisionHandler {
    pub collisions: Vec<Collision>
}

#[derive(Debug)]
pub struct Collision {
    pub pos: Point,
    pub depth: f64,
    pub normal: Point
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
    let normal = circle1.pos - circle2.pos;
    let distance = normal.norm();
    let depth = distance - (circle1.radius + circle2.radius);
    if depth < 0.0 {
        Some(Collision { 
            pos: circle1.pos.middle(circle2.pos),
            depth: depth,
            normal: normal
        })
    }
    else {
        None
    }
}

fn polygon_polygon(polygon1: &Polygon, polygon2: &Polygon) -> Option<Collision> {
    let mut min_depth_collision: Option<Collision> = None;
    let mut collision_normal_is_from_polygon1 = true;
    for (i, edge) in polygon1.get_normals().iter().chain(polygon2.get_normals().iter()).enumerate() {
        let projection1 = polygon1.project(*edge);
        let projection2 = polygon2.project(*edge);
        let depth = get_depth_from_projections(projection1, projection2);
        if depth < 0.0 {
            return None
        }
        else {
            let is_deeper = match min_depth_collision {
                None => { true }
                Some(ref collision) => { depth < collision.depth }
            };
            if is_deeper {
                let normal = edge.clone();
                if (i >= polygon1.vertices.len()) {
                    collision_normal_is_from_polygon1 = false;
                }
                let reversed_normal = match (polygon1.pos - polygon2.pos) * normal < 0.0 {
                    true => { normal.clone() }
                    false => { -normal.clone() }
                };
                let collision_pos = match collision_normal_is_from_polygon1 {
                    true =>  { get_collision_pos(polygon2, reversed_normal) }
                    false => { get_collision_pos(polygon1, -reversed_normal) }
                };
                min_depth_collision = Some(Collision{
                    depth: depth, 
                    pos: collision_pos,
                    normal: reversed_normal
                });
            }
        }
    }
    min_depth_collision
}

fn get_collision_pos(polygon: &Polygon, normal: Point) -> Point {
    polygon.vertices.iter().min_by(|&x, &y| ((*x) * normal).partial_cmp(&((*y) * normal)).unwrap()).unwrap().clone()
}

fn get_depth_from_projections(projection1: [f64; 2], projection2: [f64; 2]) -> f64 {
    return (projection1[1] - projection2[0]).min(projection2[1] - projection1[0]);
}
