use simulation::body::Body;
use simulation::Simulation;

use simulation::shape::Shape;
use simulation::polygon::Polygon;
use simulation::circle::Circle;
use point::Point;
use super::BAUMGARTE_FACTOR;

// https://stackoverflow.com/questions/30073684/how-to-get-mutable-references-to-two-array-elements-at-the-same-time/30075629
enum Pair<T> {
    Both(T, T),
    One(T),
    None,
}

fn index_twice<T>(slc: &mut [T], a: usize, b: usize) -> Pair<&mut T> {
    if a == b {
        slc.get_mut(a).map_or(Pair::None, Pair::One)
    } else {
        if a >= slc.len() || b >= slc.len() {
            Pair::None
        } else {
            // safe because a, b are in bounds and distinct
            unsafe {
                let ar = &mut *(slc.get_unchecked_mut(a) as *mut _);
                let br = &mut *(slc.get_unchecked_mut(b) as *mut _);
                Pair::Both(ar, br)
            }
        }
    }
}

pub struct CollisionHandler {
    pub collisions: Vec<Collision>
}

#[derive(Debug, Clone, Copy)]
pub struct CollisionInfo {
    pub pos: Point,
    pub depth: f64,
    pub normal: Point,
}

#[derive(Debug)]
pub struct Collision {
    pub info: CollisionInfo,
    pub body1: usize,
    pub body2: usize
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
            let body1 = & first[i-1];
            for (j, body2) in second.iter().enumerate() {
                collision = find_collision(body1, body2);
                collision.map(|c| self.collisions.push(
                    Collision{
                        info: c,
                        body1: i-1,
                        body2: j+i
                    }
                ));
            }
        }
    }

    pub fn resolve_collisions(&self, bodies: &mut Vec<Body>) {
        for collision in self.collisions.iter() {
            let body_pair = index_twice(bodies, collision.body1, collision.body2);
            match body_pair {
                Pair::Both(b1, b2) => self.resolve_collision(b1, b2, collision.info),
                _ => {println!("DAS SOLLTE NIE PASSIEREN HIHI");}
            }
        }
    }

    fn resolve_collision(&self, body1: &mut Body, body2: &mut Body, collision: CollisionInfo) {
        let r1 = collision.pos - body1.pos;
        let r2 = collision.pos - body2.pos;
        let relative_velocity = collision.normal * (body1.vel_at(r1) - body2.vel_at(r2)) + collision.depth * BAUMGARTE_FACTOR;
        let rn1 = r1 * collision.normal;
        let rn2 = r2 * collision.normal;
        let inv_m1 = if body1.is_static { 0.0 } else { 1.0 / body1.mass };
        let inv_m2 = if body2.is_static { 0.0 } else { 1.0 / body2.mass };
        let inv_i1 = if body1.is_static { 0.0 } else { 1.0 / body1.inertia };
        let inv_i2 = if body2.is_static { 0.0 } else { 1.0 / body2.inertia };
        let k = inv_m1 + inv_m2 + (r1 * r1 - (r1 * collision.normal).powi(2)) * inv_i1 + (r2 * r2 - (r2 * collision.normal).powi(2)) * inv_i2;
        let p = collision.normal * relative_velocity / k;
        if relative_velocity > 0.0 {
            body1.apply_impulse_at(-p, r1);
            body2.apply_impulse_at(p, r2);
        }
    }
}

fn find_collision(body1: &Body, body2: &Body) -> Option<CollisionInfo> {
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

// fn circle_circle(circle1: &Circle, circle2: &Circle) -> Option<Collision> {
//     let normal = circle1.pos - circle2.pos;
//     let distance = normal.norm();
//     let depth = distance - (circle1.radius + circle2.radius);
//     if depth < 0.0 {
//         Some(Collision { 
//             pos: circle1.pos.middle(circle2.pos),
//             depth: depth,
//             normal: normal
//         })
//     }
//     else {
//         None
//     }
// }

fn circle_circle(circle1: &Circle, circle2: &Circle) -> Option<CollisionInfo> {
    None
}

fn polygon_polygon(polygon1: &Polygon, polygon2: &Polygon) -> Option<CollisionInfo> {
    let mut min_depth_collision: Option<CollisionInfo> = None;
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
                min_depth_collision = Some(CollisionInfo{
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
    // polygon.vertices[0]
    polygon.vertices.iter().min_by(|&x, &y| ((*x) * normal).partial_cmp(&((*y) * normal)).unwrap()).unwrap().clone()
}

fn get_depth_from_projections(projection1: [f64; 2], projection2: [f64; 2]) -> f64 {
    return (projection1[1] - projection2[0]).min(projection2[1] - projection1[0]);
}
