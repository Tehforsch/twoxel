use simulation::body::Body;
use simulation::Simulation;
use simulation::collision_detection;

use point::Point;
use super::{BAUMGARTE_FACTOR, ALLOWED_PENETRATION, NUM_ITERATIONS, FRICTION};

// https://stackoverflow.com/questions/30073684/how-to-get-mutable-references-to-two-array-elements-at-the-same-time/30075629
enum Pair<T> {
    Both(T, T),
    One(T),
    None,
}

impl<T> Pair<T> {
    fn unwrap(self) -> (T, T){
        match self {
            Pair::Both(a, b) => (a, b),
            _ => panic!("Pair unwrap - invalid indices")
        }
    }
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

    pub fn timestep(&mut self, bodies: &mut Vec<Body>) {
        self.find_collisions(bodies);
        for _ in 0..NUM_ITERATIONS {
            self.resolve_collisions(bodies);
        }
    }

    fn resolve_collisions(&self, bodies: &mut Vec<Body>) {
        for collision in self.collisions.iter() {
            let (b1, b2) = index_twice(bodies, collision.body1, collision.body2).unwrap();
            resolve_collision(b1, b2, collision.info);
        }
    }

    pub fn find_collisions(&mut self, bodies: &mut Vec<Body>) {
        self.collisions = vec![];
        let slice = &bodies[..];
        let length = slice.len();
        for i in 1..length {
            let (first, second) = slice.split_at(i);
            let body1 = & first[i-1];
            for (j, body2) in second.iter().enumerate() {
                if body1.is_static && body2.is_static {
                    continue
                }
                let collision_infos = collision_detection::find_collisions(body1, body2);
                let collisions = &mut collision_infos.iter().map(|&c| Collision{ info: c, body1: i-1, body2: j+i }).collect();
                self.collisions.append(collisions);
            }
        }
    }

}

fn resolve_collision(body1: &mut Body, body2: &mut Body, collision: CollisionInfo) {
    let r1 = collision.pos - body1.pos;
    let r2 = collision.pos - body2.pos;
    let inv_m1 = body1.inv_mass();
    let inv_m2 = body2.inv_mass();
    let inv_i1 = body1.inv_inertia();
    let inv_i2 = body2.inv_inertia();
    // Normal impulse
    let relative_velocity_normal = collision.normal * (body1.vel_at(r1) - body2.vel_at(r2)) + (collision.depth - ALLOWED_PENETRATION) * BAUMGARTE_FACTOR;
    let k_normal = inv_m1 + inv_m2 + (r1 * r1 - (r1 * collision.normal).powi(2)) * inv_i1 + (r2 * r2 - (r2 * collision.normal).powi(2)) * inv_i2;
    let p_normal = relative_velocity_normal / k_normal;
    let p = collision.normal * p_normal;
    if relative_velocity_normal > 0.0 {
        body1.apply_impulse_at(-p, r1);
        body2.apply_impulse_at(p, r2);
    }
    // Tangent (friction) impulse
    let tangent = collision.normal.orth();
    let relative_velocity_tangent = tangent * (body1.vel_at(r1) - body2.vel_at(r2));
    let k_tangent = inv_m1 + inv_m2 + (r1 * r1 - (r1 * tangent).powi(2)) * inv_i1 + (r2 * r2 - (r2 * tangent).powi(2)) * inv_i2;
    let p_tangent = relative_velocity_tangent / k_tangent;
    let max_p_tangent = FRICTION * p_normal;
    let p_tangent = clamp(-max_p_tangent, p_tangent, max_p_tangent);
    let p = tangent * p_tangent;
    if relative_velocity_tangent > 0.0 {
        body1.apply_impulse_at(-p, r1);
        body2.apply_impulse_at(p, r2);
    }
}

pub fn clamp(min: f64, x: f64, max: f64) -> f64 {
    min.max(x.min(max))
}

