use std::fmt;
use point::Point;
use simulation::shape::Shape;
use simulation::polygon::Polygon;
use simulation::circle::Circle;
use std::f64;

pub struct Body {
    pub pos: Point,
    pub vel: Point,
    pub acc: Point,
    pub apos: f64,
    pub avel: f64,
    pub aacc: f64,
    pub mass: f64,
    pub inertia: f64,
    pub shape: Shape,
    pub is_static: bool
}

impl Body {
    pub fn timestep(&mut self, dt : f64) {
        if self.is_static {
            return
        }
        self.vel += self.acc * dt;
        self.pos += self.vel * dt;
        self.acc = Point { x: 0.0, y: 0.0 };
        self.avel += self.aacc * dt;
        self.apos += self.avel * dt;
        self.aacc = 0.0;
        self.shape.update_pos(self.pos, self.apos);
    }

    pub fn apply_force(&mut self, force : Point) {
        if self.is_static {
            return
        }
        self.acc += force / self.mass;
    }

    pub fn apply_impulse(&mut self, impulse : Point) {
        if self.is_static {
            return
        }
        self.vel += impulse / self.mass;
    }

    pub fn apply_force_at(&mut self, force : Point, pos: Point) {
        if self.is_static {
            return
        }
        self.acc += force / self.mass;
        self.aacc += pos.orth() * force / self.inertia;
    }

    pub fn apply_impulse_at(&mut self, impulse : Point, pos: Point) {
        if self.is_static {
            return
        }
        self.vel += impulse / self.mass;
        self.avel += pos.orth() * impulse / self.inertia;
    }

    pub fn vel_at(&mut self, relative_pos: Point) -> Point {
        self.vel + self.avel * relative_pos.orth()
    }

    pub fn new(pos: Point, mass: f64, shape: Shape, is_static: bool) -> Body {
        Body {
            pos: pos,
            mass: mass,
            vel: Point { x: 0.0, y: 0.0 },
            acc: Point { x: 0.0, y: 0.0 },
            apos: 0.0,
            avel: 0.0,
            aacc: 0.0,
            inertia: shape.get_moment_of_inertia() * mass,
            shape: shape,
            is_static: is_static
        }
    }
}

pub fn get_circle(pos: Point, mass: f64, radius: f64) -> Body {
    Body::new(pos, mass, Shape::Circle(Circle{ pos: pos, radius:radius }), false)
}

pub fn get_polygon(vertices: Vec<Point>, mass: f64) -> Body {
    let poly = Polygon::new(vertices);
    Body::new(poly.pos, mass, Shape::Polygon(poly), if (mass == 0.0) { true } else { false })
}

pub fn get_rectangle(pos: Point, width: f64, height: f64, mass: f64) -> Body {
    let vertices = [
        Point::new(pos.x - width / 2.0, pos.y - height / 2.0),
        Point::new(pos.x + width / 2.0, pos.y - height / 2.0),
        Point::new(pos.x + width / 2.0, pos.y + height / 2.0),
        Point::new(pos.x - width / 2.0, pos.y + height / 2.0),
    ];
    get_polygon(vertices.to_vec(), mass)
}

pub fn get_regular_polygon(pos: Point, radius: f64, num_vertices: usize, mass: f64) -> Body {
    let mut vertices = vec![];
    for j in 0..num_vertices {
        let angle = 2.0 * f64::consts::PI * (j as f64) / (num_vertices as f64);
        vertices.push(Point{
            x: pos.x + radius * angle.cos(),
            y: pos.y + radius * angle.sin()
        });
    }
    let poly = Polygon::new(vertices);
    Body::new(poly.pos, mass, Shape::Polygon(poly), if (mass == 0.0) { true } else { false })
}

impl fmt::Debug for Body {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {})", self.pos.x, self.pos.y)
    }
}
