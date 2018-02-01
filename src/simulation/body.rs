use std::fmt;
use point::Point;
use simulation::shape::Shape;
use simulation::polygon::Polygon;
use simulation::circle::Circle;

pub struct Body {
    pub pos: Point,
    pub vel: Point,
    pub acc: Point,
    pub mass: f64,
    pub shape: Shape
}

impl Body {
    pub fn timestep(&mut self, dt : f64) {
        self.vel += self.acc * dt;
        self.pos += self.vel * dt;
        self.acc = Point { x: 0.0, y: 0.0 };
        self.shape.update_pos(self.pos);
    }

    pub fn apply_force(&mut self, force : Point) {
        self.acc += force / self.mass;
    }

    pub fn apply_impulse(&mut self, impulse : Point) {
        self.vel += impulse / self.mass;
    }
}

pub fn get_circle(pos: Point, mass: f64, radius: f64) -> Body {
    Body {
        pos: pos,
        vel: Point { x: 0.0, y: 0.0 },
        acc: Point { x: 0.0, y: 0.0 },
        mass: mass,
        shape: Shape::Circle(Circle{ pos: pos, radius:radius })
    }
}

pub fn get_polygon(pos: Point, vertices: Vec<Point>, mass: f64) -> Body {
    Body {
        pos: pos,
        vel: Point { x: 0.0, y: 0.0 },
        acc: Point { x: 0.0, y: 0.0 },
        mass: mass,
        shape: Shape::Polygon(Polygon::new(vertices))
    }
}

impl fmt::Debug for Body {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {})", self.pos.x, self.pos.y)
    }
}
