use point::Point;
use simulation::polygon::Polygon;
use simulation::circle::Circle;

trait Project {
    fn project(&self, normal: Point) -> [f64;2];
}

pub enum Shape {
    Circle(Circle),
    Polygon(Polygon)
}

impl Shape {
    pub fn update_pos(&mut self, pos: Point, apos: f64) {
        match *self {
            Shape::Circle(ref mut circle) => { circle.pos = pos }
            Shape::Polygon(ref mut polygon) => { polygon.update_pos(pos, apos) }
        }
    }

    pub fn get_moment_of_inertia(&self) -> f64 {
        match *self {
            Shape::Circle(ref circle) => { circle.get_moment_of_inertia() }
            Shape::Polygon(ref polygon) => { polygon.get_moment_of_inertia() }
        }
    }
}

impl Project for Shape {
    fn project(&self, normal: Point) -> [f64;2] {
        match *self {
            Shape::Circle(ref circle) => { circle.project(normal) }
            Shape::Polygon(ref polygon) => { polygon.project(normal) }
        }
    }
}

