use point::Point;

pub enum Shape {
    Circle(Circle),
}

#[derive(Debug)]
pub struct Circle {
    pub radius: f64
}
