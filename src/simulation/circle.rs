use point::Point;

#[derive(Debug)]
pub struct Circle {
    pub pos: Point,
    pub radius: f64
}

impl Circle {
    pub fn project(&self, normal: Point) -> [f64;2] {
        let projected = self.pos * normal;
        [projected - self.radius, projected + self.radius]
    }
}
