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

    pub fn get_moment_of_inertia(&self) -> f64 {
        return 0.5 * self.radius.powi(2)
    }
}
