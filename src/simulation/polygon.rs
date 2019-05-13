use point::Point;

#[derive(Debug)]
pub struct Polygon {
    pub pos: Point,
    pub vertices: Vec<Point>,
    offsets: Vec<Point>
}

impl Polygon {
    pub fn new(vertices: Vec<Point>) -> Polygon {
        let pos = Polygon::get_center_of_mass(&vertices);
        let offsets = vertices.iter().map(|x| (*x) - pos).collect();
        Polygon {
            pos: pos,
            vertices: vertices,
            offsets: offsets
        }
    }

    pub fn project(&self, normal: Point) -> [f64;2] {
        let mut min = self.vertices[0] * normal;
        let mut max = self.vertices[0] * normal;
        for vertex in self.vertices[1..].iter() {
            let projected = (*vertex) * normal;
            if projected < min {
                min = projected;
            }
            if projected > max {
                max = projected;
            }
        }
        [min, max]
    }
    
    pub fn get_normals(&self) -> Vec<Point> {
        let mut normals = vec![];
        for (x, y) in self.vertices.iter().zip(self.vertices[1..].iter().chain([self.vertices[0]].iter())) {
            normals.push((*x - *y).normalized().orth())
        }
        normals
    }

    pub fn update_pos(&mut self, pos: Point, apos: f64) {
        self.pos = pos;
        self.vertices = self.offsets.iter().map(|x| ((*x).rotate(apos)) + pos).collect();
    }

    fn get_center_of_mass(vertices: &Vec<Point>) -> Point {
        vertices.iter().fold(Point{x:0.0, y:0.0}, |acc, &x| acc + x) / (vertices.len() as f64)
    }

    pub fn get_moment_of_inertia(&self) -> f64 {
        let mut inertia = 0.0;
        for i in 1..self.offsets.len() {
            let v1 = self.offsets[i];
            let v2 = self.offsets[(i+1) % self.offsets.len()];
            inertia += (v1.orth() * v2).abs() * (v1 * v1 + v1 * v2 + v2 * v2);
        }
        let mut norm_factor = 0.0;
        for i in 1..self.offsets.len() {
            let v1 = self.offsets[i];
            let v2 = self.offsets[(i+1) % self.offsets.len()];
            norm_factor += (v1.orth() * v2).abs();

        }

        inertia / (6.0 * norm_factor)
    // }
    }

}

#[cfg(test)]
mod tests{
    use super::*;
    use std::f64;
    #[test] 
    fn test_projection() {
        let mut vertices = vec![];
        vertices.push(Point::new(0.0, 0.0));
        vertices.push(Point::new(1.0, 0.0));
        vertices.push(Point::new(1.0, 1.0));
        vertices.push(Point::new(0.0, 1.0));
        let poly = Polygon::new(vertices);
        let projection = poly.project(Point::new(1.0, 0.0));
        assert_eq!(projection[0], 0.0);
        assert_eq!(projection[1], 1.0);
    }

    #[test]
    fn test_moment_of_inertia_square() {
        let mut vertices = vec![];
        vertices.push(Point::new(0.0, 0.0));
        vertices.push(Point::new(1.0, 0.0));
        vertices.push(Point::new(1.0, 1.0));
        vertices.push(Point::new(0.0, 1.0));
        let poly = Polygon::new(vertices);
        assert!((poly.get_moment_of_inertia() - 1.0/6.0).abs() < 0.00001);
    }

    #[test]
    fn test_moment_of_inertia_almost_circle() {
        let mut points = vec![];
        let num_points = 10000;
        for j in 0..num_points {
            let angle = 2.0 * f64::consts::PI * (j as f64) / (num_points as f64);
            points.push(Point{
                x: angle.cos(),
                y: angle.sin()
            });
        }
        let poly = Polygon::new(points);
        assert!((poly.get_moment_of_inertia() - 1.0/2.0).abs() < 0.0001);
    }
}
