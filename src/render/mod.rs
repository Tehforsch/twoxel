mod draw;

use opengl_graphics::GlGraphics;
use piston_window::{self, Context};

use self::draw::circle;
use self::draw::polygon;
use self::draw::line;
use simulation::Simulation;
use simulation::shape::Shape;
use point::Point;

pub struct Renderer {
    pub center: Point,
    pub scale_factor: f64, 
    pub window_dimensions: Point
}

impl Renderer {
    pub fn render(&self, context: Context, gl: &mut GlGraphics, sim: &mut Simulation) {
        piston_window::clear([0.0, 0.0, 0.0, 1.0], gl);

        let center = Point::new(0.0, 0.0);

        for body in &sim.bodies {
            match body.shape {
                Shape::Circle(ref c) => { circle(self.transform(c.pos), c.radius * self.scale_factor, [1.0, 0.0, 0.0, 1.0], context, gl); }
                Shape::Polygon(ref p) => { polygon(&p.vertices.iter().map(|&v| self.transform(v)).collect(), [1.0, 0.0, 0.0, 1.0], context, gl); }
                _ => {}
            }
        }
        // for coll in &sim.collision_handler.collisions {
            // line(&self.transform(coll.info.pos), &self.transform(coll.info.pos + coll.info.normal * coll.info.depth), [0.0, 1.0, 0.0, 1.0], context, gl);
            // circle(self.transform(coll.info.pos), 3.0, [0.0, 1.0, 0.0, 1.0], context, gl);
        // }
    }

    pub fn transform(&self, point: Point) -> Point {
        (point - self.center) * self.scale_factor + Point::new(self.window_dimensions.x / 2.0, self.window_dimensions.y / 2.0)
    }

    pub fn new(window_dimensions: Point) -> Renderer {
        Renderer { 
            scale_factor: 30.0,
            center: Point::new(0.0, 0.0),
            window_dimensions: window_dimensions
        }
    }
}
