mod draw;

use opengl_graphics::GlGraphics;
use piston_window::{self, Context};

use self::draw::circle;
use self::draw::polygon;
use self::draw::line;
use simulation::Simulation;
use simulation::shape::Shape;

pub fn render(context: Context, gl: &mut GlGraphics, sim: &mut Simulation) {
    piston_window::clear([0.0, 0.0, 0.0, 1.0], gl);
    for body in &sim.bodies {
        match body.shape {
            Shape::Circle(ref c) => { circle(c.pos, c.radius, [1.0, 0.0, 0.0, 1.0], context, gl); }
            Shape::Polygon(ref p) => { polygon(&p.vertices, [1.0, 0.0, 0.0, 1.0], context, gl); }
            _ => {}
        }
    }
    for coll in &sim.collision_handler.collisions {
        line(&coll.pos, &(coll.pos + coll.normal * coll.depth), [0.0, 1.0, 0.0, 1.0], context, gl);
        circle(coll.pos, 3.0, [0.0, 1.0, 0.0, 1.0], context, gl);
    }
}
