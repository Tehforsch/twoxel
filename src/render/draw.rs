use piston_window::ellipse::Ellipse;
use piston_window::line::Line;
use piston_window::line::Shape;
use piston_window::{Context, Transformed};
use piston_window::types::Color;

use opengl_graphics::GlGraphics;

use ::point::Point;

pub fn circle(pos: Point, radius: f64, color: Color, context: Context, gl: &mut GlGraphics) {
    Ellipse {
            color: color,
            border: None,
            resolution: 16,
    }.draw(
        [0.0, 0.0, 2.0*radius, 2.0*radius],
        &Default::default(),
        context.trans(pos.x-radius, pos.y-radius).transform,
        gl);
}

pub fn line(pos1: Point, pos2: Point, color: Color, context: Context, gl: &mut GlGraphics) {
    let min_x = pos1.x.min(pos2.x);
    let min_y = pos1.y.min(pos2.y);
    let max_x = pos1.x.max(pos2.x);
    let max_y = pos1.y.max(pos2.y);
    Line {
            color: color,
            radius : 1.0,
            shape: Shape::Square
    }.draw(
        [0.0, 0.0, max_x - min_x, max_y - min_y],
        &Default::default(),
        context.trans(min_x, min_y).transform,
        gl);
}
