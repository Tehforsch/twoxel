use piston_window::ellipse::Ellipse;
use piston_window::line::Line;
use piston_window::{Context, Transformed};
use piston_window::types::Color;

use opengl_graphics::GlGraphics;

use ::point::Point;

pub fn circle(pos: Point, radius: f64, color: Color, context: Context, gl: &mut GlGraphics) {
    Ellipse {
            color: color,
            border: None,
            resolution: 128,
    }.draw(
        [0.0, 0.0, 2.0*radius, 2.0*radius],
        &Default::default(),
        context.trans(pos.x-radius, pos.y-radius).transform,
        gl);
}

pub fn line(pos1: &Point, pos2: &Point, color: Color, context: Context, gl: &mut GlGraphics) {
    let line = Line::new(color, 1.0);

    let coords: [f64; 4] = [pos1.x, pos1.y, pos2.x, pos2.y];

    line.draw(coords, &Default::default(), context.transform, gl);
}

pub fn polygon(vertices: &Vec<Point>, color: Color, context: Context, gl: &mut GlGraphics) {
    for (x, y) in vertices.iter().zip(vertices[1..].iter().chain([vertices[0]].iter())) {
        line(x, y, color, context, gl);
    }
}
