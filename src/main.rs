// extern crate piston_window;
extern crate piston_window;
extern crate opengl_graphics;

use piston_window::{EventLoop, Input, OpenGL, PistonWindow, WindowSettings, Motion, MouseScrollEvent};
use opengl_graphics::GlGraphics;

mod point;
mod simulation;
mod render;

use point::Point;
use render::Renderer;

fn main() {
    let opengl = OpenGL::V3_2;

    let mut window: PistonWindow = WindowSettings::new(
        "Twoxel", [1024 as u32, 600 as u32])
        .opengl(opengl).samples(8).exit_on_esc(true).build().unwrap();

    window.set_ups(60);
    window.set_max_fps(60);

    let mut gl = GlGraphics::new(opengl);

    let mut sim = simulation::test_collision_3();

    let dimensions = window.output_color.get_dimensions();
    let window_dimensions = Point{x: (dimensions.0 as f64), y: (dimensions.1 as f64)};
    let mut renderer = Renderer::new(window_dimensions);

    while let Some(e) = window.next() {
        match e {
            Input::Update(_) => {
                sim.timestep();
            }

            Input::Render(args) => {
                gl.draw(args.viewport(), |context, gl| renderer.render(context, gl, &mut sim));
            }

            Input::Move(Motion::MouseCursor(x, y)) => {
                move_body(&mut sim, x, y);
            }

            _ => {}
        }
        e.mouse_scroll(|dx, dy| renderer.scale_factor = zoom(dy, renderer.scale_factor));
    }
}

fn zoom(dy: f64, scale_factor: f64) -> f64 {
    scale_factor * (1.0 + dy / 10.0)
}

fn move_body(sim: &mut simulation::Simulation, x: f64, y: f64) {
    match sim.bodies.get_mut(0) {
        Some(body) => { 
            body.pos = (point::Point{x:x, y:y} - Point::new(400.0, 400.0)) / 30.0; 
        }
        _ => {}
    }
}
