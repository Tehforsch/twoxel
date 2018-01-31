extern crate piston_window;
extern crate opengl_graphics;

use piston_window::{EventLoop, Input, OpenGL, PistonWindow, WindowSettings, Motion};
use opengl_graphics::GlGraphics;

mod point;
mod simulation;
mod render;

fn main() {
    let opengl = OpenGL::V3_2;

    let mut window: PistonWindow = WindowSettings::new(
        "Twoxel", [1024 as u32, 600 as u32])
        .opengl(opengl).samples(8).exit_on_esc(true).build().unwrap();

    window.set_ups(60);
    window.set_max_fps(60);

    let mut gl = GlGraphics::new(opengl);

    let mut sim = simulation::initialize_sim();

    while let Some(e) = window.next() {
        match e {
            Input::Update(_) => {
                sim.timestep();
            }

            Input::Render(args) => {
                gl.draw(args.viewport(), |context, gl| render::render(context, gl, &mut sim));
            }

            Input::Move(Motion::MouseCursor(x, y)) => {
                move_body(&mut sim, x, y);
            }
            _ => {}
        }
    }
}

fn move_body(sim: &mut simulation::Simulation, x: f64, y: f64) {
    match sim.bodies.get_mut(0) {
        Some(body) => { 
            body.pos = point::Point{x:x, y:y}; 
        }
        _ => {}
    }
}
