extern crate cgmath;
extern crate glium;
extern crate vortex;
extern crate inferno;

use inferno::rendering::{ColorVertex2d, DrawBatch};
use cgmath::*;
use vortex::collision::PolygonShape;
use vortex::common::{DebugDraw};
use vortex::dynamics::World;
use std::rc::Rc;
use std::cell::RefCell;

pub struct MyDebugDraw<'a> {
    draw_batch: DrawBatch<'a>,
}

impl<'a> MyDebugDraw<'a> {
    fn new(display: &'a glium::backend::glutin_backend::GlutinFacade) -> Self {
        MyDebugDraw {
            draw_batch: DrawBatch::new(display),
        }
    }

    fn draw_all(&mut self, frame: &mut glium::Frame) {
        self.draw_batch.create_buffers();
        self.draw_batch.draw(frame);
        self.draw_batch.clear();
    }
}

impl<'a> DebugDraw for MyDebugDraw<'a> {
    fn draw_polygon(&mut self, vertices: &Vec<Vector2<f32>>) {
        if vertices.len() > 2 {
            let mut vertex_buffer = Vec::new();
            for v in vertices {
                vertex_buffer.push(ColorVertex2d {
                    position: [v.x, v.y],
                    color: [255, 255, 255, 50],
                });
            }

            let mut index_buffer = Vec::new();
            for i in 1..vertices.len() - 1 {
                index_buffer.push(0);
                index_buffer.push(i as u32);
                index_buffer.push(i as u32 + 1);
            }
            self.draw_batch.add_color_2d_triangles(&vertex_buffer, &index_buffer);

            vertex_buffer.clear();
            for v in vertices {
                vertex_buffer.push(ColorVertex2d {
                    position: [v.x, v.y],
                    color: [255, 255, 255, 100],
                });
            }
            index_buffer.clear();
            for i in 0..vertices.len() - 1 {
                index_buffer.push(i as u32);
                index_buffer.push(i as u32 + 1);
            }
            index_buffer.push(vertices.len() as u32 - 1);
            index_buffer.push(0);
            self.draw_batch.add_color_2d_lines(&vertex_buffer, &index_buffer);
        }
    }
}

fn main() {
    use glium::{DisplayBuild, Surface};
    let display = glium::glutin::WindowBuilder::new()
                        .with_dimensions(800, 600)
                        .with_depth_buffer(24)
                        .build_glium().unwrap();

    let gravity = Vector2::new(0.0, -10.0);
    let world = World::new(gravity);
    let debug_draw = Rc::new(RefCell::new(MyDebugDraw::new(&display)));
    world.borrow_mut().set_debug_draw(Some(debug_draw.clone()));

    let mut vertices = Vec::<Vector2<f32>>::new();
    vertices.push(Vector2::new(650.0, -50.0));
    vertices.push(Vector2::new(650.0, -100.0));
    vertices.push(Vector2::new(700.0, -100.0));
    vertices.push(Vector2::new(750.0, -50.0));
    let mut shape = PolygonShape::new();
    shape.set(&vertices);
    let body = world.borrow_mut().create_body();
    body.borrow_mut().create_fixture(shape);

    let mut shape2 = PolygonShape::new();
    shape2.set_as_box(50.0, 50.0);
    let body2 = world.borrow_mut().create_body();
    body2.borrow_mut().create_fixture(shape2);
    body2.borrow_mut().set_transform(&Vector2::<f32>::new(500.0, -300.0), 0.8);

    world.borrow_mut().test();

    world.borrow().broad_phase.print();

    loop {
        use glium::glutin::Event;

        let mut target = display.draw();

        target.clear_color_and_depth((0.01, 0.01, 0.01, 1.0), 1.0);

        /*batch.draw(&mut target);
        window.create_buffers();
        window.draw(&mut target);
        text.add_to_batch(&mut overlay_batch);
        overlay_batch.create_buffers();
        overlay_batch.draw(&mut target);
        overlay_batch.clear();*/

        world.borrow().draw_debug_data();
        debug_draw.borrow_mut().draw_all(&mut target);

        target.finish().unwrap();

        for ev in display.poll_events() {
            match ev {
                Event::Closed => return,
                _ => ()
            }
        }
    }
}
