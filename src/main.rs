extern crate cgmath;
extern crate glium;
extern crate vortex;
extern crate inferno;
extern crate time;

use inferno::rendering::{ColorVertex2d, DrawBatch};
use cgmath::*;
use vortex::collision::{Shape, PolygonShape};
use vortex::common::{DebugDraw};
use vortex::dynamics::{BodyType, BodyConfig, FixtureConfig, World, WorldHandle};
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
                    position: [v.x * 100.0, v.y * 100.0],
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
                    position: [v.x * 100.0, v.y * 100.0],
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

    fn draw_segment(&mut self, v1: &Vector2<f32>, v2: &Vector2<f32>) {
        let mut vertex_buffer = Vec::new();
        vertex_buffer.push(ColorVertex2d {
            position: [v1.x * 100.0, v1.y * 100.0],
            color: [255, 255, 255, 50],
        });
        vertex_buffer.push(ColorVertex2d {
            position: [v2.x * 100.0, v2.y * 100.0],
            color: [255, 255, 255, 50],
        });

        let index_buffer = vec![0, 1];

        self.draw_batch.add_color_2d_lines(&vertex_buffer, &index_buffer);
    }
}

fn create_box(world: WorldHandle, position: Vector2<f32>) {
    let body_config = BodyConfig {
        body_type: BodyType::Dynamic,
        position: position,
        .. BodyConfig::default()
    };
    let body = world.borrow_mut().create_body(&body_config);

    let mut shape = PolygonShape::new();
    shape.set_as_box(0.2, 0.2);
    let fixture_config = FixtureConfig {
        friction: 0.3,
        density: 1.0,
        .. FixtureConfig::default()
    };
    body.borrow_mut().create_fixture(Shape::Polygon(shape), &fixture_config);
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

    let body1_config = BodyConfig {
        body_type: BodyType::Dynamic,
        position: Vector2::new(6.1, -0.5),
        .. BodyConfig::default()
    };
    let body1 = world.borrow_mut().create_body(&body1_config);

    let mut vertices = Vec::<Vector2<f32>>::new();
    vertices.push(Vector2::new(-0.5, 0.0));
    vertices.push(Vector2::new(-0.5, -0.5));
    vertices.push(Vector2::new(0.0, -0.5));
    vertices.push(Vector2::new(0.5, 0.0));
    let mut shape = PolygonShape::new();
    shape.set(&vertices);
    let fixture1_config = FixtureConfig {
        friction: 0.3,
        density: 1.0,
        .. FixtureConfig::default()
    };
    body1.borrow_mut().create_fixture(Shape::Polygon(shape), &fixture1_config);

    let body2_config = BodyConfig {
        position: Vector2::new(3.0, -6.0),
        //angle: 0.8,
        .. BodyConfig::default()
    };
    let body2 = world.borrow_mut().create_body(&body2_config);

    let mut shape2 = PolygonShape::new();
    shape2.set_as_box(3.0, 0.5);
    let fixture2_config = FixtureConfig::default();
    body2.borrow_mut().create_fixture(Shape::Polygon(shape2), &fixture2_config);

    create_box(world.clone(), vec2(1.0, 0.0));
    create_box(world.clone(), vec2(2.0, 0.0));

    world.borrow().broad_phase.print();

    let ticks_per_second = 60;
    let skip_ticks = 1000000000 / ticks_per_second;
    let max_frame_skip = 10;

    let mut next_tick = time::precise_time_ns() - 1;
    let mut loops;

    let mut mouse_x: i32 = 0;
    let mut mouse_y: i32 = 0;

    loop {
        loops = 0;
        while time::precise_time_ns() > next_tick && loops < max_frame_skip {
            use glium::glutin::Event;
            for ev in display.poll_events() {
                match ev {
                    Event::Closed => return,
                    Event::MouseMoved(x, y) => {
                        mouse_x = x;
                        mouse_y = y;
                    }
                    Event::MouseInput(glium::glutin::ElementState::Pressed, glium::glutin::MouseButton::Left) => {
                        let position = vec2(mouse_x as f32 / 100.0, -mouse_y as f32 / 100.0);
                        create_box(world.clone(), position);
                   }
                    _ => ()
                }
            }

            world.borrow_mut().step(1.0 / ticks_per_second as f32, 8, 3);

            next_tick += skip_ticks;
            loops += 1;
        }

        if loops != 0 {
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
        }
    }
}
