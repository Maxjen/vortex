use std::rc::Rc;
use std::cell::RefCell;
use cgmath::*;
use ::collision::PolygonShape;
use ::common::Transform2d;
use super::{WorldHandleWeak, FixtureHandle};

pub type BodyHandle<'a> = Rc<RefCell<Body<'a>>>;

pub struct Body<'a> {
    id: u32,

    transform: Transform2d,

    linear_velocity: Vector2<f32>,
    angular_velocity: f32,

    force: Vector2<f32>,
    torque: f32,

    mass: f32,

    fixtures: Vec<FixtureHandle>,

    world: WorldHandleWeak<'a>,
}

impl<'a> Body<'a> {
    pub fn new(id: u32, world: WorldHandleWeak<'a>) -> Body<'a> {
        Body {
            id: id,

            transform: Transform2d::new(),
            linear_velocity: Vector2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vector2::new(0.0, 0.0),
            torque: 0.0,
            mass: 0.0,
            fixtures: Vec::new(),
            world: world,
        }
    }

    pub fn create_fixture(&mut self, shape: PolygonShape) {
        let world = match self.world.upgrade() {
            Some(world) => world,
            None => return,
        };
        let fixture = world.borrow_mut().create_fixture(self.id, shape);
        let broad_phase = &mut world.borrow_mut().broad_phase;
        fixture.borrow_mut().create_proxy(broad_phase, &self.transform);
        self.fixtures.push(fixture);
    }

    pub fn delete_fixtures(&mut self) {
        let world = match self.world.upgrade() {
            Some(world) => world,
            None => return,
        };

        for fixture in &self.fixtures {
            let broad_phase = &mut world.borrow_mut().broad_phase;
            fixture.borrow_mut().destroy_proxy(broad_phase);
            world.borrow_mut().delete_fixture(fixture.borrow().id);
        }
    }

    pub fn get_fixtures(&self) -> &Vec<FixtureHandle> {
        &self.fixtures
    }

    pub fn set_transform(&mut self, position: &Vector2<f32>, angle: f32) {
        self.transform.position = *position;
        self.transform.rotation.set_angle(angle);
    }

    pub fn get_transform(&self) -> Transform2d {
        self.transform
    }

    pub fn get_position(&self) -> Vector2<f32> {
        self.transform.position
    }

    pub fn set_linear_velocity(&mut self, linear_velocity: Vector2<f32>) {
        self.linear_velocity = linear_velocity;
    }

    pub fn get_linear_velocity(&self) -> Vector2<f32> {
        self.linear_velocity
    }

    pub fn set_angular_velocity(&mut self, angular_velocity: f32) {
        self.angular_velocity = angular_velocity;
    }

    pub fn get_angular_velocity(&self) -> f32 {
        self.angular_velocity
    }

    pub fn get_mass(&self) -> f32 {
        self.mass
    }

    pub fn apply_force_to_center(&mut self, force: &Vector2<f32>) {
        self.force = self.force + force;
    }

    pub fn apply_torque(&mut self, torque: f32) {
        self.torque += torque;
    }
}
