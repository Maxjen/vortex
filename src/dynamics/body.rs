use std::rc::Rc;
use std::cell::RefCell;
use cgmath::*;
use ::collision::PolygonShape;
use ::common::Transform2d;
use super::Fixture;

pub type BodyHandle = Rc<RefCell<Body>>;

pub struct Body {
    id: u32,

    transform: Transform2d,

    linear_velocity: Vector2<f32>,
    angular_velocity: f32,

    force: Vector2<f32>,
    torque: f32,

    mass: f32,

    fixtures: Vec<Fixture>,
}

impl Body {
    pub fn new(id: u32) -> Body {
        Body {
            id: id,

            transform: Transform2d::new(),
            linear_velocity: Vector2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vector2::new(0.0, 0.0),
            torque: 0.0,
            mass: 0.0,
            fixtures: Vec::new(),
        }
    }

    pub fn create_fixture(&mut self, shape: PolygonShape) {
        self.fixtures.push(Fixture::new(shape));
    }

    pub fn get_fixtures(&self) -> &Vec<Fixture> {
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
