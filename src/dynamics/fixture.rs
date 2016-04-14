use std::rc::Rc;
use std::cell::RefCell;
use ::collision::{BroadPhase, Shape, PolygonShape, Aabb};
use ::common::Transform2d;
use super::BodyHandleWeak;
use cgmath::*;

pub type FixtureHandle<'a> = Rc<RefCell<Fixture<'a>>>;

pub struct FixtureConfig {
    pub friction: f32,
    pub restitution: f32,
    pub density: f32,
    pub is_sensor: bool,
}

impl Default for FixtureConfig {
    fn default() -> FixtureConfig {
        FixtureConfig {
            friction: 0.2,
            restitution: 0.0,
            density: 0.0,
            is_sensor: false,
        }
    }
}

pub struct Fixture<'a> {
    pub body: BodyHandleWeak<'a>,
    pub shape: Shape,
    pub friction: f32,
    pub restitution: f32,
    pub density: f32,
    pub is_sensor: bool,
    pub proxy_id: Option<u32>,
}

impl<'a> Fixture<'a> {
    pub fn new(body: BodyHandleWeak<'a>, shape: Shape, fixture_config: &FixtureConfig) -> Self {
        Fixture {
            body: body,
            shape: shape,
            friction: fixture_config.friction,
            restitution: fixture_config.restitution,
            density: fixture_config.density,
            is_sensor: fixture_config.is_sensor,
            proxy_id: None,
        }
    }

    /// Creates proxy in the broad phase.
    pub fn create_proxy(&mut self, self_handle: FixtureHandle<'a>, broad_phase: &mut BroadPhase<'a>, transform: &Transform2d) {
        if let None = self.proxy_id {
            let aabb = self.shape.compute_aabb(transform);
            self.proxy_id = Some(broad_phase.create_proxy(&aabb, self_handle));
        }
    }

    /// Destroys proxy in the broad phase.
    pub fn destroy_proxy(&mut self, broad_phase: &mut BroadPhase) {
        if let Some(proxy_id) = self.proxy_id {
            broad_phase.destroy_proxy(proxy_id);
        }
        self.proxy_id = None;
    }

    pub fn synchronize(&self, broad_phase: &mut BroadPhase<'a>, transform1: &Transform2d, transform2: &Transform2d) {
        if let Some(proxy_id) = self.proxy_id {
            // Compute an Aabb that covers the swept shape (may miss some rotation effect).
            let aabb1 = self.shape.compute_aabb(transform1);
            let aabb2 = self.shape.compute_aabb(transform2);

            let proxy_aabb = Aabb::combine(&aabb1, &aabb2);

            let displacement = transform2.position - transform1.position;

            broad_phase.move_proxy(proxy_id, &proxy_aabb, displacement);
        }
    }

    pub fn get_mass_data(&self) -> (f32, Vector2<f32>, f32) {
        self.shape.compute_mass(self.density)
    }

    /*pub fn get_shape(&self) -> &PolygonShape {
        &self.shape
    }*/
}
