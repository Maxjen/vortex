use std::rc::Rc;
use std::cell::RefCell;
use ::collision::{BroadPhase, PolygonShape};
use ::common::Transform2d;

pub type FixtureHandle = Rc<RefCell<Fixture>>;

pub struct Fixture {
    pub id: u32,
    pub body_id: u32,
    //density: f32,
    pub shape: PolygonShape,
    /*friction: f32,
    restitution: f32,*/
    pub proxy_id: u32,
}

impl Fixture {
    pub fn new(id: u32, body_id: u32, shape: PolygonShape) -> Self {
        Fixture {
            id: id,
            body_id: body_id,
            shape: shape,
            proxy_id: 0,
        }
    }

    /*pub fn get_shape(&self) -> &PolygonShape {
        &self.shape
    }*/

    pub fn create_proxy(&mut self, broad_phase: &mut BroadPhase, transform: &Transform2d) {
        let aabb = self.shape.compute_aabb(transform);
        self.proxy_id = broad_phase.create_proxy(&aabb, Some(self.id));
    }

    pub fn destroy_proxy(&mut self, broad_phase: &mut BroadPhase) {
        broad_phase.destroy_proxy(self.proxy_id);
    }
}
