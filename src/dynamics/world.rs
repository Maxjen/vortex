use std::collections::HashMap;
use std::rc::{Rc, Weak};
use std::cell::RefCell;
use cgmath::*;
use super::{Body, BodyHandle, Fixture, FixtureHandle, ContactManager};
use ::collision::{BroadPhase, PolygonShape};
use ::common::{IndexPool, DebugDraw};
use ::common::debug_draw;

pub type WorldHandle<'a> = Rc<RefCell<World<'a>>>;

pub struct World<'a> {
    pub broad_phase: BroadPhase,
    contact_manager: ContactManager,

    gravity: Vector2<f32>,

    bodies: HashMap<u32, BodyHandle<'a>>,
    body_index_pool: IndexPool,
    fixtures: HashMap<u32, FixtureHandle>,
    fixture_index_pool: IndexPool,

    debug_draw: Option<Rc<RefCell<DebugDraw + 'a>>>,
    self_handle: Option<Weak<RefCell<World<'a>>>>,
}

impl<'a> World<'a> {
    pub fn new(gravity: Vector2<f32>) -> WorldHandle<'a> {
        let result = Rc::new(RefCell::new(World {
            broad_phase: BroadPhase::new(),
            contact_manager: ContactManager::new(),
            gravity: gravity,
            bodies: HashMap::new(),
            body_index_pool: IndexPool::new(),
            fixtures: HashMap::new(),
            fixture_index_pool: IndexPool::new(),
            debug_draw: None,
            self_handle: None,
        }));
        let self_handle = Some(Rc::downgrade(&result));
        result.borrow_mut().self_handle = self_handle;
        result
    }

    pub fn create_body(&mut self) -> BodyHandle<'a> {
        let index = self.body_index_pool.get_index();
        let result = Rc::new(RefCell::new(Body::new(index)));
        result.borrow_mut().set_world_handle(self.self_handle.as_ref().unwrap().upgrade());
        self.bodies.insert(index, result.clone());
        result
    }

    pub fn delete_body(&mut self, id: u32) {
        if let Some(to_remove) = self.bodies.get(&id) {
            to_remove.borrow_mut().delete_fixtures();
        }
        if let Some(_) = self.bodies.remove(&id) {
            self.body_index_pool.recycle_index(id);
        }
    }

    pub fn create_fixture(&mut self, body_index: u32, shape: PolygonShape) -> FixtureHandle {
        let index = self.fixture_index_pool.get_index();
        let result = Rc::new(RefCell::new(Fixture::new(index, body_index, shape)));
        self.fixtures.insert(index, result.clone());
        result
    }

    pub fn delete_fixture(&mut self, id: u32) {
        if let Some(_) = self.fixtures.remove(&id) {
            self.fixture_index_pool.recycle_index(id);
        }
    }

    pub fn set_debug_draw(&mut self, debug_draw: Option<Rc<RefCell<DebugDraw + 'a>>>) {
        self.debug_draw = debug_draw;
    }

    pub fn draw_debug_data(&self) {
        if let Some(ref debug_draw) = self.debug_draw {
            for (_, b) in &self.bodies {
                let transform = b.borrow().get_transform();
                //let () = b;
                //let shape = b.borrow().get_shape();
                for f in b.borrow().get_fixtures() {
                    let f = f.borrow();
                    //let shape = f.get_shape();
                    let shape = &f.shape;
                    let mut vertices = Vec::new();
                    for v in &shape.vertices {
                        vertices.push(transform.apply_to_vector(v));
                    }
                    debug_draw.borrow_mut().draw_polygon(&vertices);
                }
            }
        }
    }

    pub fn test(&mut self) {
        self.broad_phase.update_pairs(&mut self.contact_manager);
    }
}