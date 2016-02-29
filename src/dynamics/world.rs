use std::collections::HashMap;
use std::rc::Rc;
use std::cell::RefCell;
use cgmath::*;
use super::{Body, BodyHandle, ContactManager};
use ::collision::BroadPhase;
use ::common::{IndexPool, DebugDraw};
use ::common::debug_draw;

pub type WorldHandle<'a> = Rc<RefCell<World<'a>>>;

pub struct World<'a> {
    broad_phase: BroadPhase,
    contact_manager: ContactManager,

    index_pool: IndexPool,
    gravity: Vector2<f32>,
    bodies: HashMap<u32, BodyHandle>,
    debug_draw: Option<Rc<RefCell<DebugDraw + 'a>>>,
}

impl<'a> World<'a> {
    pub fn new(gravity: Vector2<f32>) -> WorldHandle<'a> {
        Rc::new(RefCell::new(World {
            broad_phase: BroadPhase::new(),
            contact_manager: ContactManager::new(),
            index_pool: IndexPool::new(),
            gravity: gravity,
            bodies: HashMap::new(),
            debug_draw: None,
        }))
    }

    pub fn create_body(&mut self) -> BodyHandle {
        let index = self.index_pool.get_index();
        let result = Rc::new(RefCell::new(Body::new(index)));
        self.bodies.insert(index, result.clone());
        result
    }

    pub fn delete_body(&mut self, id: u32) {
        self.bodies.remove(&id);
        self.index_pool.recycle_index(id);
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
                    let shape = f.get_shape();
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
