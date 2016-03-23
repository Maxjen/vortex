use std::collections::HashMap;
use std::rc::{Rc, Weak};
use std::cell::RefCell;
use std::mem;
use std::ptr;
use cgmath::*;
use super::{Body, BodyHandle, Fixture, FixtureHandle, ContactManager, Island};
use ::collision::{BroadPhase, PolygonShape};
use ::common::{IndexPool, Timer, DebugDraw};
use ::common::debug_draw;

pub type WorldHandle<'a> = Rc<RefCell<World<'a>>>;
pub type WorldHandleWeak<'a> = Weak<RefCell<World<'a>>>;

#[derive(Default)]
pub struct Profile {
    step: f32,
    collide: f32,
    solve: f32,
    pub solve_init: f32,
    solve_velocity: f32,
    solve_position: f32,
    broad_phase: f32,
    solve_toi: f32,
}

#[derive(Clone, Copy)]
pub struct TimeStep {
    // time step
    pub dt: f32,
    // inverse time step (0 if dt == 0)
    inv_dt: f32,
    // dt * inv_dt0
    dt_ratio: f32,
    pub velocity_iterations: u32,
    position_iterations: u32,
    pub warm_starting: bool,
}

pub struct World<'a> {
    pub broad_phase: BroadPhase<'a>,
    contact_manager: ContactManager<'a>,

    gravity: Vector2<f32>,

    bodies: HashMap<u32, BodyHandle<'a>>,
    body_index_pool: IndexPool,
    fixtures: HashMap<u32, FixtureHandle<'a>>,
    fixture_index_pool: IndexPool,

    debug_draw: Option<Rc<RefCell<DebugDraw + 'a>>>,
    self_handle: Option<WorldHandleWeak<'a>>,

    // This is used to compute the time step ratio to support a variable time step.
    inv_dt0: f32,

    // These are used for debugging the solver.
    warm_starting: bool,

    step_complete: bool,

    profile: Profile,
}

impl<'a> World<'a> {
    pub fn new(gravity: Vector2<f32>) -> WorldHandle<'a> {
        let result;
        unsafe {
            result = Rc::new(RefCell::new(World {
                broad_phase: BroadPhase::new(),
                contact_manager: mem::uninitialized(),
                gravity: gravity,
                bodies: HashMap::new(),
                body_index_pool: IndexPool::new(),
                fixtures: HashMap::new(),
                fixture_index_pool: IndexPool::new(),
                debug_draw: None,
                self_handle: None,
                inv_dt0: 0.0,
                warm_starting: true,
                step_complete: true,
                profile: Default::default(),
            }));
            let self_handle = Rc::downgrade(&result);
            let contact_manager = ContactManager::new(self_handle.clone());
            ptr::write(&mut result.borrow_mut().contact_manager, contact_manager);
            //ptr::write(&mut result.borrow_mut().self_handle, self_handle);
            result.borrow_mut().self_handle = Some(self_handle);
        }
        result
    }

    pub fn create_body(&mut self) -> BodyHandle<'a> {
        let index = self.body_index_pool.get_index();
        let world = self.self_handle.as_ref().unwrap().clone();
        let result = Rc::new(RefCell::new(Body::new(index, world)));
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

    pub fn get_body(&self, id: u32) -> Option<BodyHandle<'a>> {
        if let Some(body) = self.bodies.get(&id) {
            return Some(body.clone());
        } else {
            return None;
        }
    }

    pub fn create_fixture(&mut self, body_index: u32, shape: PolygonShape) -> FixtureHandle<'a> {
        let index = self.fixture_index_pool.get_index();
        let body = Rc::downgrade(self.bodies.get(&body_index).unwrap());
        let result = Rc::new(RefCell::new(Fixture::new(index, body, shape)));
        self.fixtures.insert(index, result.clone());
        result
    }

    pub fn delete_fixture(&mut self, id: u32) {
        if let Some(_) = self.fixtures.remove(&id) {
            self.fixture_index_pool.recycle_index(id);
        }
    }

    pub fn get_fixture(&self, id: u32) -> Option<FixtureHandle<'a>> {
        if let Some(fixture) = self.fixtures.get(&id) {
            return Some(fixture.clone());
        } else {
            return None;
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
                        vertices.push(transform.apply(v));
                    }
                    debug_draw.borrow_mut().draw_polygon(&vertices);
                }
            }
        }
    }

    fn solve(&mut self, step: &TimeStep) {
        self.profile.solve_init = 0.0;
        self.profile.solve_velocity = 0.0;
        self.profile.solve_position = 0.0;

        // Size the island for the worst case.
        let mut island = Island::new(self.bodies.len(), self.contact_manager.get_contact_count());

        // Clear all the island flags.
        for (_, b) in &self.bodies {
            b.borrow_mut().is_island = false;
        }
        for c in &self.contact_manager.contacts {
            c.borrow_mut().is_island = false;
        }

        // Build and simulate all awake islands.
        let mut stack: Vec<BodyHandle> = Vec::with_capacity(self.bodies.len());
        for (_, seed) in &self.bodies {
            if seed.borrow().is_island {
                continue;
            }

            // Reset island and stack.
            island.clear();
            stack.clear();
            stack.push(seed.clone());
            seed.borrow_mut().is_island = true;

            // Perform a depth first search (DFS) on the constraint graph.
            while stack.len() > 0 {
                // Grab the next body off the stack and add it to the island.
                let b = stack.pop().unwrap();
                island.add_body(Rc::downgrade(&b));

                // Search all contacts connected to this body.
                for ce in b.borrow().get_contact_edges() {
                    let contact = ce.contact.upgrade().unwrap();

                    // Has this contact already been added to an island?
                    if contact.borrow().is_island {
                        continue;
                    }

                    // TODO: check is_enabled
                    // Is this contact solid and touching?
                    if !contact.borrow().is_touching() {
                        continue
                    }

                    island.add_contact(Rc::downgrade(&contact));
                    contact.borrow_mut().is_island = true;

                    let other = ce.body.upgrade().unwrap();

                    // Was the other body already added to this island?
                    if other.borrow().is_island {
                        continue;
                    }

                    other.borrow_mut().is_island = true;
                    stack.push(other);
                }
            }
        }
    }

    pub fn step(&mut self, dt: f32, velocity_iterations: u32, position_iterations: u32) {
        let step_timer = Timer::new();

        let inv_dt = if dt > 0.0 {
            1.0 / dt
        } else {
            0.0
        };

        let step = TimeStep {
            dt: dt,
            inv_dt: inv_dt,
            dt_ratio: self.inv_dt0 * dt,
            velocity_iterations: velocity_iterations,
            position_iterations: position_iterations,
            warm_starting: self.warm_starting,
        };

        // Update contacts. This is where some contacts are destroyed.
        {
            let timer = Timer::new();
            self.contact_manager.collide();
            self.profile.collide = timer.get_milliseconds();
        }

        // Integrate velocities, solve velocity constraints, and integrate positions.
        if self.step_complete && step.dt > 0.0 {
            let timer = Timer::new();
            self.solve(&step);
            self.profile.solve = timer.get_milliseconds();
        }
    }

    pub fn test(&mut self) {
        self.broad_phase.update_pairs(&mut self.contact_manager);
    }
}
