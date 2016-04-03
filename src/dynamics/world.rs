use std::collections::HashMap;
use std::rc::{Rc, Weak};
use std::cell::RefCell;
use std::mem;
use std::ptr;
use cgmath::*;
use super::{Body, BodyHandle, Fixture, FixtureHandle, ContactManager, Island};
use ::collision::{BroadPhase, Shape};
use ::common;
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
    pub solve_velocity: f32,
    pub solve_position: f32,
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
    pub dt_ratio: f32,
    pub velocity_iterations: u32,
    pub position_iterations: u32,
    pub warm_starting: bool,
}

pub struct World<'a> {
    pub broad_phase: BroadPhase<'a>,
    contact_manager: ContactManager<'a>,

    gravity: Vector2<f32>,
    allow_sleep: bool,

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
    continuous_physics: bool,

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
                allow_sleep: true,
                bodies: HashMap::new(),
                body_index_pool: IndexPool::new(),
                fixtures: HashMap::new(),
                fixture_index_pool: IndexPool::new(),
                debug_draw: None,
                self_handle: None,
                inv_dt0: 0.0,
                warm_starting: true,
                continuous_physics: true,
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

    pub fn create_fixture(&mut self, body_index: u32, shape: Shape) -> FixtureHandle<'a> {
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
                    if let Shape::Polygon(ref shape) = f.shape {
                        let mut vertices = Vec::new();
                        for v in &shape.vertices {
                            vertices.push(transform.apply(v));
                        }
                        debug_draw.borrow_mut().draw_polygon(&vertices);
                    }
                }
            }
        }
    }

    // Find islands, integrate and solve constraints, solve position constraints.
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

            let mut profile = Profile::default();
            island.solve(&mut profile, &step, &self.gravity, self.allow_sleep);
            self.profile.solve_init += profile.solve_init;
            self.profile.solve_velocity += profile.solve_velocity;
            self.profile.solve_position += profile.solve_position;
        }

        let timer = Timer::new();
        // Synchronize fixtures, check for out of range bodies.
        for (_, b) in &self.bodies {
            // If a body was not in an island then it did not move.
            if !b.borrow().is_island {
                continue;
            }

            // Update fixtures (for broad phase).
            b.borrow_mut().synchronize_fixtures();
        }

        // Look for new contacts.
        self.broad_phase.update_pairs(&mut self.contact_manager);
        self.profile.broad_phase = timer.get_milliseconds();
    }

    // Find TOI contacts and solve them.
    pub fn solve_toi(&mut self, step: &TimeStep) {
        let island = Island::new(2 * common::MAX_TOI_CONTACTS, common::MAX_TOI_CONTACTS);

        if self.step_complete {
            for (_, b) in &self.bodies {
                b.borrow_mut().is_island = false;
                b.borrow_mut().sweep.alpha0 = 0.0;
            }

            for c in &self.contact_manager.contacts {
                // Invalidate TOI.
                c.borrow_mut().is_island = false;
                c.borrow_mut().is_toi = false;
                c.borrow_mut().toi_count = 0;
                c.borrow_mut().toi = 1.0;
            }

            // Find TOI events and solve them.
            loop {
                // Find the first TOI.

                for c in &self.contact_manager.contacts {
                    // Is this contact disabled?
                    if !c.borrow().is_enabled {
                        continue;
                    }

                    // Prevent excessive sub-stepping.
                    if c.borrow().toi_count > common::MAX_SUB_STEPS {
                        continue;
                    }

                    let mut alpha = 1.0;
                    if c.borrow().is_toi {
                        // This contact has a valid cached TOI.
                        alpha = c.borrow().toi;
                    } else {
                        let f_a = c.borrow().fixture_a.clone();
                        let f_b = c.borrow().fixture_b.clone();

                        let b_a = f_a.borrow().body.upgrade().unwrap();
                        let b_b = f_b.borrow().body.upgrade().unwrap();

                        let active_a = b_a.borrow().is_awake;
                        let active_b = b_b.borrow().is_awake;

                        // Is at least one body active (awake and dynamic or kinematic)?
                        if !active_a && !active_b {
                            continue;
                        }

                        let collide_a = b_a.borrow().is_bullet;
                        let collide_b = b_b.borrow().is_bullet;

                        // Are these two non-bullet dynamic bodies?
                        if !collide_a && !collide_b {
                            continue;
                        }

                        // Compute the TOI for this contact.
                        // Put the sweeps onto the same time interval.
                        let mut alpha0 = b_a.borrow().sweep.alpha0;
                        let alpha0_a = b_a.borrow().sweep.alpha0;
                        let alpha0_b = b_b.borrow().sweep.alpha0;

                        if alpha0_a < alpha0_b {
                            alpha0 = alpha0_b;
                            b_a.borrow_mut().sweep.advance(alpha0);
                        } else if alpha0_b < alpha0_a {
                            alpha0 = alpha0_a;
                            b_b.borrow_mut().sweep.advance(alpha0);
                        }

                        assert!(alpha0 < 1.0);

                        // Compute the time of impact in interval [0, minTOI]
                        //collision::time_of_impact();

                        // Beta is the fraction of the remaining portion of the .
                    }

                    /*if alpha < min_alpha {
                        // This is the minimum TOI found so far.

                    }*/
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

        // Handle TOI events.
        if self.continuous_physics && step.dt > 0.0 {
            let timer = Timer::new();
            self.solve_toi(&step);
            self.profile.solve_toi = timer.get_milliseconds();
        }

        if step.dt > 0.0 {
            self.inv_dt0 = step.inv_dt;
        }

        self.profile.step = step_timer.get_milliseconds();
    }

    pub fn test(&mut self) {
        self.broad_phase.update_pairs(&mut self.contact_manager);
    }
}
