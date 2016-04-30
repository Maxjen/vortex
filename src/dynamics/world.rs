use std::collections::HashMap;
use std::rc::{Rc, Weak};
use std::cell::RefCell;
use std::mem;
use std::ptr;
use std::f32;
use cgmath::*;
use super::{BodyType, BodyConfig, Body, BodyHandle, Fixture, FixtureHandle, JointConfig, Joint, JointHandle, ContactManager, Island};
use ::collision;
use ::collision::{BroadPhase, Shape, DistanceProxy, ToiState};
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

bitflags! {
    flags Flags: u32 {
        const FLAG_NEW_FIXTURES = 0b00000001,
        const FLAG_LOCKED       = 0b00000010, // TODO: use somewhere
        const FLAG_CLEAR_FORCES = 0b00000100,
    }
}

pub struct World<'a> {
    pub broad_phase: BroadPhase<'a>,
    pub contact_manager: ContactManager<'a>,

    flags: Flags,

    gravity: Vector2<f32>,
    allow_sleep: bool,

    bodies: HashMap<u32, BodyHandle<'a>>,
    body_index_pool: IndexPool,

    joints: HashMap<u32, JointHandle<'a>>,
    joint_index_pool: IndexPool,

    debug_draw: Option<Rc<RefCell<DebugDraw + 'a>>>,
    self_handle: WorldHandleWeak<'a>,

    // This is used to compute the time step ratio to support a variable time step.
    inv_dt0: f32,

    // These are used for debugging the solver.
    warm_starting: bool,
    continuous_physics: bool,
    sub_stepping: bool,

    step_complete: bool,

    profile: Profile,
}

impl<'a> World<'a> {
    pub fn new(gravity: Vector2<f32>) -> WorldHandle<'a> {
        let result;
        unsafe {
            result = Rc::new(RefCell::new(World {
                broad_phase: BroadPhase::new(),
                contact_manager: ContactManager::new(),
                flags: FLAG_CLEAR_FORCES,
                gravity: gravity,
                allow_sleep: true,
                bodies: HashMap::new(),
                body_index_pool: IndexPool::new(),
                joints: HashMap::new(),
                joint_index_pool: IndexPool::new(),
                debug_draw: None,
                self_handle: mem::uninitialized(),
                inv_dt0: 0.0,
                warm_starting: true,
                sub_stepping: false,
                continuous_physics: false,
                step_complete: true,
                profile: Default::default(),
            }));
            let self_handle = Rc::downgrade(&result);
            ptr::write(&mut result.borrow_mut().self_handle, self_handle);
            //result.borrow_mut().self_handle = Some(self_handle);
        }
        result
    }

    pub fn create_body(&mut self, body_config: &BodyConfig) -> BodyHandle<'a> {
        let index = self.body_index_pool.get_index();
        let world = self.self_handle.clone();
        //let result = Rc::new(RefCell::new(Body::new(index, world)));
        let result = Body::new(index, world, body_config);
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

    pub fn create_joint(&mut self, joint_config: &JointConfig<'a>) -> JointHandle<'a> {
        let index = self.joint_index_pool.get_index();
        let world = self.self_handle.clone();
        let result = Joint::new(joint_config);
        self.joints.insert(index, result.clone());

        let joint_weak = Rc::downgrade(&result);
        let body_a = joint_config.body_a.upgrade().unwrap();
        let body_b = joint_config.body_b.upgrade().unwrap();
        body_a.borrow_mut().add_joint(joint_weak.clone());
        body_b.borrow_mut().add_joint(joint_weak);

        result
    }

    /*pub fn delete_joint(&mut self) {

    }*/

    pub fn get_body(&self, id: u32) -> Option<BodyHandle<'a>> {
        if let Some(body) = self.bodies.get(&id) {
            return Some(body.clone());
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

                        /*let aabb = self.broad_phase.get_fat_aabb(f.proxy_id.unwrap());
                        let v1 = aabb.min;
                        let v2 = vec2(aabb.max.x, aabb.min.y);
                        let v3 = aabb.max;
                        let v4 = vec2(aabb.min.x, aabb.max.y);
                        debug_draw.borrow_mut().draw_segment(&v1, &v2);
                        debug_draw.borrow_mut().draw_segment(&v2, &v3);
                        debug_draw.borrow_mut().draw_segment(&v3, &v4);
                        debug_draw.borrow_mut().draw_segment(&v4, &v1);*/
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
        let mut island = Island::new(self.bodies.len(), self.contact_manager.get_contact_count(), self.joints.len());

        // Clear all the island flags.
        for (_, b) in &self.bodies {
            b.borrow_mut().set_island(false);
        }
        for c in &self.contact_manager.contacts {
            c.borrow_mut().is_island = false;
        }
        for (_, j) in &self.joints {
            j.borrow_mut().set_island(false);
        }

        // Build and simulate all awake islands.
        let mut stack: Vec<BodyHandle> = Vec::with_capacity(self.bodies.len());
        for (_, seed) in &self.bodies {
            if seed.borrow().is_island() {
                continue;
            }

            if !seed.borrow().is_awake() || !seed.borrow().is_active() {
                continue;
            }

            // The seed can be dynamic or kinematic.
            if let BodyType::Static = seed.borrow().body_type {
                continue;
            }

            // Reset island and stack.
            island.clear();
            stack.clear();
            stack.push(seed.clone());
            seed.borrow_mut().set_island(true);

            // Perform a depth first search (DFS) on the constraint graph.
            while stack.len() > 0 {
                // Grab the next body off the stack and add it to the island.
                let b = stack.pop().unwrap();
                island.add_body(Rc::downgrade(&b));

                // Make sure the body is awake.
                b.borrow_mut().set_awake(true);

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if let BodyType::Static = b.borrow().body_type {
                    continue;
                }

                // Search all contacts connected to this body.
                for ce in b.borrow().get_contact_edges() {
                    let contact = ce.contact.upgrade().unwrap();

                    // Has this contact already been added to an island?
                    if contact.borrow().is_island {
                        continue;
                    }

                    // Is this contact solid and touching?
                    if !contact.borrow().is_enabled || !contact.borrow().is_touching() {
                        continue
                    }

                    // Skip sensors.
                    {
                        let contact = contact.borrow();
                        let sensor_a = contact.fixture_a.borrow().is_sensor;
                        let sensor_b = contact.fixture_b.borrow().is_sensor;
                        if sensor_a || sensor_b {
                            continue;
                        }
                    }

                    island.add_contact(Rc::downgrade(&contact));
                    contact.borrow_mut().is_island = true;

                    let other = ce.body.upgrade().unwrap();

                    // Was the other body already added to this island?
                    if other.borrow().is_island() {
                        continue;
                    }

                    other.borrow_mut().set_island(true);
                    stack.push(other);
                }

                // Search all joints connected to this body.
                for j in b.borrow().get_joints() {
                    let joint = j.upgrade().unwrap();

                    if joint.borrow().is_island() {
                        continue;
                    }

                    let other = joint.borrow().get_other_body(Rc::downgrade(&b)).unwrap().upgrade().unwrap();

                    // Don't simulate joints connected to inactive bodies.
                    if !other.borrow().is_active() {
                        continue;
                    }

                    island.add_joint(j.clone());
                    joint.borrow_mut().set_island(true);

                    if other.borrow().is_island() {
                        continue;
                    }

                    other.borrow_mut().set_island(true);
                    stack.push(other);
                }
            }

            let mut profile = Profile::default();
            island.solve(&mut profile, &step, &self.gravity, self.allow_sleep);
            self.profile.solve_init += profile.solve_init;
            self.profile.solve_velocity += profile.solve_velocity;
            self.profile.solve_position += profile.solve_position;

            // Post solve cleanup.
            for (_, b) in &self.bodies {
                // Allow static bodies to participate in other islands.
                let is_static = if let BodyType::Static = b.borrow().body_type {
                    true
                } else {
                    false
                };
                if is_static {
                    b.borrow_mut().set_island(false);
                }
            }
        }

        let timer = Timer::new();
        // Synchronize fixtures, check for out of range bodies.
        for (_, b) in &self.bodies {
            // If a body was not in an island then it did not move.
            if !b.borrow().is_island() {
                continue;
            }

            if let BodyType::Static = b.borrow().body_type {
                continue;
            }

            // Update fixtures (for broad phase).
            b.borrow_mut().synchronize_fixtures(&mut self.broad_phase);
        }

        // Look for new contacts.
        self.broad_phase.update_pairs(&mut self.contact_manager);
        self.profile.broad_phase = timer.get_milliseconds();
    }

    // Find TOI contacts and solve them.
    pub fn solve_toi(&mut self, step: &TimeStep) {
        let mut island = Island::new(2 * common::MAX_TOI_CONTACTS, common::MAX_TOI_CONTACTS, 0);

        if self.step_complete {
            for (_, b) in &self.bodies {
                b.borrow_mut().set_island(false);
                b.borrow_mut().sweep.alpha0 = 0.0;
            }

            for c in &self.contact_manager.contacts {
                // Invalidate TOI.
                c.borrow_mut().is_island = false;
                c.borrow_mut().is_toi = false;
                c.borrow_mut().toi_count = 0;
                c.borrow_mut().toi = 1.0;
            }
        }

        // Find TOI events and solve them.
        loop {
            // Find the first TOI.
            let mut min_contact = None;
            let mut min_alpha = 1.0;

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
                let is_toi = c.borrow().is_toi;
                if is_toi {
                    // This contact has a valid cached TOI.
                    alpha = c.borrow().toi;
                } else {
                    //let f_a = c.borrow().fixture_a.clone();
                    //let f_b = c.borrow().fixture_b.clone();
                    let f_a = c.borrow().fixture_a.clone();
                    let f_b = c.borrow().fixture_b.clone();

                    // Is there a sensor?
                    if f_a.borrow().is_sensor || f_b.borrow().is_sensor {
                        continue;
                    }

                    let b_a = f_a.borrow().body.upgrade().unwrap();
                    let b_b = f_b.borrow().body.upgrade().unwrap();

                    let active_a = b_a.borrow().is_awake();
                    let active_b = b_b.borrow().is_awake();

                    // Is at least one body active (awake and dynamic or kinematic)?
                    if !active_a && !active_b {
                        continue;
                    }

                    let dynamic_a = if let BodyType::Dynamic = b_a.borrow().body_type {
                        true
                    } else {
                        false
                    };
                    let dynamic_b = if let BodyType::Dynamic = b_b.borrow().body_type {
                        true
                    } else {
                        false
                    };
                    let collide_a = b_a.borrow().is_bullet() || !dynamic_a;
                    let collide_b = b_b.borrow().is_bullet() || !dynamic_b;

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
                    let proxy_a = DistanceProxy::new(&f_a.borrow().shape);
                    let proxy_b = DistanceProxy::new(&f_b.borrow().shape);
                    let (state, t) = collision::time_of_impact(&proxy_a, &proxy_b, b_a.borrow().sweep, b_b.borrow().sweep, 1.0);

                    // Beta is the fraction of the remaining portion of the .
                    let beta = t;
                    if let ToiState::Touching = state {
                        alpha = f32::min(alpha0 + (1.0 - alpha0) * beta, 1.0);
                    } else {
                        alpha = 1.0;
                    }

                    c.borrow_mut().toi = alpha;
                    c.borrow_mut().is_toi = true;
                }

                if alpha < min_alpha {
                    // This is the minimum TOI found so far.
                    min_contact = Some(c.clone());
                    min_alpha = alpha;
                }
            }

            let no_min_contact = if let None = min_contact { true } else { false };
            if no_min_contact || 1.0 - 10.0 * f32::EPSILON < min_alpha {
                // No more TOI events. Done!
                self.step_complete = true;
                break;
            }
            let min_contact = min_contact.unwrap();

            // Advance the bodies to the TOI.
            let f_a = min_contact.borrow().fixture_a.clone();
            let f_b = min_contact.borrow().fixture_b.clone();
            let b_a = f_a.borrow().body.upgrade().unwrap();
            let b_b = f_b.borrow().body.upgrade().unwrap();

            let backup1 = b_a.borrow().sweep;
            let backup2 = b_b.borrow().sweep;

            b_a.borrow_mut().advance(min_alpha);
            b_b.borrow_mut().advance(min_alpha);

            // The TOI contact likely has some new contact points.
            min_contact.borrow_mut().update();
            min_contact.borrow_mut().is_toi = false;
            min_contact.borrow_mut().toi_count += 1;

            // Is the contact solid?
            if !min_contact.borrow().is_enabled || !min_contact.borrow().is_touching {
                // Restore the sweeps.
                min_contact.borrow_mut().is_enabled = false;
                b_a.borrow_mut().sweep = backup1;
                b_b.borrow_mut().sweep = backup2;
                b_a.borrow_mut().synchronize_transform();
                b_b.borrow_mut().synchronize_transform();
                continue;
            }

            b_a.borrow_mut().set_awake(true);
            b_b.borrow_mut().set_awake(false);

            // Build the island.
            island.clear();
            island.add_body(Rc::downgrade(&b_a));
            island.add_body(Rc::downgrade(&b_b));
            island.add_contact(Rc::downgrade(&min_contact));

            b_a.borrow_mut().set_island(true);
            b_b.borrow_mut().set_island(true);
            min_contact.borrow_mut().is_island = true;

            // Get contacts on body A and body B.
            let bodies = vec![&b_a, &b_b];
            for b in &bodies {
                if let BodyType::Dynamic = b.borrow().body_type {
                    for ce in b.borrow().get_contact_edges() {
                        if island.bodies.len() == island.bodies.capacity() {
                            break;
                        }

                        if island.contacts.len() == island.contacts.capacity() {
                            break;
                        }

                        let contact = ce.contact.upgrade().unwrap();

                        // Has this contact already been added to the island?
                        if contact.borrow().is_island {
                            continue;
                        }

                        // Only add static, kinematic, or bullet bodies.
                        let other = ce.body.upgrade().unwrap();
                        let other_dynamic = if let BodyType::Dynamic = other.borrow().body_type {
                            true
                        } else {
                            false
                        };
                        if other_dynamic && !b.borrow().is_bullet() && !other.borrow().is_bullet() {
                            continue;
                        }

                        // Skip sensors.
                        {
                            let contact = contact.borrow();
                            let sensor_a = contact.fixture_a.borrow().is_sensor;
                            let sensor_b = contact.fixture_b.borrow().is_sensor;
                            if sensor_a || sensor_b {
                                continue;
                            }
                        }

                        // Tentatively advance the body to the TOI.
                        let backup = other.borrow().sweep;
                        if !other.borrow().is_island() {
                            other.borrow_mut().advance(min_alpha);
                        }

                        // Update contact points.
                        contact.borrow_mut().update();

                        // Was the contact disabled by the user?
                        if !contact.borrow().is_enabled {
                            other.borrow_mut().sweep = backup;
                            other.borrow_mut().synchronize_transform();
                            continue;
                        }

                        // Are there contact points?
                        if !contact.borrow().is_touching {
                            other.borrow_mut().sweep = backup;
                            other.borrow_mut().synchronize_transform();
                            continue;
                        }

                        // Add the contact to the island.
                        contact.borrow_mut().is_island = true;
                        island.add_contact(Rc::downgrade(&contact));

                        // Has the other body already been added to the island?
                        if other.borrow().is_island() {
                            continue;
                        }

                        // Add the other body to the island.
                        other.borrow_mut().set_island(true);

                        // TODO: dynamic check
                        other.borrow_mut().set_awake(true);

                        let other_static = if let BodyType::Static = other.borrow().body_type {
                            true
                        } else {
                            false
                        };
                        if !other_static {
                            other.borrow_mut().set_awake(true);
                        }

                        island.add_body(Rc::downgrade(&other));
                    }
                }
            }

            let dt = (1.0 - min_alpha) * step.dt;
            let sub_step = TimeStep {
                dt: dt,
                inv_dt: 1.0 / dt,
                dt_ratio: 1.0,
                velocity_iterations: step.velocity_iterations,
                position_iterations: 20,
                warm_starting: false,
            };
            let island_index_a = b_a.borrow().island_index;
            let island_index_b = b_b.borrow().island_index;
            island.solve_toi(&sub_step, island_index_a, island_index_b);

            // Reset island flags and synchronize broad phase proxies.
            for body in &island.bodies {
                let body = body.upgrade().unwrap();
                body.borrow_mut().set_island(false);

                let dynamic = if let BodyType::Dynamic = body.borrow().body_type {
                    true
                } else {
                    false
                };
                if !dynamic {
                    continue;
                }

                body.borrow_mut().synchronize_fixtures(&mut self.broad_phase);

                // Invalidate all contact TOIs on this displaced body.
                for ce in body.borrow().get_contact_edges() {
                    let contact = ce.contact.upgrade().unwrap();
                    contact.borrow_mut().is_toi = false;
                    contact.borrow_mut().is_island = false;
                }
            }

            // Commit fixture proxy movements to the broad phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            self.broad_phase.update_pairs(&mut self.contact_manager);

            if self.sub_stepping {
                self.step_complete = false;
                break;
            }
        }
    }

    /// Take a time step. This performs collision detection, integration, and constraint solution.
    pub fn step(&mut self, dt: f32, velocity_iterations: u32, position_iterations: u32) {
        let step_timer = Timer::new();

        // If new fixtures were added, we need to find the new contacts.
        if self.flags.contains(FLAG_NEW_FIXTURES) {
            self.broad_phase.update_pairs(&mut self.contact_manager);
            self.flags.remove(FLAG_NEW_FIXTURES);
        }

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
            self.contact_manager.collide(&self.broad_phase);
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

        if self.flags.contains(FLAG_CLEAR_FORCES) {
            self.clear_forces();
        }

        self.profile.step = step_timer.get_milliseconds();
    }

    pub fn set_new_fixtures(&mut self, new_fixtures: bool) {
        if new_fixtures {
            self.flags.insert(FLAG_NEW_FIXTURES);
        } else {
            self.flags.remove(FLAG_NEW_FIXTURES);
        }
    }

    /// Set the flag to control automatic clearing of forces after each time step.
    pub fn set_auto_clear_forces(&mut self, auto_clear_forces: bool) {
        if auto_clear_forces {
            self.flags.insert(FLAG_CLEAR_FORCES);
        } else {
            self.flags.remove(FLAG_CLEAR_FORCES);
        }
    }

    pub fn get_auto_clear_forces(&self) -> bool {
        self.flags.contains(FLAG_CLEAR_FORCES)
    }

    /// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
    /// after each call to `step`. The default is modified by calling `set_auto_clear_forces`.
    /// The purpose of this function is to support sub-stepping. Sub-stepping is often used to
    /// maintain a fixed sized time step under a variable frame-rate.
    /// When you perform sub-stepping you will disable auto clearing of forces and instead call
    /// clear_forces after all sub-steps are complete in one pass of your game loop.
    pub fn clear_forces(&self) {
        for (_, body) in &self.bodies {
            body.borrow_mut().force = Vector2::zero();
            body.borrow_mut().torque = 0.0;
        }
    }
}
