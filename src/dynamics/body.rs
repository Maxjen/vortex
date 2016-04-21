use std::rc::{Rc, Weak};
use std::cell::RefCell;
use std::mem;
use std::ptr;
use cgmath::*;
use ::collision::{BroadPhase, Shape};
use ::common;
use ::common::{Rotation2d, Transform2d, Sweep};
use super::{WorldHandleWeak, FixtureHandle, FixtureConfig, Fixture, ContactEdge, Contact};

pub type BodyHandle<'a> = Rc<RefCell<Body<'a>>>;
pub type BodyHandleWeak<'a> = Weak<RefCell<Body<'a>>>;

#[derive(Clone, Copy, Debug)]
/// The body type.
pub enum BodyType {
    /// zero mass, zero velocity, may be manually moved
    Static,

    /// zero mass, non-zero velocity set by user, moved by solver
    Kinematic,

    /// positive mass, non-zero velocity determined by forces, moved by solver
    Dynamic,
}

/// A `BodyConfig` holds all the data needed to construct a rigid body.
/// You can safely re-use `BodyConfig`s. Shapes are added to a body after construction.
pub struct BodyConfig {
    /// The body type: `Static`, `Kinematic`, or `Dynamic`.
    /// Note: If a dynamic body would have zero mass, the mass is set to one.
    pub body_type: BodyType,

    /// The world position of the body. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    pub position: Vector2<f32>,

    /// The world angle of the body in radians.
    pub angle: f32,

    /// The linear velocity of the body's origin in world coordinates.
    pub linear_velocity: Vector2<f32>,

    /// The angular velocity of the body.
    pub angular_velocity: f32,

    /// Linear damping is used to reduce the linear velocity. The damping parameter
    /// can be larger than 1.0 but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    pub linear_damping: f32,

    /// Angular damping is used to reduce the angular velocity. The damping parameter
    /// can be larger than 1.0 but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    pub angular_damping: f32,

    /// Scales the gravity applied to this body.
    pub gravity_scale: f32,

    /// Is this body initially awake or sleeping?
    pub awake: bool,

    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    pub auto_sleep: bool,

    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling through
    /// kinematic and static bodies. This setting is only considered on dynamic bodies.
    /// You should use this flag sparingly since it increases processing time.
    pub bullet: bool,

    /// Should this body be prevented from rotating? Useful for characters.
    pub fixed_rotation: bool,

    /// Does this body start out active?
    pub active: bool,
}

impl Default for BodyConfig {
    fn default() -> BodyConfig {
        BodyConfig {
            body_type: BodyType::Static,
            position: Vector2::zero(),
            angle: 0.0,
            linear_velocity: Vector2::zero(),
            angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            awake: true,
            auto_sleep: true,
            bullet: false,
            fixed_rotation: false,
            active: true,
        }
    }
}

bitflags! {
    flags Flags: u32 {
        const FLAG_ISLAND         = 0b00000001,
        const FLAG_AWAKE          = 0b00000010,
        const FLAG_AUTO_SLEEP     = 0b00000100,
        const FLAG_BULLET         = 0b00001000,
        const FLAG_FIXED_ROTATION = 0b00010000,
        const FLAG_ACTIVE         = 0b00100000,
    }
}

/// A rigid body. These are created via `World::create_body`.
pub struct Body<'a> {
    pub id: u32,

    pub body_type: BodyType,

    /// The body origin transform
    transform: Transform2d,
    /// The swept motion for CCD
    pub sweep: Sweep,

    linear_velocity: Vector2<f32>,
    angular_velocity: f32,

    pub force: Vector2<f32>,
    pub torque: f32,

    mass: f32,
    pub inv_mass: f32,
    inertia: f32,
    pub inv_inertia: f32,

    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,

    flags: Flags,

    pub island_index: usize,

    fixtures: Vec<FixtureHandle<'a>>,
    contact_edges: Vec<ContactEdge<'a>>,

    world: WorldHandleWeak<'a>,
    self_handle: BodyHandleWeak<'a>,
}

impl<'a> Body<'a> {
    pub fn new(id: u32, world: WorldHandleWeak<'a>, body_config: &BodyConfig) -> BodyHandle<'a> {
        let result;

        let transform = Transform2d::new(body_config.position, Rotation2d::new(body_config.angle));

        let mut sweep = Sweep::default();
        sweep.local_center = Vector2::zero();
        sweep.c0 = transform.position;
        sweep.c = transform.position;
        sweep.a0 = body_config.angle;
        sweep.a = body_config.angle;
        sweep.alpha0 = 0.0;

        let mass;
        let inv_mass;
        if let BodyType::Dynamic = body_config.body_type {
            mass = 1.0;
            inv_mass = 1.0;
        } else {
            mass = 0.0;
            inv_mass = 0.0;
        }

        let mut flags = Flags::empty();
        if body_config.awake {
            flags.insert(FLAG_AWAKE);
        }
        if body_config.auto_sleep {
            flags.insert(FLAG_AUTO_SLEEP);
        }
        if body_config.bullet {
            flags.insert(FLAG_BULLET);
        }
        if body_config.fixed_rotation {
            flags.insert(FLAG_FIXED_ROTATION);
        }
        if body_config.active {
            flags.insert(FLAG_ACTIVE);
        }

        unsafe {
            result = Rc::new(RefCell::new(Body {
                id: id,
                body_type: body_config.body_type,
                transform: transform,
                sweep: sweep,
                linear_velocity: body_config.linear_velocity,
                angular_velocity: body_config.angular_velocity,
                force: Vector2::zero(),
                torque: 0.0,
                mass: mass,
                inv_mass: inv_mass,
                inertia: 0.0,
                inv_inertia: 0.0,
                linear_damping: body_config.linear_damping,
                angular_damping: body_config.angular_damping,
                gravity_scale: body_config.gravity_scale,
                flags: flags,
                island_index: 0,
                fixtures: Vec::new(),
                contact_edges: Vec::new(),
                world: world,
                self_handle: mem::uninitialized(),
            }));
            let self_handle = Rc::downgrade(&result);
            ptr::write(&mut result.borrow_mut().self_handle, self_handle);
        }
        result
    }

    pub fn create_fixture(&mut self, shape: Shape, fixture_config: &FixtureConfig) -> FixtureHandle<'a> {
        let fixture = Rc::new(RefCell::new(Fixture::new(self.self_handle.clone(), shape, fixture_config)));
        self.fixtures.push(fixture.clone());

        let world = self.world.upgrade().unwrap();
        {
            let broad_phase = &mut world.borrow_mut().broad_phase;

            let fixture_handle = fixture.clone();
            fixture.borrow_mut().create_proxy(fixture_handle, broad_phase, &self.transform);
        }

        /*let aabb = fixture.borrow().shape.compute_aabb(&self.transform);
        fixture.borrow_mut().proxy_id = broad_phase.create_proxy(&aabb, fixture.clone());*/

        // Adjust mass properties if needed.
        if fixture.borrow().density > 0.0 {
            self.reset_mass_data();
        }

        // Let the world know we have a new fixture. This will cause new contacts
        // to be created at the beginning of the next time step.
        world.borrow_mut().set_new_fixtures(true);

        fixture
    }

    pub fn delete_fixtures(&mut self) {
        let world = match self.world.upgrade() {
            Some(world) => world,
            None => return,
        };

        for fixture in &self.fixtures {
            let f = fixture.clone();
            let pf = &(*f) as *const RefCell<Fixture>;

            // Destroy any contact associated with the fixture.
            for ce in &self.contact_edges {
                let contact = ce.contact.upgrade().unwrap();
                let f_a = contact.borrow().fixture_a.clone();
                let f_b = contact.borrow().fixture_b.clone();
                let pf_a = &(*f_a) as *const RefCell<Fixture>;
                let pf_b = &(*f_b) as *const RefCell<Fixture>;

                if pf == pf_a || pf == pf_b {
                    let contact_manager = &mut world.borrow_mut().contact_manager;
                    contact_manager.delete_contact(contact);
                }
            }

            let broad_phase = &mut world.borrow_mut().broad_phase;
            //broad_phase.destroy_proxy(fixture.borrow().proxy_id);
            f.borrow_mut().destroy_proxy(broad_phase);
        }

        self.fixtures.clear();
    }

    pub fn get_fixtures(&self) -> &Vec<FixtureHandle<'a>> {
        &self.fixtures
    }

    pub fn get_contact_edges(&self) -> &Vec<ContactEdge<'a>> {
        &self.contact_edges
    }

    pub fn add_contact_edge(&mut self, contact_edge: ContactEdge<'a>) {
        self.contact_edges.push(contact_edge);
    }

    pub fn remove_contact_edge(&mut self, contact_edge: ContactEdge<'a>) {
        let b1 = contact_edge.body.upgrade().unwrap();
        let c1 = contact_edge.contact.upgrade().unwrap();
        let pb1 = &(*b1) as *const RefCell<Body>;
        let pc1 = &(*c1) as *const RefCell<Contact>;

        let mut remove = None;
        for (i, ce) in self.contact_edges.iter().enumerate() {
            let b2 = ce.body.upgrade().unwrap();
            let c2 = ce.contact.upgrade().unwrap();
            let pb2 = &(*b2) as *const RefCell<Body>;
            let pc2 = &(*c2) as *const RefCell<Contact>;

            if pb1 == pb2 && pc1 == pc2 {
                remove = Some(i);
            }
        }

        if let Some(remove) = remove {
            self.contact_edges.swap_remove(remove);
        }
    }

    pub fn set_transform(&mut self, position: &Vector2<f32>, angle: f32) {
        self.transform.position = *position;
        self.transform.rotation.set_angle(angle);

        self.sweep.c = self.transform.apply(&self.sweep.local_center);
        self.sweep.a = angle;

        self.sweep.c0 = self.sweep.c;
        self.sweep.a0 = angle;

        let world = self.world.upgrade().unwrap();
        let broad_phase = &mut world.borrow_mut().broad_phase;
        for fixture in &self.fixtures {
            fixture.borrow_mut().synchronize(broad_phase, &self.transform, &self.transform);
        }
    }

    pub fn get_transform(&self) -> Transform2d {
        self.transform
    }

    pub fn get_position(&self) -> Vector2<f32> {
        self.transform.position
    }

    pub fn set_linear_velocity(&mut self, linear_velocity: Vector2<f32>) {
        if let BodyType::Static = self.body_type {
            return;
        }

        if dot(linear_velocity, linear_velocity) > 0.0 {
            self.set_awake(true);
        }

        self.linear_velocity = linear_velocity;
    }

    pub fn get_linear_velocity(&self) -> Vector2<f32> {
        self.linear_velocity
    }

    pub fn set_angular_velocity(&mut self, angular_velocity: f32) {
        if let BodyType::Static = self.body_type {
            return;
        }

        if angular_velocity * angular_velocity > 0.0 {
            self.set_awake(true);
        }

        self.angular_velocity = angular_velocity;
    }

    pub fn get_angular_velocity(&self) -> f32 {
        self.angular_velocity
    }

    pub fn get_mass(&self) -> f32 {
        self.mass
    }

    /// Get the rotational inertia of the body about the local origin.
    pub fn get_inertia(&self) -> f32 {
        self.inertia + self.mass * dot(self.sweep.local_center, self.sweep.local_center)
    }

    /// This resets the mass properties to the sum of the mass properties of the fixtures.
    /// This normally does not need to be called unless you called `set_mass_data` to override
    /// the mass and you later want to reset the mass.
    pub fn reset_mass_data(&mut self) {
        // Compute mass data from shapes. Each shape has its own density.
        self.mass = 0.0;
        self.inv_mass = 0.0;
        self.inertia = 0.0;
        self.inv_inertia = 0.0;
        self.sweep.local_center = Vector2::zero();

        // Static and kinematic bodies have zero mass.
        let is_static = if let BodyType::Static = self.body_type { true } else { false };
        let is_kinematic = if let BodyType::Kinematic = self.body_type { true } else { false };
        if is_static || is_kinematic {
            self.sweep.c0 = self.transform.position;
            self.sweep.c = self.transform.position;
            self.sweep.a0 = self.sweep.a;
            return;
        }

        // Accumulate mass over all fixtures.
        let mut local_center = Vector2::<f32>::zero();
        for f in &self.fixtures {
            if f.borrow().density == 0.0 {
                continue;
            }

            let (mass, center, inertia) = f.borrow().get_mass_data();
            self.mass += mass;
            local_center = local_center + center * mass;
            self.inertia += inertia;
        }

        // Compute center of mass.
        if self.mass > 0.0 {
            self.inv_mass = 1.0 / self.mass;
            local_center = local_center * self.inv_mass;
        } else {
            // Force all dynamic bodies to have a positive mass.
            self.mass = 1.0;
            self.inv_mass = 1.0;
        }

        if self.inertia > 0.0 && !self.flags.contains(FLAG_FIXED_ROTATION) {
            // Center the inertia about the center of mass.
            self.inertia -= self.mass * dot(local_center, local_center);
            assert!(self.inertia > 0.0);
            self.inv_inertia = 1.0 / self.inertia;
        } else {
            self.inertia = 0.0;
            self.inv_inertia = 0.0;
        }

        // Move center of mass.
        let old_center = self.sweep.c;
        self.sweep.local_center = local_center;
        self.sweep.c0 = self.transform.apply(&self.sweep.local_center);
        self.sweep.c = self.sweep.c0;

        // Update center of mass velocity.
        self.linear_velocity = self.linear_velocity + common::cross_s_v(self.angular_velocity, &(self.sweep.c - old_center));
    }

    pub fn apply_force_to_center(&mut self, force: &Vector2<f32>, wake: bool) {
        let dynamic = if let BodyType::Dynamic = self.body_type { true } else { false };
        if !dynamic {
            return;
        }

        if wake && !self.is_awake() {
            self.set_awake(true);
        }

        // Don't accumulate a force if the body is sleeping.
        if self.is_awake() {
            self.force = self.force + force;
        }
    }

    pub fn apply_torque(&mut self, torque: f32, wake: bool) {
        let dynamic = if let BodyType::Dynamic = self.body_type { true } else { false };
        if !dynamic {
            return;
        }

        if wake && !self.is_awake() {
            self.set_awake(true);
        }

        // Don't accumulate a torque if the body is sleeping.
        if self.is_awake() {
            self.torque += torque;
        }
    }

    pub fn synchronize_transform(&mut self) {
        self.transform.rotation.set_angle(self.sweep.a);
        self.transform.position = self.sweep.c - self.transform.rotation.apply(&self.sweep.local_center);
    }

    pub fn synchronize_fixtures(&mut self, broad_phase: &mut BroadPhase<'a>) {
        let rotation = Rotation2d::new(self.sweep.a0);
        let position = self.sweep.c0 - rotation.apply(&self.sweep.local_center);
        let transform = Transform2d::new(position, rotation);

        for f in &self.fixtures {
            f.borrow().synchronize(broad_phase, &transform, &self.transform);
        }
    }

    pub fn set_island(&mut self, is_island: bool) {
        if is_island {
            self.flags.insert(FLAG_ISLAND);
        } else {
            self.flags.remove(FLAG_ISLAND);
        }
    }

    pub fn is_island(&self) -> bool {
        self.flags.contains(FLAG_ISLAND)
    }

    /// Set the sleep state of the body. A sleeping body has very low CPU cost.
    pub fn set_awake(&mut self, is_awake: bool) {
        // TODO: sleep_time
        if is_awake {
            self.flags.insert(FLAG_AWAKE);
        } else {
            self.flags.remove(FLAG_AWAKE);
            self.linear_velocity = Vector2::zero();
            self.angular_velocity = 0.0;
            self.force = Vector2::zero();
            self.torque = 0.0;
        }
    }

    pub fn is_awake(&self) -> bool {
        self.flags.contains(FLAG_AWAKE)
    }

    pub fn set_bullet(&mut self, is_bullet: bool) {
        if is_bullet {
            self.flags.insert(FLAG_BULLET);
        } else {
            self.flags.remove(FLAG_BULLET);
        }
    }

    pub fn is_bullet(&self) -> bool {
        self.flags.contains(FLAG_BULLET)
    }

    pub fn set_fixed_rotation(&mut self, fixed_rotation: bool) {
        if fixed_rotation == self.flags.contains(FLAG_FIXED_ROTATION) {
            return;
        }

        if fixed_rotation {
            self.flags.insert(FLAG_FIXED_ROTATION);
        } else {
            self.flags.remove(FLAG_FIXED_ROTATION);
        }

        self.angular_velocity = 0.0;

        self.reset_mass_data();
    }

    pub fn is_fixed_rotation(&self) -> bool {
        self.flags.contains(FLAG_FIXED_ROTATION)
    }

    /// Set the active state of the body. An inactive body is not simulated and cannot be
    /// collided with or woken up.
    /// If you pass true, all fixtures will be added to the broad phase.
    /// If you pass false, all fixtures will be removed from the broad phase and all contacts
    /// will be destroyed.
    /// Fixtures and joints are otherwise unaffected. You may continue to create/destroy
    /// fixtures and joints on inactive bodies. Fixtures on an inactive body are implicitly
    /// inactive and will not participate in collisions, ray-casts, or queries.
    /// Joints connected to an inactive body are implicitly inactive.
    /// An inactive body is still owned by a `World` object and remains in the body list.
    pub fn set_active(&mut self, is_active: bool) {
        let world = self.world.upgrade().unwrap();

        if is_active == self.is_active() {
            return;
        }

        if is_active {
            self.flags.insert(FLAG_ACTIVE);

            // Create all proxies.
            let broad_phase = &mut world.borrow_mut().broad_phase;
            for fixture in &self.fixtures {
                /*let aabb = fixture.borrow().shape.compute_aabb(&self.transform);
                fixture.borrow_mut().proxy_id = broad_phase.create_proxy(&aabb, fixture.clone());*/
                let fixture_handle = fixture.clone();
                fixture.borrow_mut().create_proxy(fixture_handle, broad_phase, &self.transform);

                // Contacts are created the next time step.
            }
        } else {
            self.flags.remove(FLAG_ACTIVE);

            {
                // Destroy all proxies.
                let broad_phase = &mut world.borrow_mut().broad_phase;
                for fixture in &self.fixtures {
                    fixture.borrow_mut().destroy_proxy(broad_phase);
                }
            }

            {
                // Destroy the attacted contacts.
                let contact_manager = &mut world.borrow_mut().contact_manager;
                for ce in &self.contact_edges {
                    let contact = ce.contact.upgrade().unwrap();
                    contact_manager.delete_contact(contact);
                }
                self.contact_edges.clear();
            }
        }
    }

    pub fn is_active(&self) -> bool {
        self.flags.contains(FLAG_ACTIVE)
    }

    pub fn advance(&mut self, alpha: f32) {
        // Advance to the new safe time. This doesn't sync the broad phase.
        self.sweep.advance(alpha);
        self.sweep.c = self.sweep.c0;
        self.sweep.a = self.sweep.a0;
        self.transform.rotation.set_angle(self.sweep.a);
        self.transform.position = self.sweep.c - self.transform.rotation.apply(&self.sweep.local_center);
    }
}
