use std::rc::{Rc, Weak};
use std::cell::RefCell;
use std::mem;
use std::ptr;
use cgmath::*;
use ::collision::{BroadPhase, Shape};
use ::common::{Rotation2d, Transform2d, Sweep};
use super::{WorldHandleWeak, FixtureHandle, Fixture, ContactEdge, Contact};

pub type BodyHandle<'a> = Rc<RefCell<Body<'a>>>;
pub type BodyHandleWeak<'a> = Weak<RefCell<Body<'a>>>;

pub enum BodyType {
    Static,
    Kinematic,
    Dynamic,
}

/*bitflags! {
    flags Flags: u32 {
        const FLAG_ISLAND = 0b00000001,
        const FLAG_AWAKE  = 0b00000010,
        const FLAG_BULLET = 0b00000100,
        const FLAG_ACTIVE = 0b00001000,
    }
}*/

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

    //flags: Flags,
    pub is_island: bool,
    is_awake: bool,
    pub is_bullet: bool,
    is_active: bool,

    pub island_index: usize,

    fixtures: Vec<FixtureHandle<'a>>,
    contact_edges: Vec<ContactEdge<'a>>,

    world: WorldHandleWeak<'a>,
    self_handle: BodyHandleWeak<'a>,
}

impl<'a> Body<'a> {
    pub fn new(id: u32, world: WorldHandleWeak<'a>) -> BodyHandle<'a> {
        let result;
        unsafe {
            result = Rc::new(RefCell::new(Body {
                id: id,
                body_type: BodyType::Static,
                transform: Transform2d::default(),
                sweep: Sweep::default(),
                linear_velocity: Vector2::zero(),
                angular_velocity: 0.0,
                force: Vector2::zero(),
                torque: 0.0,
                mass: 1.0,
                inv_mass: 1.0,
                inertia: 0.0,
                inv_inertia: 0.0,
                linear_damping: 0.0,
                angular_damping: 0.0,
                gravity_scale: 1.0,
                //flags: FLAG_AWAKE,
                is_island: false,
                is_awake: true,
                is_bullet: false,
                is_active: true,
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

    pub fn create_fixture(&mut self, shape: Shape) -> FixtureHandle<'a> {
        let fixture = Rc::new(RefCell::new(Fixture::new(self.self_handle.clone(), shape)));
        self.fixtures.push(fixture.clone());

        let world = self.world.upgrade().unwrap();
        let broad_phase = &mut world.borrow_mut().broad_phase;

        let fixture_handle = fixture.clone();
        fixture.borrow_mut().create_proxy(fixture_handle, broad_phase, &self.transform);
        /*let aabb = fixture.borrow().shape.compute_aabb(&self.transform);
        fixture.borrow_mut().proxy_id = broad_phase.create_proxy(&aabb, fixture.clone());*/
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

    /// Set the sleep state of the body. A sleeping body has very low CPU cost.
    pub fn set_awake(&mut self, is_awake: bool) {
        // TODO: sleep_time
        if !is_awake {
            self.linear_velocity = Vector2::zero();
            self.angular_velocity = 0.0;
            self.force = Vector2::zero();
            self.torque = 0.0;
        }
        self.is_awake = is_awake;
    }

    pub fn is_awake(&self) -> bool {
        self.is_awake
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

        if is_active {
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
            }
        }
    }

    pub fn is_active(&self) -> bool {
        self.is_active
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
