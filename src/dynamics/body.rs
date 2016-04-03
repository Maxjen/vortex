use std::rc::{Rc, Weak};
use std::cell::RefCell;
use cgmath::*;
use ::collision::Shape;
use ::common::{Rotation2d, Transform2d, Sweep};
use super::{WorldHandleWeak, FixtureHandle, ContactEdge};

pub type BodyHandle<'a> = Rc<RefCell<Body<'a>>>;
pub type BodyHandleWeak<'a> = Weak<RefCell<Body<'a>>>;

pub struct Body<'a> {
    pub id: u32,

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

    pub is_island: bool,
    pub is_awake: bool,
    pub is_bullet: bool,

    pub island_index: usize,

    fixtures: Vec<FixtureHandle<'a>>,
    contact_edges: Vec<ContactEdge<'a>>,

    world: WorldHandleWeak<'a>,
}

impl<'a> Body<'a> {
    pub fn new(id: u32, world: WorldHandleWeak<'a>) -> Body<'a> {
        Body {
            id: id,
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
            is_island: false,
            is_awake: true,
            is_bullet: false,
            island_index: 0,
            fixtures: Vec::new(),
            contact_edges: Vec::new(),
            world: world,
        }
    }

    pub fn create_fixture(&mut self, shape: Shape) {
        let world = match self.world.upgrade() {
            Some(world) => world,
            None => return,
        };
        let fixture = world.borrow_mut().create_fixture(self.id, shape);
        let broad_phase = &mut world.borrow_mut().broad_phase;

        let aabb = fixture.borrow().shape.compute_aabb(&self.transform);
        fixture.borrow_mut().proxy_id = broad_phase.create_proxy(&aabb, fixture.clone());
        self.fixtures.push(fixture);
    }

    pub fn delete_fixtures(&mut self) {
        let world = match self.world.upgrade() {
            Some(world) => world,
            None => return,
        };

        for fixture in &self.fixtures {
            let broad_phase = &mut world.borrow_mut().broad_phase;
            broad_phase.destroy_proxy(fixture.borrow().proxy_id);
            world.borrow_mut().delete_fixture(fixture.borrow().id);
        }
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

    pub fn synchronize_transform(&mut self) {
        self.transform.rotation.set_angle(self.sweep.a);
        self.transform.position = self.sweep.c - self.transform.rotation.apply(&self.sweep.local_center);
    }

    pub fn synchronize_fixtures(&mut self) {
        let rotation = Rotation2d::new(self.sweep.a0);
        let position = self.sweep.c0 - rotation.apply(&self.sweep.local_center);
        let transform = Transform2d::new(position, rotation);

        unimplemented!();
    }
}
