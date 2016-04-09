use std::rc::{Rc, Weak};
use std::cell::RefCell;
use std::mem;
use std::ptr;
use super::{FixtureHandle, World, BodyHandle, BodyHandleWeak};
use ::collision::{Manifold, Shape};
use ::collision;

pub type ContactHandle<'a> = Rc<RefCell<Contact<'a>>>;
pub type ContactHandleWeak<'a> = Weak<RefCell<Contact<'a>>>;

macro_rules! try_some {
    ($e:expr) => (match $e { Some(s) => s, None => return })
}

/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
/// For example, anything slides on ice.
fn mix_friction(friction1: f32, friction2: f32) -> f32 {
    (friction1 * friction2).sqrt()
}

/// Restitution mixing law. The idea is to allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
fn mix_restitution(restitution1: f32, restitution2: f32) -> f32 {
    f32::max(restitution1, restitution2)
}

/// A contact edge is used to connect bodies and contacts together in a contact graph where
/// each body is a node and each contact is an edge. Each contact has two contact nodes,
/// one for each attached body.
#[derive(Clone)]
pub struct ContactEdge<'a> {
    pub body: BodyHandleWeak<'a>,
    pub contact: ContactHandleWeak<'a>,
}

pub enum ContactType {
    Polygon,
}

pub struct Contact<'a> {
    contact_type: ContactType,

    pub edge_a: ContactEdge<'a>,
    pub edge_b: ContactEdge<'a>,

    pub fixture_a: FixtureHandle<'a>,
    pub fixture_b: FixtureHandle<'a>,

    pub manifold: Manifold,

    pub toi_count: u32,
    pub toi: f32,

    pub friction: f32,
    pub restitution: f32,
    pub tangent_speed: f32,

    // Used when crawling contact graph when forming islands.
    pub is_island: bool,
    // Set when shapes are touching.
    pub is_touching: bool,
    // This contact can be disabled (by user).
    pub is_enabled: bool,
    // This contact has a valid TOI in `toi`
    pub is_toi: bool,
}

impl<'a> Contact<'a> {
    pub fn new(fixture_a: FixtureHandle<'a>, fixture_b: FixtureHandle<'a>) -> ContactHandle<'a> {
        let result;
        let body_a = fixture_a.borrow().body.upgrade().unwrap();
        let body_b = fixture_b.borrow().body.upgrade().unwrap();
        let edge_a;
        let edge_b;
        let friction_a = fixture_a.borrow().friction;
        let friction_b = fixture_b.borrow().friction;
        let restitution_a = fixture_a.borrow().restitution;
        let restitution_b = fixture_b.borrow().restitution;
        unsafe {
            result = Rc::new(RefCell::new(Contact {
                contact_type: ContactType::Polygon,
                edge_a: mem::uninitialized(),
                edge_b: mem::uninitialized(),
                fixture_a: fixture_a,
                fixture_b: fixture_b,
                manifold: Manifold::new(),
                toi_count: 0,
                toi: 0.0,
                friction: mix_friction(friction_a, friction_b),
                restitution: mix_restitution(restitution_a, restitution_b),
                tangent_speed: 0.0,
                is_island: false,
                is_touching: false,
                is_enabled: true,
                is_toi: false,
            }));
            edge_a = ContactEdge {
                body: Rc::downgrade(&body_a),
                contact: Rc::downgrade(&result),
            };
            edge_b = ContactEdge {
                body: Rc::downgrade(&body_b),
                contact: Rc::downgrade(&result),
            };
            ptr::write(&mut result.borrow_mut().edge_a, edge_a.clone());
            ptr::write(&mut result.borrow_mut().edge_b, edge_b.clone());
        }
        body_a.borrow_mut().add_contact_edge(edge_a);
        body_b.borrow_mut().add_contact_edge(edge_b);
        result
    }

    pub fn update(&mut self) {
        let old_manifold = self.manifold.clone();

        let body_a = try_some!(self.fixture_a.borrow().body.upgrade());
        let body_b = try_some!(self.fixture_b.borrow().body.upgrade());

        let transform_a = body_a.borrow().get_transform();
        let transform_b = body_a.borrow().get_transform();

        let was_touching = self.is_touching;

        let fixture_a = self.fixture_a.borrow();
        let poly_a = match fixture_a.shape {
            Shape::Polygon(ref poly_a) => poly_a,
            _ => return,
        };
        let fixture_b = self.fixture_b.borrow();
        let poly_b = match fixture_b.shape {
            Shape::Polygon(ref poly_b) => poly_b,
            _ => return,
        };
        self.manifold = collision::collide_polygons(poly_a, &transform_a, poly_b, &transform_b);
        self.is_touching = self.manifold.points.len() > 0;

        // Match old contact ids to new contact ids and copy the
        // stored impulses to warm start the solver.
        for mp in &mut self.manifold.points {
            for mp_old in &old_manifold.points {
                if mp.id == mp_old.id {
                    mp.normal_impulse = mp_old.normal_impulse;
                    mp.tangent_impulse = mp_old.tangent_impulse;
                    break;
                }
            }
        }
    }

    pub fn is_touching(&self) -> bool {
        self.is_touching
    }
}
