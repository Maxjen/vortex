use std::rc::{Rc, Weak};
use std::cell::RefCell;
use std::mem;
use std::ptr;
use super::{FixtureHandle, World, BodyHandle, BodyHandleWeak};
use ::collision::Manifold;
use ::collision;

macro_rules! try_some {
    ($e:expr) => (match $e { Some(s) => s, None => return })
}

pub type ContactHandle<'a> = Rc<RefCell<PolygonContact<'a>>>;
pub type ContactHandleWeak<'a> = Weak<RefCell<PolygonContact<'a>>>;

/// A contact edge is used to connect bodies and contacts together in a contact graph where
/// each body is a node and each contact is an edge. Each contact has two contact nodes,
/// one for each attached body.
#[derive(Clone)]
pub struct ContactEdge<'a> {
    pub body: BodyHandleWeak<'a>,
    pub contact: ContactHandleWeak<'a>,
}

pub struct PolygonContact<'a> {
    edge_a: ContactEdge<'a>,
    edge_b: ContactEdge<'a>,

    pub fixture_a: FixtureHandle<'a>,
    pub fixture_b: FixtureHandle<'a>,

    manifold: Manifold,

    pub is_island: bool,
    pub is_touching: bool,
}

impl<'a> PolygonContact<'a> {
    pub fn new(fixture_a: FixtureHandle<'a>, fixture_b: FixtureHandle<'a>,
               body_a: BodyHandle<'a>, body_b: BodyHandle<'a>) -> ContactHandle<'a> {
        let result;
        let edge_a;
        let edge_b;
        unsafe {
            result = Rc::new(RefCell::new(PolygonContact {
                edge_a: mem::uninitialized(),
                edge_b: mem::uninitialized(),
                fixture_a: fixture_a,
                fixture_b: fixture_b,
                manifold: Manifold::new(),
                is_island: false,
                is_touching: false,
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

    pub fn update(&mut self, world: &World) {
        let old_manifold = self.manifold.clone();

        let body_a = try_some!(self.fixture_a.borrow().body.upgrade());
        let body_b = try_some!(self.fixture_b.borrow().body.upgrade());

        let transform_a = body_a.borrow().get_transform();
        let transform_b = body_a.borrow().get_transform();

        let was_touching = self.is_touching;

        self.manifold = collision::collide_polygons(&self.fixture_a.borrow().shape, &transform_a, &self.fixture_b.borrow().shape, &transform_b);
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
