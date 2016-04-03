use ::collision::{BroadPhase, BroadPhaseCallback};
use super::{WorldHandleWeak, ContactHandle, Contact, Body, Fixture, FixtureHandle};
use std::rc::Rc;
use std::cell::RefCell;

macro_rules! try_some {
    ($e:expr) => (match $e { Some(s) => s, None => return })
}

pub struct ContactManager<'a> {
    pub contacts: Vec<ContactHandle<'a>>,
    world: WorldHandleWeak<'a>,
}

impl<'a> BroadPhaseCallback<'a> for ContactManager<'a> {
    fn add_pair(&mut self, fixture_a: FixtureHandle<'a>, fixture_b: FixtureHandle<'a>) {
        let body_a = try_some!(fixture_a.borrow().body.upgrade());
        let body_b = try_some!(fixture_b.borrow().body.upgrade());

        // Are the fixtures on the same body?
        let pb1 = &(*body_a) as *const RefCell<Body>;
        let pb2 = &(*body_b) as *const RefCell<Body>;
        if pb1 == pb2 {
            return;
        }

        // Does a contact already exist?
        {
            let body_b = body_b.borrow();
            let contact_edges = body_b.get_contact_edges();
            for ce in contact_edges {
                let body = try_some!(ce.body.upgrade());
                let pb1 = &(*body) as *const RefCell<Body>;
                let pb2 = &(*body_a) as *const RefCell<Body>;
                if pb1 == pb2 {
                    let contact = try_some!(ce.contact.upgrade());
                    let pfa = &(*fixture_a) as *const RefCell<Fixture>;
                    let pfb = &(*fixture_b) as *const RefCell<Fixture>;
                    let pf1 = &(*contact.borrow().fixture_a) as *const RefCell<Fixture>;
                    let pf2 = &(*contact.borrow().fixture_b) as *const RefCell<Fixture>;
                    if pfa == pf1 && pfb == pf2 {
                        // A contact already exists;
                        return;
                    }
                    if pfa == pf2 && pfb == pf1 {
                        // A contact already exists;
                        return;
                    }
                }
            }
        }

        // Insert into the world.
        let contact = Contact::new(fixture_a, fixture_b);
        self.contacts.push(contact);
    }
}

impl<'a> ContactManager<'a> {
    pub fn new(world: WorldHandleWeak<'a>) -> ContactManager<'a> {
        ContactManager {
            world: world,
            contacts: Vec::new(),
        }
    }

    pub fn get_contact_count(&self) -> usize {
        self.contacts.len()
    }

    /// This is the top level collision call for the time step. Here all the narrow phase
    /// collison is processed for the world contact list.
    pub fn collide(&mut self) {
        let mut to_remove = Vec::new();
        for (i, contact) in self.contacts.iter_mut().enumerate() {
            let world = try_some!(self.world.upgrade());
            let broad_phase = &world.borrow().broad_phase;
            let overlap;
            {
                let contact = contact.borrow();
                overlap = broad_phase.test_overlap(contact.fixture_a.borrow().proxy_id, contact.fixture_b.borrow().proxy_id);
            }

            // Here we destroy contacts that cease to overlap in the broad phase.
            if !overlap {
                to_remove.push(i);
                continue;
            }

            // The contact persists.
            contact.borrow_mut().update(&world.borrow());
        }

        for i in &to_remove {
            self.contacts.swap_remove(*i);
        }
    }
}
