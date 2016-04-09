use ::collision::{BroadPhase, BroadPhaseCallback};
use super::{WorldHandleWeak, ContactHandle, Contact, BodyType, Body, Fixture, FixtureHandle};
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

    fn delete_contact_with_index(&mut self, contact_index: usize) {
        // Remove from the world.
        let contact = self.contacts.swap_remove(contact_index);

        let fixture_a = &contact.borrow().fixture_a;
        let fixture_b = &contact.borrow().fixture_b;
        let body_a = fixture_a.borrow().body.upgrade().unwrap();
        let body_b = fixture_b.borrow().body.upgrade().unwrap();

        // Remove from body A and body B.
        body_a.borrow_mut().remove_contact_edge(contact.borrow().edge_a.clone());
        body_b.borrow_mut().remove_contact_edge(contact.borrow().edge_b.clone());
    }

    pub fn delete_contact(&mut self, contact: ContactHandle<'a>) {
        let pc1 = &(*contact) as *const RefCell<Contact>;

        let mut remove = None;
        for (i, c) in self.contacts.iter().enumerate() {
            let c = c.clone();
            let pc2 = &(*c) as *const RefCell<Contact>;

            if pc1 == pc2 {
                remove = Some(i);
            }
        }

        if let Some(remove) = remove {
            self.delete_contact_with_index(remove);
        }
    }

    /// This is the top level collision call for the time step. Here all the narrow phase
    /// collison is processed for the world contact list.
    pub fn collide(&mut self) {
        let mut to_remove = Vec::new();
        // Update awake contacts.
        for (i, contact) in self.contacts.iter_mut().enumerate() {
            {
                let contact = contact.borrow();
                let fixture_a = contact.fixture_a.borrow();
                let fixture_b = contact.fixture_b.borrow();
                let body_a = fixture_a.body.upgrade().unwrap();
                let body_b = fixture_b.body.upgrade().unwrap();

                let static_a = if let BodyType::Static = body_a.borrow().body_type {
                    true
                } else {
                    false
                };
                let static_b = if let BodyType::Static = body_b.borrow().body_type {
                    true
                } else {
                    false
                };
                let active_a = body_a.borrow().is_awake() && !static_a;
                let active_b = body_b.borrow().is_awake() && !static_b;

                // At least one body must be awake and it must be dynamic or kinematic.
                if !active_a && !active_b {
                    continue;
                }
            }

            let world = try_some!(self.world.upgrade());
            let broad_phase = &world.borrow().broad_phase;
            let overlap;
            {
                let contact = contact.borrow();
                let proxy_id_a = contact.fixture_a.borrow().proxy_id.unwrap();
                let proxy_id_b = contact.fixture_b.borrow().proxy_id.unwrap();
                overlap = broad_phase.test_overlap(proxy_id_a, proxy_id_b);
            }

            // Here we destroy contacts that cease to overlap in the broad phase.
            if !overlap {
                to_remove.push(i);
                continue;
            }

            // The contact persists.
            contact.borrow_mut().update();
        }

        for i in &to_remove {
            self.delete_contact_with_index(*i);
        }
    }
}
