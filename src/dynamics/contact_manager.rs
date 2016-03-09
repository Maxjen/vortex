use ::collision::{BroadPhase, BroadPhaseCallback};
use super::{WorldHandleWeak, PolygonContact};

macro_rules! try_some {
    ($e:expr) => (match $e { Some(s) => s, None => return })
}

pub struct ContactManager<'a> {
    contacts: Vec<PolygonContact>,
    world: WorldHandleWeak<'a>,
}

impl<'a> BroadPhaseCallback for ContactManager<'a> {
    fn add_pair(&mut self, data_id_a: Option<u32>, data_id_b: Option<u32>) {
        //println!("a: {:?}, b: {:?}", data_id_a, data_id_b);
        let fixture_id_a = try_some!(data_id_a);
        let fixture_id_b = try_some!(data_id_b);

        let world = try_some!(self.world.upgrade());

        let fixture_a = try_some!(world.borrow().get_fixture(fixture_id_a));
        let fixture_b = try_some!(world.borrow().get_fixture(fixture_id_b));

        let body_id_a = fixture_a.borrow().body_id;
        let body_id_b = fixture_b.borrow().body_id;

        if body_id_a == body_id_b {
            return;
        }

        let contact = PolygonContact::new(fixture_a, fixture_b);
        self.contacts.push(contact);

        // TODO: check if contact already exists
    }
}

impl<'a> ContactManager<'a> {
    pub fn new(world: WorldHandleWeak<'a>) -> ContactManager<'a> {
        ContactManager {
            world: world,
            contacts: Vec::new(),
        }
    }

    pub fn collide(&mut self) {
        let mut to_remove = Vec::new();
        for (i, contact) in self.contacts.iter_mut().enumerate() {
            let world = try_some!(self.world.upgrade());
            let broad_phase = &world.borrow().broad_phase;
            let overlap = broad_phase.test_overlap(contact.fixture_a.borrow().proxy_id, contact.fixture_b.borrow().proxy_id);
            if !overlap {
                to_remove.push(i);
                continue;
            }
            contact.update(&world.borrow());
        }
        for i in &to_remove {
            self.contacts.swap_remove(*i);
        }
    }
}
