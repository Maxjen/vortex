use ::collision::{BroadPhase, BroadPhaseCallback};
use super::{WorldHandle};

pub struct ContactManager {
    broad_phase: BroadPhase,
    //world: WorldHandle,
}

impl ContactManager {
    pub fn new() -> Self {
        ContactManager {
            broad_phase: BroadPhase::new(),
            //world: world,
        }
    }
}

impl BroadPhaseCallback for ContactManager {
    fn add_pair(&mut self, data_id_a: Option<u32>, data_id_b: Option<u32>) {
        //println!("a: {:?}, b: {:?}", data_id_a, data_id_b);

        // TODO: check if contact already exists
    }
}

impl ContactManager {
    /*pub fn set_world_handle(&mut self, world: Option<WorldHandle<'a>>) {
        self.world = world;
    }*/
}
