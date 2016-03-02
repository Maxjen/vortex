use ::collision::{BroadPhase, BroadPhaseCallback};

pub struct ContactManager {
    broad_phase: BroadPhase,
}

impl ContactManager {
    pub fn new() -> Self {
        ContactManager {
            broad_phase: BroadPhase::new(),
        }
    }
}

impl BroadPhaseCallback for ContactManager {
    fn add_pair(&mut self, data_id_a: Option<u32>, data_id_b: Option<u32>) {
        println!("a: {:?}, b: {:?}", data_id_a, data_id_b);
    }
}
