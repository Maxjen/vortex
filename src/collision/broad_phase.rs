use super::{DynamicTree, TreeCallback};
use super::Aabb;
use cgmath::*;

pub trait BroadPhaseCallback {
    fn add_pair(&mut self, data_id_a: u32, data_id_b: u32);
}

struct Pair {
    proxy_id_a: u32,
    proxy_id_b: u32,
}

pub struct BroadPhase {
    tree: DynamicTree,
}

impl BroadPhase {
    pub fn new() -> Self {
        BroadPhase {
            tree: DynamicTree::new(),
        }
    }

    pub fn create_proxy(&mut self, aabb: &Aabb, data_id: Option<u32>) -> u32 {
        self.tree.create_proxy(aabb, data_id)
    }

    pub fn destroy_proxy(&mut self, proxy_id: u32) {
        self.tree.destroy_proxy(proxy_id);
    }

    pub fn move_proxy(&mut self, proxy_id: u32, aabb: &Aabb, displacement: Vector2<f32>) {

    }

    pub fn touch_proxy() {

    }

    pub fn get_fat_aabb(&self, proxy_id: u32) -> Aabb {
        self.tree.get_fat_aabb(proxy_id)
    }

    pub fn get_data_id(&self, proxy_id: u32) -> Option<u32> {
        self.tree.get_data_id(proxy_id)
    }

    pub fn update_pairs(&mut self, callback: &mut BroadPhaseCallback) {
        //self.tree.query(self, &Aabb::new());
        callback.add_pair(0, 1);
    }
}

/*impl TreeCallback for BroadPhase {
    fn query_callback(&mut self) {

    }
}*/
