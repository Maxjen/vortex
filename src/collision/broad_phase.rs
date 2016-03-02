use super::{DynamicTree, TreeCallback};
use super::Aabb;
use cgmath::*;
use std::cmp;

pub trait BroadPhaseCallback {
    fn add_pair(&mut self, data_id_a: Option<u32>, data_id_b: Option<u32>);
}

struct PairManager {
    pairs: Vec<(u32, u32)>,
    cur_proxy_id: u32,
}

impl PairManager {
    fn new() -> Self {
        PairManager {
            pairs: Vec::new(),
            cur_proxy_id: 0,
        }
    }
}

impl TreeCallback for PairManager {
    fn query_callback(&mut self, proxy_id: u32) -> bool {
        /*let cur_proxy_id = match self.cur_proxy_id {
            Some(cur_proxy_id) => cur_proxy_id,
            None => return false,
        }*/

        if proxy_id == self.cur_proxy_id {
            return true;
        }

        let proxy_id_a = cmp::min(proxy_id, self.cur_proxy_id);
        let proxy_id_b = cmp::max(proxy_id, self.cur_proxy_id);
        self.pairs.push((proxy_id_a, proxy_id_b));
        true
    }
}

pub struct BroadPhase {
    tree: DynamicTree,
    pair_manager: PairManager,
    move_buffer: Vec<u32>,
}

impl BroadPhase {
    pub fn new() -> Self {
        BroadPhase {
            tree: DynamicTree::new(),
            pair_manager: PairManager::new(),
            move_buffer: Vec::new(),
        }
    }

    pub fn create_proxy(&mut self, aabb: &Aabb, data_id: Option<u32>) -> u32 {
        let result = self.tree.create_proxy(aabb, data_id);
        self.move_buffer.push(result);
        result
    }

    pub fn destroy_proxy(&mut self, proxy_id: u32) {
        self.remove_from_move_buffer(proxy_id);
        self.tree.destroy_proxy(proxy_id);
    }

    pub fn move_proxy(&mut self, proxy_id: u32, aabb: &Aabb, displacement: Vector2<f32>) {
        if self.tree.move_proxy(proxy_id, aabb, displacement) {
            self.move_buffer.push(proxy_id);
        }
    }

    pub fn touch_proxy(&mut self, proxy_id: u32) {
        self.move_buffer.push(proxy_id);
    }

    fn remove_from_move_buffer(&mut self, proxy_id: u32) {
        //let to_remove = self.move_buffer.iter().enumerate().filter(|&(i, id)| *id == proxy_id).map(|(i, _)| i).collect::<Vec<usize>>();

        let mut to_remove = Vec::new();
        for (i, id) in self.move_buffer.iter().enumerate() {
            if proxy_id == *id {
                to_remove.push(i);
            }
        }
        for i in &to_remove {
            self.move_buffer.swap_remove(*i);
        }
    }

    pub fn get_fat_aabb(&self, proxy_id: u32) -> Aabb {
        self.tree.get_fat_aabb(proxy_id)
    }

    pub fn get_data_id(&self, proxy_id: u32) -> Option<u32> {
        self.tree.get_data_id(proxy_id)
    }

    pub fn update_pairs(&mut self, callback: &mut BroadPhaseCallback) {
        self.pair_manager.pairs.clear();

        for i in &self.move_buffer {
            self.pair_manager.cur_proxy_id = *i;
            let fat_aabb = self.tree.get_fat_aabb(*i);
            self.tree.query(&mut self.pair_manager, &fat_aabb);
        }

        self.move_buffer.clear();

        self.pair_manager.pairs.sort();
        self.pair_manager.pairs.dedup();

        for &(proxy_id_a, proxy_id_b) in &self.pair_manager.pairs {
            let data_id_a = self.tree.get_data_id(proxy_id_a);
            let data_id_b = self.tree.get_data_id(proxy_id_b);
            callback.add_pair(data_id_a, data_id_b);
        }
    }

    pub fn print(&self) {
        self.tree.print();
    }
}

/*impl TreeCallback for BroadPhase {
    fn query_callback(&mut self) {

    }
}*/
