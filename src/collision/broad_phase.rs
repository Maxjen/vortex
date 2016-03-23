use super::{Aabb, DynamicTree, TreeCallback};
use ::dynamics::FixtureHandle;
use cgmath::*;
use std::cmp;

pub trait BroadPhaseCallback<'a> {
    fn add_pair(&mut self, fixture_a: FixtureHandle<'a>, fixture_b: FixtureHandle<'a>);
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

pub struct BroadPhase<'a> {
    tree: DynamicTree<FixtureHandle<'a>>,
    pair_manager: PairManager,
    move_buffer: Vec<u32>,
}

impl<'a> BroadPhase<'a> {
    pub fn new() -> Self {
        BroadPhase {
            tree: DynamicTree::new(),
            pair_manager: PairManager::new(),
            move_buffer: Vec::new(),
        }
    }

    pub fn create_proxy(&mut self, aabb: &Aabb, fixture: FixtureHandle<'a>) -> u32 {
        let result = self.tree.create_proxy(aabb, Some(fixture));
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

    pub fn test_overlap(&self, proxy_id_a: u32, proxy_id_b: u32) -> bool {
        let aabb_a = self.tree.get_fat_aabb(proxy_id_a);
        let aabb_b = self.tree.get_fat_aabb(proxy_id_b);
        aabb_a.overlaps(&aabb_b)
    }

    pub fn get_fat_aabb(&self, proxy_id: u32) -> Aabb {
        self.tree.get_fat_aabb(proxy_id)
    }

    /*pub fn get_user_data(&self, proxy_id: u32) -> Option<&FixtureHandle> {
        self.tree.get_user_data(proxy_id)
    }*/

    pub fn update_pairs(&mut self, callback: &mut BroadPhaseCallback<'a>) {
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
            let fixture_a = self.tree.get_user_data(proxy_id_a).unwrap().clone();
            let fixture_b = self.tree.get_user_data(proxy_id_b).unwrap().clone();
            callback.add_pair(fixture_a, fixture_b);
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
