use ::common;
use ::common::IndexPool;
use super::Aabb;
use ::common::DebugDraw;
use cgmath::*;
use std::cmp;

const NULL_NODE: u32 = !0 as u32;

pub trait TreeCallback {
    fn query_callback(&mut self, proxy_id: u32) -> bool;
}

pub struct TreeNode<T> {
    parent: u32,
    children: (u32, u32),
    aabb: Aabb,
    user_data: Option<T>,
    height: i32,
}

impl<T> TreeNode<T> {
    pub fn new() -> Self {
        TreeNode {
            parent: NULL_NODE,
            children: (NULL_NODE, NULL_NODE),
            aabb: Aabb::new(),
            user_data: None,
            height: -1,
        }
    }

    pub fn is_leaf(&self) -> bool {
        self.children.0 == NULL_NODE
    }
}

pub struct DynamicTree<T> {
    root: u32,
    nodes: Vec<TreeNode<T>>,
    index_pool: IndexPool,
    node_count: usize,
    node_capacity: usize,
}

impl<T> DynamicTree<T> {
    pub fn new() -> Self {
        let mut result = DynamicTree {
            root: NULL_NODE,
            nodes: Vec::with_capacity(16),
            index_pool: IndexPool::new(),
            node_count: 0,
            node_capacity: 16
        };
        for _ in 0..16 {
            result.nodes.push(TreeNode::new());
        }
        result
    }

    fn allocate_node(&mut self) -> u32 {
        if self.node_count == self.node_capacity {
            for _ in 0..self.node_capacity {
                self.nodes.push(TreeNode::new());
            }
            self.node_capacity *= 2;
        }
        self.node_count += 1;
        self.index_pool.get_index()
    }

    fn free_node(&mut self, node_id: u32) {
        self.nodes[node_id as usize].height = -1;
        self.index_pool.recycle_index(node_id);
        self.node_count -= 1;
    }

    /// Creates a proxy. Provide a tight fitting Aabb and an optional `u32`.
    pub fn create_proxy(&mut self, aabb: &Aabb, user_data: Option<T>) -> u32 {
        let proxy_id = self.allocate_node();

        let r = vec2(common::AABB_EXTENSION, common::AABB_EXTENSION);
        self.nodes[proxy_id as usize].aabb.min = aabb.min - r;
        self.nodes[proxy_id as usize].aabb.max = aabb.max + r;
        self.nodes[proxy_id as usize].user_data = user_data;
        self.nodes[proxy_id as usize].height = 0;

        self.insert_leaf(proxy_id);

        proxy_id
    }

    /// Destroys a proxy.
    pub fn destroy_proxy(&mut self, proxy_id: u32) {
        assert!((proxy_id as usize) < self.node_capacity);
        assert!(self.nodes[proxy_id as usize].is_leaf());

        self.remove_leaf(proxy_id);
        self.free_node(proxy_id);
    }

    /// Moves a proxy with a new Aabb. If the proxy has moved outside of its fattened Aabb,
    /// then the proxy is removed from the tree and reinserted. Otherwise the function
    /// returns immediately.
    ///
    /// Returns true if the proxy was reinserted.
    pub fn move_proxy(&mut self, proxy_id: u32, aabb: &Aabb, displacement: Vector2<f32>) -> bool     {
        assert!((proxy_id as usize) < self.node_capacity);
        assert!(self.nodes[proxy_id as usize].is_leaf());

        if self.nodes[proxy_id as usize].aabb.contains(aabb) {
            return false;
        }

        self.remove_leaf(proxy_id);

        let mut new_aabb = aabb.clone();
        let r = vec2(common::AABB_EXTENSION, common::AABB_EXTENSION);
        new_aabb.min = new_aabb.min - r;
        new_aabb.max = new_aabb.max + r;

        let d: Vector2<f32> = displacement * 2.0;

        if d.x < 0.0 {
            new_aabb.min.x += d.x;
        } else {
            new_aabb.max.x += d.x;
        }

        if d.y < 0.0 {
            new_aabb.min.y += d.y;
        } else {
            new_aabb.max.y += d.y;
        }

        self.nodes[proxy_id as usize].aabb = new_aabb;
        self.insert_leaf(proxy_id);
        true
    }

    /// Returns the fat Aabb for a proxy.
    pub fn get_fat_aabb(&self, proxy_id: u32) -> Aabb {
        assert!((proxy_id as usize) < self.node_capacity);
        self.nodes[proxy_id as usize].aabb.clone()
    }

    pub fn get_user_data(&self, proxy_id: u32) -> Option<&T> {
        assert!((proxy_id as usize) < self.node_capacity);
        self.nodes[proxy_id as usize].user_data.as_ref()
    }

    pub fn query(&self, callback: &mut TreeCallback, aabb: &Aabb) {
        let mut nodes = Vec::with_capacity(256);
        nodes.push(self.root as usize);

        while nodes.len() > 0 {
            let node_id = match nodes.pop() {
                Some(node_id) => node_id,
                None => continue,
            };

            if self.nodes[node_id].aabb.overlaps(aabb) {
                if self.nodes[node_id].is_leaf() {
                    let proceed = callback.query_callback(node_id as u32);
                    if !proceed {
                        return;
                    }
                } else {
                    nodes.push(self.nodes[node_id].children.0 as usize);
                    nodes.push(self.nodes[node_id].children.1 as usize);
                }
            }
        }
    }

    fn insert_leaf(&mut self, node_id: u32) {
        let leaf = node_id as usize;

        if self.root == NULL_NODE {
            self.root = node_id;
            self.nodes[leaf].parent = NULL_NODE;
            return;
        }

        let leaf_aabb = self.nodes[leaf].aabb.clone();
        let mut index = self.root as usize;
        while self.nodes[index].is_leaf() == false {
            let child1 = self.nodes[index].children.0 as usize;
            let child2 = self.nodes[index].children.1 as usize;

            let area = self.nodes[index].aabb.get_perimeter();

            let combined_aabb = Aabb::combine(&leaf_aabb, &self.nodes[index].aabb);
            let combined_area = combined_aabb.get_perimeter();

            let cost = 2.0 * combined_area;
            let inheritance_cost = 2.0 * (combined_area - area);

            let cost1;
            if self.nodes[child1].is_leaf() {
                let aabb = Aabb::combine(&leaf_aabb, &self.nodes[child1].aabb);
                cost1 = aabb.get_perimeter() + inheritance_cost;
            } else {
                let aabb = Aabb::combine(&leaf_aabb, &self.nodes[child1].aabb);
                let old_area = self.nodes[child1].aabb.get_perimeter();
                let new_area = aabb.get_perimeter();
                cost1 = new_area - old_area + inheritance_cost;
            }

            let cost2;
            if self.nodes[child2].is_leaf() {
                let aabb = Aabb::combine(&leaf_aabb, &self.nodes[child2].aabb);
                cost2 = aabb.get_perimeter() + inheritance_cost;
            } else {
                let aabb = Aabb::combine(&leaf_aabb, &self.nodes[child2].aabb);
                let old_area = self.nodes[child2].aabb.get_perimeter();
                let new_area = aabb.get_perimeter();
                cost2 = new_area - old_area + inheritance_cost;
            }

            if cost < cost1 && cost < cost2 {
                break;
            }

            if cost1 < cost2 {
                index = child1;
            } else {
                index = child2;
            }
        }
        let sibling = index;

        let old_parent = self.nodes[sibling].parent as usize;
        let new_parent = self.allocate_node() as usize;
        self.nodes[new_parent].parent = old_parent as u32;
        self.nodes[new_parent].children = (sibling as u32, leaf as u32);
        self.nodes[new_parent].aabb = Aabb::combine(&leaf_aabb, &self.nodes[sibling].aabb);
        self.nodes[new_parent].height = self.nodes[sibling].height + 1;
        self.nodes[sibling].parent = new_parent as u32;
        self.nodes[leaf].parent = new_parent as u32;

        if old_parent as u32 != NULL_NODE {
            if self.nodes[old_parent].children.0 == sibling as u32 {
                self.nodes[old_parent].children.0 = new_parent as u32;
            } else {
                self.nodes[old_parent].children.1 = new_parent as u32;
            }
        } else {
            self.root = new_parent as u32;
        }

        index = self.nodes[leaf].parent as usize;
        while index as u32 != NULL_NODE {
            index = self.balance(index as u32) as usize;

            let child1 = self.nodes[index].children.0 as usize;
            let child2 = self.nodes[index].children.1 as usize;

            assert!(child1 as u32 != NULL_NODE);
            assert!(child2 as u32 != NULL_NODE);

            self.nodes[index].aabb = Aabb::combine(&self.nodes[child1].aabb, &self.nodes[child2].aabb);
            self.nodes[index].height = 1 + cmp::max(self.nodes[child1].height, self.nodes[child2].height);

            index = self.nodes[index].parent as usize;
        }
    }

    fn remove_leaf(&mut self, node_id: u32) {
        if node_id == self.root {
            self.root = NULL_NODE;
            return;
        }

        let leaf = node_id as usize;
        let parent = self.nodes[leaf].parent as usize;
        let grand_parent = self.nodes[parent].parent as usize;
        let sibling = if self.nodes[parent].children.0 == leaf as u32 {
            self.nodes[parent].children.1 as usize
        } else {
            self.nodes[parent].children.0 as usize
        };

        if grand_parent as u32 != NULL_NODE {
            if self.nodes[grand_parent].children.0 == parent as u32 {
                self.nodes[grand_parent].children.0 = sibling as u32;
            } else {
                self.nodes[grand_parent].children.1 = sibling as u32;
            }
            self.nodes[sibling].parent = grand_parent as u32;
            self.free_node(parent as u32);

            let mut index = grand_parent;
            while index as u32 != NULL_NODE {
                index = self.balance(index as u32) as usize;

                let child1 = self.nodes[index].children.0 as usize;
                let child2 = self.nodes[index].children.1 as usize;

                self.nodes[index].aabb = Aabb::combine(&self.nodes[child1].aabb, &self.nodes[child2].aabb);
                self.nodes[index].height = 1 + cmp::max(self.nodes[child1].height, self.nodes[child2].height);

                index = self.nodes[index].parent as usize;
            }
        } else {
            self.root = sibling as u32;
            self.nodes[sibling].parent = NULL_NODE;
            self.free_node(parent as u32);
        }
    }

    fn balance(&mut self, node_id: u32) -> u32 {
        assert!(node_id != NULL_NODE);
        let a = node_id as usize;

        if self.nodes[node_id as usize].is_leaf() || self.nodes[node_id as usize].height < 2 {
            return node_id;
        }

        let b = self.nodes[node_id as usize].children.0 as usize;
        let c = self.nodes[node_id as usize].children.1 as usize;

        let balance = self.nodes[c].height - self.nodes[b].height;
        if balance > 1 {
            let f = self.nodes[c].children.0 as usize;
            let g = self.nodes[c].children.1 as usize;

            self.nodes[c].children.0 = a as u32;
            self.nodes[c].parent = self.nodes[a].parent;
            self.nodes[a].parent = c as u32;

            if self.nodes[c].parent != NULL_NODE {
                let c_parent = self.nodes[c].parent as usize;
                if self.nodes[c_parent].children.0 == a as u32 {
                    self.nodes[c_parent].children.0 = c as u32;
                } else {
                    self.nodes[c_parent].children.1 = c as u32;
                }
            } else {
                self.root = c as u32;
            }

            if self.nodes[f].height > self.nodes[g].height {
                self.nodes[c].children.1 = f as u32;
                self.nodes[a].children.1 = g as u32;
                self.nodes[g].parent = a as u32;
                self.nodes[a].aabb = Aabb::combine(&self.nodes[b].aabb, &self.nodes[g].aabb);
                self.nodes[c].aabb = Aabb::combine(&self.nodes[a].aabb, &self.nodes[f].aabb);

                self.nodes[a].height = 1 + cmp::max(self.nodes[b].height, self.nodes[g].height);
                self.nodes[c].height = 1 + cmp::max(self.nodes[a].height, self.nodes[f].height);
            } else {
                self.nodes[c].children.1 = g as u32;
                self.nodes[a].children.1 = f as u32;
                self.nodes[f].parent = a as u32;
                self.nodes[a].aabb = Aabb::combine(&self.nodes[b].aabb, &self.nodes[f].aabb);
                self.nodes[c].aabb = Aabb::combine(&self.nodes[a].aabb, &self.nodes[g].aabb);

                self.nodes[a].height = 1 + cmp::max(self.nodes[b].height, self.nodes[f].height);
                self.nodes[c].height = 1 + cmp::max(self.nodes[a].height, self.nodes[g].height);
            }
            return c as u32;
        }

        if balance < -1 {
            let d = self.nodes[b].children.0 as usize;
            let e = self.nodes[b].children.1 as usize;

            self.nodes[b].children.0 = a as u32;
            self.nodes[b].parent = self.nodes[a].parent;
            self.nodes[a].parent = b as u32;

            if self.nodes[b].parent != NULL_NODE {
                let b_parent = self.nodes[b].parent as usize;
                if self.nodes[b_parent].children.0 == a as u32 {
                    self.nodes[b_parent].children.0 = b as u32;
                } else {
                    self.nodes[b_parent].children.1 = b as u32;
                }
            } else {
                self.root = b as u32;
            }

            if self.nodes[d].height > self.nodes[e].height {
                self.nodes[b].children.1 = d as u32;
                self.nodes[a].children.0 = e as u32;
                self.nodes[e].parent = a as u32;
                self.nodes[a].aabb = Aabb::combine(&self.nodes[c].aabb, &self.nodes[e].aabb);
                self.nodes[b].aabb = Aabb::combine(&self.nodes[a].aabb, &self.nodes[d].aabb);

                self.nodes[a].height = 1 + cmp::max(self.nodes[c].height, self.nodes[e].height);
                self.nodes[b].height = 1 + cmp::max(self.nodes[a].height, self.nodes[d].height);
            } else {
                self.nodes[b].children.1 = e as u32;
                self.nodes[a].children.0 = d as u32;
                self.nodes[d].parent = a as u32;
                self.nodes[a].aabb = Aabb::combine(&self.nodes[c].aabb, &self.nodes[d].aabb);
                self.nodes[b].aabb = Aabb::combine(&self.nodes[a].aabb, &self.nodes[e].aabb);

                self.nodes[a].height = 1 + cmp::max(self.nodes[c].height, self.nodes[d].height);
                self.nodes[b].height = 1 + cmp::max(self.nodes[a].height, self.nodes[e].height);
            }
            return b as u32;
        }

        return a as u32;
    }

    pub fn get_height(&self) -> i32 {
        if self.root == NULL_NODE {
            return 0;
        }
        self.nodes[self.root as usize].height
    }

    pub fn get_area_ratio(&self) -> f32 {
        if self.root == NULL_NODE {
            return 0.0
        }

        let root_area = self.nodes[self.root as usize].aabb.get_perimeter();

        let mut total_area = 0.0;
        for node in &self.nodes {
            if node.height < 0 {
                continue
            }

            total_area += node.aabb.get_perimeter();
        }

        total_area / root_area
    }

    fn compute_node_height(&self, node_id: u32) -> i32 {
        assert!((node_id as usize) < self.node_capacity);

        if self.nodes[node_id as usize].is_leaf() {
            return 0
        }

        let height1 = self.compute_node_height(self.nodes[node_id as usize].children.0);
        let height2 = self.compute_node_height(self.nodes[node_id as usize].children.1);
        1 + cmp::max(height1, height2)
    }

    fn compute_height(&self) -> i32 {
        self.compute_node_height(self.root)
    }

    fn validate_structure(&self, node_id: u32) {
        if node_id == NULL_NODE {
            return;
        }

        if node_id == self.root {
            assert!(self.nodes[node_id as usize].parent == NULL_NODE);
        }

        let child1 = self.nodes[node_id as usize].children.0 as usize;
        let child2 = self.nodes[node_id as usize].children.1 as usize;

        if self.nodes[node_id as usize].is_leaf() {
            assert!(child1 as u32 == NULL_NODE);
            assert!(child2 as u32 == NULL_NODE);
            assert!(self.nodes[node_id as usize].height == 0);
            return;
        }

        assert!(child1 < self.node_capacity);
        assert!(child2 < self.node_capacity);

        assert!(self.nodes[child1].parent == node_id);
        assert!(self.nodes[child2].parent == node_id);

        self.validate_structure(child1 as u32);
        self.validate_structure(child2 as u32);
    }

    fn validate_metrics(&self, node_id: u32) {
        if node_id == NULL_NODE {
            return;
        }

        let child1 = self.nodes[node_id as usize].children.0 as usize;
        let child2 = self.nodes[node_id as usize].children.1 as usize;

        if self.nodes[node_id as usize].is_leaf() {
            assert!(child1 as u32 == NULL_NODE);
            assert!(child2 as u32 == NULL_NODE);
            assert!(self.nodes[node_id as usize].height == 0);
            return;
        }

        assert!(child1 < self.node_capacity);
        assert!(child2 < self.node_capacity);

        let height1 = self.nodes[child1].height;
        let height2 = self.nodes[child2].height;
        let height = 1 + cmp::max(height1, height2);
        assert!(self.nodes[node_id as usize].height == height);

        let aabb = Aabb::combine(&self.nodes[child1].aabb, &self.nodes[child2].aabb);

        assert!(aabb.min == self.nodes[node_id as usize].aabb.min);
        assert!(aabb.max == self.nodes[node_id as usize].aabb.max);

        self.validate_metrics(child1 as u32);
        self.validate_metrics(child2 as u32);
    }

    pub fn validate(&self) {
        self.validate_structure(self.root);
        self.validate_metrics(self.root);

        // TODO
        //let mut free_count = 0;

        assert!(self.get_height() == self.compute_height());
    }

    pub fn get_max_balance(&self) -> i32 {
        let mut max_balance = 0;
        for node in &self.nodes {
            if node.height <= 1 {
                continue;
            }

            assert!(node.is_leaf() == false);

            let child1 = node.children.0 as usize;
            let child2 = node.children.1 as usize;
            let balance = i32::abs(self.nodes[child2].height - self.nodes[child1].height);
            max_balance = cmp::max(max_balance, balance);
        }
        max_balance
    }

    pub fn rebuild_bottom_up(&mut self) {
        let mut nodes = Vec::<usize>::with_capacity(self.node_count);

        for i in 0..self.nodes.len() {
            if self.nodes[i].height < 0 {
                continue;
            }

            if self.nodes[i].is_leaf() {
                self.nodes[i].parent = NULL_NODE;
                nodes.push(i);
            } else {
                self.free_node(i as u32);
            }
        }

        let mut count = nodes.len();
        while count > 1 {
            let mut min_cost = None;
            let mut i_min = 0;
            let mut j_min = 1;
            for i in 0..count {
                let aabbi = self.nodes[nodes[i]].aabb.clone();

                for j in i + 1..count {
                    let aabbj = self.nodes[nodes[j]].aabb.clone();
                    let aabb = Aabb::combine(&aabbi, &aabbj);
                    let cost = aabb.get_perimeter();
                    if let Some(cur_min_cost) = min_cost {
                        if cost < cur_min_cost {
                            i_min = i;
                            j_min = j;
                            min_cost = Some(cost);
                        }
                    } else {
                        i_min = i;
                        j_min = j;
                        min_cost = Some(cost);
                    }
                }
            }

            let child1 = nodes[i_min];
            let child2 = nodes[j_min];

            let parent = self.allocate_node() as usize;
            self.nodes[parent].children.0 = child1 as u32;
            self.nodes[parent].children.1 = child2 as u32;
            self.nodes[parent].height = 1 + cmp::max(self.nodes[child1].height, self.nodes[child2].height);
            self.nodes[parent].parent = NULL_NODE;

            self.nodes[child1].parent = parent as u32;
            self.nodes[child2].parent = parent as u32;

            nodes[j_min] = nodes[count - 1];
            nodes[i_min] = parent;
            count -= 1;
        }

        self.root = nodes[0] as u32;
        self.validate();
    }

    pub fn shift_origin(&mut self, new_origin: Vector2<f32>) {
        for node in &mut self.nodes {
            node.aabb.min -= new_origin;
            node.aabb.max -= new_origin;
        }
    }

    pub fn print(&self) {
        for node in &self.nodes {
            if node.height < 0 {
                continue;
            }

            println!("min {:?} max {:?}", node.aabb.min, node.aabb.max);
        }
    }
}
