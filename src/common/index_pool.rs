pub struct IndexPool {
    recycled: Vec<u32>,
    next_index: u32,
}

impl IndexPool {
    pub fn new() -> IndexPool {
        IndexPool {
            recycled: Vec::new(),
            next_index: 0,
        }
    }

    pub fn get_index(&mut self) -> u32 {
        match self.recycled.pop() {
            Some(index) => index,
            None => {
                self.next_index += 1;
                self.next_index - 1
            }
        }
    }

    pub fn recycle_index(&mut self, index: u32) {
        if index < self.next_index {
            self.recycled.push(index);
        }
    }
}
