use time;

pub struct Timer {
    start: u64,
}

impl Timer {
    pub fn new() -> Timer {
        Timer {
            start: time::precise_time_ns(),
        }
    }

    pub fn reset(&mut self) {
        self.start = time::precise_time_ns();
    }

    pub fn get_milliseconds(&self) -> f32 {
        (time::precise_time_ns() - self.start) as f32 / 1000000.0
    }
}
