pub use self::index_pool::IndexPool;
pub use self::debug_draw::DebugDraw;

use cgmath::*;

mod index_pool;
pub mod debug_draw;

pub const AABB_EXTENSION: f32 = 0.1;
pub const LINEAR_SLOP: f32 = 0.005;
pub const POLYGON_RADIUS: f32 = 2.0 * LINEAR_SLOP;

#[derive(Clone, Copy)]
pub struct Rotation2d {
    sin: f32,
    cos: f32,
}

impl Rotation2d {
    pub fn new() -> Self {
        Rotation2d {
            sin: 0.0,
            cos: 1.0,
        }
    }

    pub fn set_angle(&mut self, angle: f32) {
        self.sin = angle.sin();
        self.cos = angle.cos();
    }

    pub fn get_angle(&self) -> f32 {
        f32::atan2(self.sin, self.cos)
    }
}

#[derive(Clone, Copy)]
pub struct Transform2d {
    pub position: Vector2<f32>,
    pub rotation: Rotation2d,
}

impl Transform2d {
    pub fn new() -> Self {
        Transform2d {
            position: Vector2::<f32>::new(0.0, 0.0),
            rotation: Rotation2d::new(),
        }
    }

    pub fn apply_to_vector(&self, v: &Vector2<f32>) -> Vector2<f32> {
        let x = self.rotation.cos * v.x - self.rotation.sin * v.y + self.position.x;
        let y = self.rotation.sin * v.x + self.rotation.cos * v.y + self.position.y;
        Vector2::new(x, y)
    }
}
