pub use self::dynamic_tree::{DynamicTree, TreeCallback};
pub use self::broad_phase::{BroadPhase, BroadPhaseCallback};
pub use self::polygon_shape::PolygonShape;

use cgmath::*;

mod dynamic_tree;
mod broad_phase;
mod polygon_shape;

#[derive(Clone)]
pub struct Aabb {
    pub min: Vector2<f32>,
    pub max: Vector2<f32>,
}

impl Aabb {
    pub fn new() -> Self {
        Aabb {
            min: Vector2::<f32>::new(0.0, 0.0),
            max: Vector2::<f32>::new(0.0, 0.0),
        }
    }

    pub fn get_perimeter(&self) -> f32 {
        let width = self.max.x - self.min.x;
        let height = self.max.y - self.min.y;
        2.0 * (width + height)
    }

    pub fn combine(aabb1: &Aabb, aabb2: &Aabb) -> Aabb {
        let min_x = f32::min(aabb1.min.x, aabb2.min.x);
        let min_y = f32::min(aabb1.min.y, aabb2.min.y);
        let max_x = f32::max(aabb1.max.x, aabb2.max.x);
        let max_y = f32::max(aabb1.max.y, aabb2.max.y);
        Aabb {
            min: Vector2::<f32>::new(min_x, min_y),
            max: Vector2::<f32>::new(max_x, max_y),
        }
    }

    pub fn contains(&self, aabb: &Aabb) -> bool {
        let mut result = true;
        result = result && self.min.x <= aabb.min.x;
        result = result && self.min.y <= aabb.min.y;
        result = result && self.max.x >= aabb.max.x;
        result = result && self.max.y >= aabb.max.y;
        result
    }

    pub fn overlaps(&self, aabb: &Aabb) -> bool {
        let d1 = aabb.min - self.max;
        let d2 = self.min - aabb.max;

        if d1.x > 0.0 || d1.y > 0.0 {
            return false;
        }

        if d2.x > 0.0 || d2.y > 0.0 {
            return false;
        }

        true
    }
}
