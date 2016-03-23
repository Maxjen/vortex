use super::PolygonShape;
use ::collision::Aabb;
use ::common::Transform2d;

pub enum Shape {
    Polygon(PolygonShape),
}

impl Shape {
    pub fn compute_aabb(&self, transform: &Transform2d) -> Aabb {
        match self {
            &Shape::Polygon(ref shape) => shape.compute_aabb(transform),
        }
    }
}
