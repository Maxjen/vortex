use super::PolygonShape;
use ::collision::Aabb;
use ::common::Transform2d;

pub enum Shape {
    Polygon(PolygonShape),
    Tmp,
}

impl Shape {
    pub fn compute_aabb(&self, transform: &Transform2d) -> Aabb {
        match self {
            &Shape::Polygon(ref shape) => shape.compute_aabb(transform),
            _ => Aabb::new(),
        }
    }

    pub fn get_radius(&self) -> f32 {
        match self {
            &Shape::Polygon(_) => ::common::POLYGON_RADIUS,
            _ => 0.0,
        }
    }
}
