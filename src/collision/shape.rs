use super::PolygonShape;
use ::collision::Aabb;
use ::common::Transform2d;
use cgmath::*;

pub enum Shape {
    Polygon(PolygonShape),
    Tmp,
}

impl Shape {
    /// Given a transform, compute the associated axis aligned bounding box for a child shape.
    pub fn compute_aabb(&self, transform: &Transform2d) -> Aabb {
        match self {
            &Shape::Polygon(ref shape) => shape.compute_aabb(transform),
            _ => Aabb::new(),
        }
    }

    /// Compute the mass properties of this shape using its dimensions and density.
    /// The inertia tensor is computed about the local origin.
    pub fn compute_mass(&self, density: f32) -> (f32, Vector2<f32>, f32) {
        match self {
            &Shape::Polygon(ref shape) => shape.compute_mass(density),
            _ => (0.0, Vector2::zero(), 0.0),
        }
    }

    pub fn get_radius(&self) -> f32 {
        match self {
            &Shape::Polygon(_) => ::common::POLYGON_RADIUS,
            _ => 0.0,
        }
    }
}
