pub use self::dynamic_tree::{DynamicTree, TreeCallback};
pub use self::broad_phase::{BroadPhase, BroadPhaseCallback};
pub use self::shape::Shape;
pub use self::polygon_shape::PolygonShape;
pub use self::collide_polygons::collide_polygons;
pub use self::distance::{DistanceProxy, SimplexCache, distance};
pub use self::time_of_impact::{ToiState, time_of_impact};

use cgmath::*;
use ::common::{Transform2d};

mod dynamic_tree;
mod broad_phase;
mod shape;
mod polygon_shape;
mod collide_polygons;
mod time_of_impact;
mod distance;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FeatureType {
    Vertex,
    Face,
}

/// Contact ids to facilitate warm starting.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct ContactId {
    // The features that intersect to form the contact point.
    //
    // Feature index on shape_a
    index_a: u8,
    // Feature index on shape_b
    index_b: u8,
    // Feature type on shape_a
    type_a: FeatureType,
    // Feature type on shape_b
    type_b: FeatureType,
}

impl ContactId {
    pub fn new() -> Self {
        ContactId {
            index_a: 0,
            index_b: 0,
            type_a: FeatureType::Vertex,
            type_b: FeatureType::Vertex,
        }
    }
}

/// A contact point belonging to a manifold.
#[derive(Clone)]
pub struct ManifoldPoint {
    pub local_point: Vector2<f32>,
    pub normal_impulse: f32,
    pub tangent_impulse: f32,
    pub id: ContactId,
}

impl ManifoldPoint {
    pub fn new(local_point: Vector2<f32>, id: ContactId) -> Self {
        ManifoldPoint {
            local_point: local_point,
            normal_impulse: 0.0,
            tangent_impulse: 0.0,
            id: id,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum ManifoldType {
    FaceA,
    FaceB,
}

/// A manifold for two touching convex shapes.
#[derive(Clone)]
pub struct Manifold {
    /// The points of contact.
    pub points: Vec<ManifoldPoint>,
    pub local_normal: Vector2<f32>,
    pub local_point: Vector2<f32>,
    pub manifold_type: ManifoldType,
}

impl Manifold {
    pub fn new() -> Self {
        Manifold {
            points: Vec::with_capacity(2),
            local_normal: Vector2::zero(),
            local_point: Vector2::zero(),
            manifold_type: ManifoldType::FaceA,
        }
    }
}

/// This is used to compute the current state of a contact manifold.
pub struct WorldManifold {
    /// World vector pointing from A to B.
    pub normal: Vector2<f32>,
    /// World contact point. (point of intersection)
    pub points: Vec<Vector2<f32>>,
    /// A negative value indicates overlap, in meters.
    separations: Vec<f32>,
}

impl Default for WorldManifold {
    fn default() -> WorldManifold {
        WorldManifold {
            normal: Vector2::zero(),
            points: Vec::with_capacity(2),
            separations: Vec::with_capacity(2),
        }
    }
}

impl WorldManifold {
    /// Evaluate the manifold with supplied transforms. This assumes modest motion from the
    /// original state. The radii must come from the shapes that generated the manifold.
    pub fn initialize(&mut self, manifold: &Manifold,
           transform_a: &Transform2d, radius_a: f32,
           transform_b: &Transform2d, radius_b: f32) {
        if manifold.points.len() == 0 {
            return;
        }

        self.points.clear();

        match manifold.manifold_type {
            ManifoldType::FaceA => {
                self.normal = transform_a.rotation.apply(&manifold.local_normal);
                let plane_point = transform_a.apply(&manifold.local_normal);

                for mp in &manifold.points {
                    let clip_point = transform_b.apply(&mp.local_point);
                    let c_a = clip_point + self.normal * (radius_a - dot(clip_point - plane_point, self.normal));
                    let c_b = clip_point - self.normal * radius_b;
                    self.points.push((c_a + c_b) * 0.5);
                    self.separations.push(dot(c_b - c_a, self.normal));
                }
            }
            ManifoldType::FaceB => {
                self.normal = transform_b.rotation.apply(&manifold.local_normal);
                let plane_point = transform_b.apply(&manifold.local_normal);

                for mp in &manifold.points {
                    let clip_point = transform_a.apply(&mp.local_point);
                    let c_b = clip_point + self.normal * (radius_b - dot(clip_point - plane_point, self.normal));
                    let c_a = clip_point - self.normal * radius_a;
                    self.points.push((c_a + c_b) * 0.5);
                    self.separations.push(dot(c_a - c_b, self.normal));
                }

                // Ensure normal points from A to B.
                self.normal = -self.normal;
            }
        }
    }
}

/// Used for computing contact manifolds.
#[derive(Clone, Copy)]
pub struct ClipVertex {
    v: Vector2<f32>,
    id: ContactId,
}

impl ClipVertex {
    pub fn new() -> Self {
        ClipVertex {
            v: Vector2::zero(),
            id: ContactId::new(),
        }
    }

    /// Sutherland-Hodgman clipping.
    pub fn clip_segment_to_line(segment: &[ClipVertex; 2], normal: &Vector2<f32>,
                                offset: f32, vertex_index_a: usize) -> Option<[ClipVertex; 2]> {
        // Calculate the distance of the end points to the line.
        let distance0 = normal.dot(segment[0].v) - offset;
        let distance1 = normal.dot(segment[1].v) - offset;

        let mut clipped_segment = [ClipVertex::new(), ClipVertex::new()];
        let mut i = 0;
        if distance0 <= 0.0 {
            clipped_segment[i] = segment[0];
            i += 1;
        }
        if distance1 <= 0.0 {
            clipped_segment[i] = segment[1];
            i += 1;
        }

        if i == 0 {
            return None;
        }

        // If the points are on different sides of the plane.
        if distance0 * distance1 < 0.0 {
            // Find intersection point of edge and plane.
            let interp = distance0 / (distance0 - distance1);

            // vertex_a is hitting edge_b.
            let index_b = segment[0].id.index_b;

            clipped_segment[1] = ClipVertex {
                v: segment[0].v + (segment[1].v - segment[0].v) * interp,
                id: ContactId {
                    index_a: vertex_index_a as u8,
                    index_b: index_b,
                    type_a: FeatureType::Vertex,
                    type_b: FeatureType::Face,
                },
            };
        }
        Some(clipped_segment)
    }
}

/// An axis aligned bounding box.
#[derive(Clone)]
pub struct Aabb {
    /// The lower bound of the Aabb.
    pub min: Vector2<f32>,
    /// The upper bound of the Aabb.
    pub max: Vector2<f32>,
}

impl Aabb {
    /// Constructs a new Aabb.
    pub fn new() -> Self {
        Aabb {
            min: Vector2::zero(),
            max: Vector2::zero(),
        }
    }

    /// Returns the perimeter.
    pub fn get_perimeter(&self) -> f32 {
        let width = self.max.x - self.min.x;
        let height = self.max.y - self.min.y;
        2.0 * (width + height)
    }

    /// Combines the provided Aabbs into a new one.
    pub fn combine(aabb1: &Aabb, aabb2: &Aabb) -> Aabb {
        let min_x = f32::min(aabb1.min.x, aabb2.min.x);
        let min_y = f32::min(aabb1.min.y, aabb2.min.y);
        let max_x = f32::max(aabb1.max.x, aabb2.max.x);
        let max_y = f32::max(aabb1.max.y, aabb2.max.y);
        Aabb {
            min: Vector2::new(min_x, min_y),
            max: Vector2::new(max_x, max_y),
        }
    }

    /// Returns true if this Aabb contains the provided Aabb.
    pub fn contains(&self, aabb: &Aabb) -> bool {
        let mut result = true;
        result = result && self.min.x <= aabb.min.x;
        result = result && self.min.y <= aabb.min.y;
        result = result && self.max.x >= aabb.max.x;
        result = result && self.max.y >= aabb.max.y;
        result
    }

    /// Returns true if this Aabb overlaps the provided Aabb.
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
