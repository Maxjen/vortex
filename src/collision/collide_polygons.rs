use super::{PolygonShape, ManifoldPoint, ManifoldType, Manifold, ContactId, FeatureType, ClipVertex};
use ::common;
use ::common::Transform2d;
use cgmath::*;
use std::f32;

/// Finds the max separation between `poly1` and `poly2` using edge normalse from `poly1`.
fn find_max_separation(poly1: &PolygonShape, transform1: &Transform2d, poly2: &PolygonShape, transform2: &Transform2d) -> (usize, f32) {
    let count1 = poly1.vertices.len();
    let count2 = poly2.vertices.len();
    let transform = transform2.mul_t(transform1);

    let mut best_index = 0;
    let mut max_separation = f32::MIN;
    for i in 0..count1 {
        // Get poly1 normal in frame2
        let n = transform.rotation.apply(&poly1.normals[i]);
        let v1 = transform.apply(&poly1.vertices[i]);

        // Find deepest point for normal i
        let mut si = f32::MAX;
        for j in 0..count2 {
            let sij = n.dot(poly2.vertices[j] - v1);
            if sij < si {
                si = sij;
            }
        }

        if si > max_separation {
            max_separation = si;
            best_index = i;
        }
    }

    (best_index, max_separation)
}

fn find_incident_edge(poly1: &PolygonShape, transform1: &Transform2d, edge1: usize,
                      poly2: &PolygonShape, transform2: &Transform2d) -> [ClipVertex; 2] {
    // Get the normal of the reference edge in poly2's frame.
    let normal1 = transform2.rotation.apply_t(&transform1.rotation.apply(&poly1.normals[edge1]));

    // Find the incident edge on poly2.
    let mut index = 0;
    let mut min_dot = f32::MAX;
    for (i, n) in poly2.normals.iter().enumerate() {
        let dot = normal1.dot(*n);
        if dot < min_dot {
            min_dot = dot;
            index = i;
        }
    }

    // Build the clip vertices for the incident edge.
    let index1 = index;
    let index2 = if index + 1 < poly2.vertices.len() {
        index1 + 1
    } else {
        0
    };

    [
        ClipVertex {
            v: transform2.apply(&poly2.vertices[index1]),
            id: ContactId {
                index_a: edge1 as u8,
                index_b: index1 as u8,
                type_a: FeatureType::Face,
                type_b: FeatureType::Vertex,
            },
        },
        ClipVertex {
            v: transform2.apply(&poly2.vertices[index2]),
            id: ContactId {
                index_a: edge1 as u8,
                index_b: index2 as u8,
                type_a: FeatureType::Face,
                type_b: FeatureType::Vertex,
            },
        },
    ]
}

// Find edge normal of max separation on `poly_a` - return if separating axis is found
// Find edge normal of max separation on `poly_b` - return if separating axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// collide_polygons
// clip

// The normal points from `polyA` to `polyB`
pub fn collide_polygons(poly_a: &PolygonShape, transform_a: &Transform2d,
                        poly_b: &PolygonShape, transform_b: &Transform2d) -> Manifold {
    let mut manifold = Manifold::new();
    let total_radius = 2.0 * common::POLYGON_RADIUS;

    let (edge_a, separation_a) = find_max_separation(poly_a, transform_a, poly_b, transform_b);
    if separation_a > total_radius {
        return manifold;
    }

    let (edge_b, separation_b) = find_max_separation(poly_b, transform_b, poly_a, transform_a);
    if separation_b > total_radius {
        return manifold;
    }

    let (poly1, poly2);
    let (transform1, transform2);
    let edge1;
    let flip;
    let k_tol = 0.1 * common::LINEAR_SLOP;

    if separation_b > separation_a {
        manifold.manifold_type = ManifoldType::FaceB;
        poly1 = poly_b;
        poly2 = poly_a;
        transform1 = transform_b;
        transform2 = transform_a;
        edge1 = edge_b;
        flip = true;
    } else {
        manifold.manifold_type = ManifoldType::FaceA;
        poly1 = poly_a;
        poly2 = poly_b;
        transform1 = transform_a;
        transform2 = transform_b;
        edge1 = edge_a;
        flip = false;
    }

    let incident_edge = find_incident_edge(poly1, transform1, edge1, poly2, transform2);

    let index1 = edge1;
    let index2 = if edge1 + 1 < poly1.vertices.len() {
        edge1 + 1
    } else {
        0
    };

    let mut v11 = poly1.vertices[index1];
    let mut v12 = poly1.vertices[index2];

    let mut local_tangent = v12 - v11;
    local_tangent = local_tangent.normalize();

    let local_normal = common::cross_v_s(&local_tangent, 1.0);

    let plane_point = (v11 + v12) * 0.5;

    let tangent = transform1.rotation.apply(&local_tangent);
    let normal = common::cross_v_s(&tangent, 1.0);

    v11 = transform1.apply(&v11);
    v12 = transform1.apply(&v12);

    // Face offset.
    let front_offset = normal.dot(v11);

    // Side offsets, extended by polytope skin thickness.
    let side_offset1 = -tangent.dot(v11) + total_radius;
    let side_offset2 = tangent.dot(v12) + total_radius;

    // Clip incident edge against extruded edge1 side edges.

    // Clip to box side 1.
    let mut clip_points = match ClipVertex::clip_segment_to_line(&incident_edge, &-tangent, side_offset1, index1) {
        Some(clip_points) => clip_points,
        None => return manifold,
    };

    // Clip to negative box side 1.
    let clip_points = match ClipVertex::clip_segment_to_line(&clip_points, &tangent, side_offset2, index2) {
        Some(clip_points) => clip_points,
        None => return manifold,
    };

    manifold.local_normal = local_normal;
    manifold.local_point = plane_point;

    for cp in &clip_points {
        let separation = normal.dot(cp.v) - front_offset;

        if separation <= total_radius {
            use std::mem;

            let mut c_id = cp.id;
            if flip {
                mem::swap(&mut c_id.index_a, &mut c_id.index_b);
                mem::swap(&mut c_id.type_a, &mut c_id.type_b);
            }

            manifold.points.push(ManifoldPoint::new(transform2.apply_t(&cp.v), c_id));
        }
    }

    manifold
}
