use std::f32;
use cgmath::*;
use ::common;
use ::common::{Transform2d};
use super::Aabb;

/// A convex polygon. It is assumed that the interior of the polygon is to the left of each edge.
pub struct PolygonShape {
    pub centroid: Vector2<f32>,
    pub vertices: Vec<Vector2<f32>>,
    pub normals: Vec<Vector2<f32>>,
}

impl PolygonShape {
    pub fn new() -> PolygonShape {
        PolygonShape {
            centroid: Vector2::zero(),
            vertices: Vec::new(),
            normals: Vec::new(),
        }
    }

    fn compute_centroid(&mut self) {
        self.centroid = Vector2::zero();
        let mut area = 0.0;

        // `p_ref` is the reference point for forming triangles.
        // Its location doesn't change the result (except for rounding error).
        let p_ref = Vector2::<f32>::zero();
        // This code would put the reference point inside the polygon.
        // for v in &self.vertices {
        //     p_ref = p_ref + v;
        // }
        // p_ref = p_ref * 1.0 / self.vertices.len() as f32;

        let inv3 = 1.0 / 3.0;
        for i in 0..self.vertices.len() {
            // Triangle vertices. (p_ref would be p0)
            let p1 = self.vertices[i];
            let p2 = if i + 1 < self.vertices.len() {
                self.vertices[i + 1]
            } else {
                self.vertices[0]
            };

            let e1 = p1 - p_ref;
            let e2 = p2 - p_ref;

            let triangle_area = 0.5 * e1.perp_dot(e2);
            area += triangle_area;

            // Area weighted centroid.
            self.centroid = self.centroid + (p_ref + p1 + p2) * triangle_area * inv3;
        }
        self.centroid = self.centroid * 1.0 / area;
    }

    /// Create a convex hull from the given vertices.
    pub fn set(&mut self, vertices: &Vec<Vector2<f32>>) {
        if vertices.len() < 3 {
            return;
        }

        let mut vertices_tmp = Vec::<Vector2<f32>>::new();

        // Perform welding and copy vertices into local buffer.
        'outer: for v in vertices {
            for v_tmp in &vertices_tmp {
                if (v - v_tmp).magnitude2() < (0.5 * common::LINEAR_SLOP) * (0.5 * common::LINEAR_SLOP) {
                    continue 'outer;
                }
            }
            vertices_tmp.push(*v);
        }

        if vertices_tmp.len() < 3 {
            return;
        }

        // Create the convex hull using the Gift wrapping algorithm.
        // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

        // Find the rightmost point on the hull.
        let mut i0 = 0;
        //let mut x0 = vertices_tmp.first().unwrap().x;
        //let mut y0 = vertices_tmp.first().unwrap().x;
        for (i, v) in vertices_tmp.iter().enumerate().skip(1) {
            if v.x > vertices_tmp[i0].x || (v.x == vertices_tmp[i0].x && v.y < vertices_tmp[i0].y) {
                i0 = i;
            }
        }

        let mut hull = Vec::<usize>::new();
        let mut ih = i0;

        loop {
            hull.push(ih);

            let mut ie = 0;
            if ie == ih {
                ie = 1;
            }

            for (i, _) in vertices_tmp.iter().enumerate().skip(1) {
                let r = vertices_tmp[ie] - vertices_tmp[*hull.last().unwrap()];
                let v = vertices_tmp[i] - vertices_tmp[*hull.last().unwrap()];
                let c = r.perp_dot(v);
                if c < 0.0 {
                    ie = i;
                }

                // Collinearity check.
                if c == 0.0 && v.magnitude2() > r.magnitude2() {
                    ie = i;
                }
            }

            ih = ie;
            if ie == i0 {
                break;
            }
        }

        if hull.len() < 3 {
            // Polygon is degenerate.
            return;
        }

        // Copy vertices.
        for i in &hull {
            self.vertices.push(vertices_tmp[*i]);
        }

        // Compute normals. Ensure the edges have non-zero length.
        for i in 0..self.vertices.len() {
            let i2 = if i + 1 < self.vertices.len() {
                i + 1
            } else {
                0
            };
            let edge = self.vertices[i2] - self.vertices[i];
            assert!(edge.magnitude2() > f32::EPSILON * f32::EPSILON);
            let mut normal = common::cross_v_s(&edge, 1.0);
            normal = normal.normalize();
            self.normals.push(normal);
        }

        self.compute_centroid();

        for (i, v) in self.vertices.iter().enumerate() {
            println!("{} {:?}", i, v);
        }
        println!("");
    }

    /// Build vertices to represent an axis-aligned box centered on the local origin.
    pub fn set_as_box(&mut self, half_width: f32, half_height: f32) {
        self.vertices.clear();
        self.vertices.push(vec2(-half_width, -half_height));
        self.vertices.push(vec2(half_width, -half_height));
        self.vertices.push(vec2(half_width, half_height));
        self.vertices.push(vec2(-half_width, half_height));

        self.normals.clear();
        self.normals.push(vec2(0.0, -1.0));
        self.normals.push(vec2(1.0, 0.0));
        self.normals.push(vec2(0.0, 1.0));
        self.normals.push(vec2(-1.0, 0.0));

        self.centroid = Vector2::zero();
    }

    pub fn compute_aabb(&self, transform: &Transform2d) -> Aabb {
        let mut min = transform.apply(&self.vertices[0]);
        let mut max = min;
        for v in &self.vertices {
            let v = transform.apply(v);
            min.x = f32::min(min.x, v.x);
            min.y = f32::min(min.y, v.y);
            max.x = f32::max(max.x, v.x);
            max.y = f32::max(max.y, v.y);
        }
        let r = Vector2::<f32>::new(common::POLYGON_RADIUS, common::POLYGON_RADIUS);
        Aabb {
            min: min - r,
            max: max + r,
        }
    }

    pub fn compute_mass(&self, density: f32) -> (f32, Vector2<f32>, f32) {
        // From Box2D:
        //
        // Polygon mass, centroid, and inertia.
        // Let rho be the polygon density in mass per unit area.
        // Then:
        // mass = rho * int(dA)
        // centroid.x = (1/mass) * rho * int(x * dA)
        // centroid.y = (1/mass) * rho * int(y * dA)
        // I = rho * int((x*x + y*y) * dA)
        //
        // We can compute these integrals by summing all the integrals
        // for each triangle of the polygon. To evaluate the integral
        // for a single triangle, we make a change of variables to
        // the (u,v) coordinates of the triangle:
        // x = x0 + e1x * u + e2x * v
        // y = y0 + e1y * u + e2x * v
        // where 0 <= u && 0 <= v && u + v <= 1
        //
        // We integrate u from [0,1-v] and then v from [0,1].
        // We also need to use the Jacobian of the transformation:
        // D = cross(e1, e2)
        //
        // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
        //
        // The rest of the derivation is handled by computer algebra.

        assert!(self.vertices.len() >= 3);

        let mut center = Vector2::<f32>::zero();
        let mut area = 0.0;
        let mut inertia = 0.0;

        // `p_ref` is the reference point for forming triangles.
        // Its location doesn't change the result (except for rounding error).
        let mut p_ref = Vector2::<f32>::zero();

        // This code would put the reference point inside the polygon.
        for v in &self.vertices {
            p_ref = p_ref + v;
        }
        p_ref = p_ref * 1.0 / self.vertices.len() as f32;

        let inv3 = 1.0 / 3.0;
        for i in 0..self.vertices.len() {
            // Triangle vertices.
            let p1 = self.vertices[i];
            let p2 = if i + 1 < self.vertices.len() {
                self.vertices[i + 1]
            } else {
                self.vertices[0]
            };

            let e1 = p1 - p_ref;
            let e2 = p2 - p_ref;

            let d = e1.perp_dot(e2);

            let triangle_area = 0.5 * d;
            area += triangle_area;

            // Area weighted centroid.
            center = center + (e1 + e2) * triangle_area * inv3;
            //self.centroid = self.centroid + (p_ref + p1 + p2) * triangle_area * inv3;

            let ex1 = e1.x;
            let ey1 = e1.y;
            let ex2 = e2.x;
            let ey2 = e2.y;

            let intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
            let inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

            inertia += (0.25 * inv3 * d) * (intx2 + inty2);
        }
        //self.centroid = self.centroid * 1.0 / area;

        // Total mass.
        let mass = density * area;

        // Center of mass.
        assert!(area > f32::EPSILON);
        center = center * 1.0 / area;
        let center_result = center + p_ref;

        // Inertia tensor relative to the local origin (p_ref).
        inertia = density * inertia;

        // Shift to center of mass then to original body origin.
        inertia = inertia + mass * (dot(center_result, center_result) - dot(center, center));

        (mass, center_result, inertia)
    }
}
