use std::f32;
use cgmath::*;
use super::Shape;
use ::common;
use ::common::Transform2d;

/// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
pub struct DistanceProxy {
    buffer: Vec<Vector2<f32>>,
    pub vertices: Vec<Vector2<f32>>,
    pub radius: f32,
}

impl DistanceProxy {
    // TODO: check comment later
    /// Constructs a new proxy using the given shape. The shape
    /// must remain in scope while the proxy is in use.
    pub fn new(shape: &Shape) -> Self {
        match shape {
            &Shape::Polygon(ref polygon) => {
                DistanceProxy {
                    buffer: Vec::with_capacity(2),
                    vertices: polygon.vertices.clone(),
                    radius: shape.get_radius(),
                }
            }
            _ => unimplemented!(),
        }
    }

    /// Get the supporting vertex index in the given direction.
    pub fn get_support(&self, d: &Vector2<f32>) -> usize {
        let mut best_index = 0;
        let mut best_value = dot(self.vertices[0], *d);
        for (i, v) in self.vertices.iter().enumerate().skip(1) {
            let value = dot(*v, *d);
            if value > best_value {
                best_index = i;
                best_value = value;
            }
        }

        best_index
    }

    /// Get the supporting vertex in the given direction.
    pub fn get_support_vertex(&self, d: &Vector2<f32>) -> Vector2<f32> {
        /*let mut best_index = 0;
        let mut best_value = dot(self.vertices[0], *d);
        for (i, v) in self.vertices.iter().enumerate().skip(1) {
            let value = dot(*v, *d);
            if value > best_value {
                best_index = i;
                best_value = value;
            }
        }*/
        let best_index = self.get_support(d);

        self.vertices[best_index]
    }
}

// TODO: maybe change usize to u8 like in Box2D
/// Used to warm start `distance`.
/// Set count to zero on first call.
pub struct SimplexCache {
    /// Length or area.
    metric: f32,
    /// Vertices on shape A and shape B.
    indices: Vec<(usize, usize)>,
}

#[derive(Clone, Copy)]
struct SimplexVertex {
    w_a: Vector2<f32>,
    w_b: Vector2<f32>,
    w: Vector2<f32>,
    a: f32,
    index_a: usize,
    index_b: usize,
}

struct Simplex {
    vertices: Vec<SimplexVertex>,
}

impl Simplex {
    fn new(cache: &SimplexCache, proxy_a: &DistanceProxy, proxy_b: &DistanceProxy,
           transform_a: &Transform2d, transform_b: &Transform2d) -> Self {
        assert!(cache.indices.len() <= 3);

        let mut result = Simplex { vertices: Vec::with_capacity(3) };

        // Copy data from cache.
        for &(i_a, i_b) in &cache.indices {
            let w_a_local = proxy_a.vertices[i_a];
            let w_b_local = proxy_b.vertices[i_b];
            let w_a = transform_a.apply(&w_a_local);
            let w_b = transform_b.apply(&w_b_local);
            result.vertices.push(SimplexVertex {
                w_a: w_a,
                w_b: w_b,
                w: w_b - w_a,
                a: 0.0,
                index_a: i_a,
                index_b: i_b,
            });
        }

        // Compute the new simplex metric, if it is substantially different than
        // old metric then flush the simplex.
        if result.vertices.len() > 1 {
            let metric1 = cache.metric;
            let metric2 = result.get_metric();
            if metric2 < 0.5 * metric1 || 2.0 * metric1 < metric2 || metric2 < f32::EPSILON {
                // Reset the simplex.
                result.vertices.clear();
            }
        }

        // If the cache is empty or invalid ...
        if result.vertices.is_empty() {
            let w_a_local = proxy_a.vertices[0];
            let w_b_local = proxy_b.vertices[0];
            let w_a = transform_a.apply(&w_a_local);
            let w_b = transform_b.apply(&w_b_local);
            result.vertices.push(SimplexVertex {
                w_a: w_a,
                w_b: w_b,
                w: w_b - w_a,
                a: 1.0,
                index_a: 0,
                index_b: 0,
            });
        }

        result
    }

    fn write_cache(&self, cache: &mut SimplexCache) {
        cache.metric = self.get_metric();
        cache.indices.clear();
        for v in &self.vertices {
            cache.indices.push((v.index_a, v.index_b));
        }
    }

    fn get_search_direction(&self) -> Vector2<f32> {
        match self.vertices.len() {
            1 => -self.vertices[0].w,
            2 => {
                let e12 = self.vertices[1].w - self.vertices[0].w;
                let sgn = e12.perp_dot(-self.vertices[0].w);
                if sgn > 0.0 {
                    common::cross_s_v(1.0, &e12)
                } else {
                    common::cross_v_s(&e12, 1.0)
                }
            }
            _ => unreachable!(),
        }
    }

    fn get_closest_point(&self) -> Vector2<f32> {
        match self.vertices.len() {
            1 => self.vertices[0].w,
            2 => self.vertices[0].w * self.vertices[0].a + self.vertices[1].w * self.vertices[1].a,
            3 => Vector2::zero(),
            _ => unreachable!("Invalid number of simplex vertices!"),
        }
    }

    fn get_witness_points(&self) -> (Vector2<f32>, Vector2<f32>) {
        match self.vertices.len() {
            1 => (self.vertices[0].w_a, self.vertices[0].w_b),
            2 => (self.vertices[0].w_a * self.vertices[0].a +
                  self.vertices[1].w_a * self.vertices[1].a,
                  self.vertices[0].w_b * self.vertices[0].a +
                  self.vertices[1].w_b * self.vertices[1].a),
            3 => {
                let result = self.vertices[0].w_a * self.vertices[0].a +
                             self.vertices[1].w_a * self.vertices[1].a +
                             self.vertices[2].w_a * self.vertices[2].a;
                (result, result)
            }
            _ => unreachable!("Invalid number of simplex vertices!"),
        }
    }

    fn get_metric(&self) -> f32 {
        match self.vertices.len() {
            1 => 0.0,
            2 => (self.vertices[1].w - self.vertices[0].w).length(),
            3 => (self.vertices[1].w - self.vertices[0].w).perp_dot(self.vertices[2].w - self.vertices[0].w),
            _ => unreachable!("Invalid number of simplex vertices!"),
        }
    }

    // Solve a line segment using barycentric coordinates.
    //
    // p = a1 * w1 + a2 * w2
    // a1 + a2 = 1
    //
    // The vector from the origin to the closest point on the line is
    // perpendicular to the line.
    // e12 = w2 - w1
    // dot(p, e) = 0
    // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
    //
    // 2-by-2 linear system
    // [1      1     ] * [a1] = [1]
    // [w1.e12 w2.e12]   [a2]   [0]
    //
    // Define
    // d12_1 =  dot(w2, e12)
    // d12_2 = -dot(w1, e12)
    // d12 = d12_1 + d12_2
    //
    // Solution
    // a1 = d12_1 / d12
    // a2 = d12_2 / d12
    fn solve2(&mut self) {
        let w1 = self.vertices[0].w;
        let w2 = self.vertices[1].w;
        let e12 = w2 - w1;

        // w1 region
        let d12_2 = -dot(w1, e12);
        if d12_2 <= 0.0 {
            // a2 <= 0, so we clamp it to 0.
            self.vertices[0].a = 1.0;
            self.vertices.truncate(1);
            return;
        }

        // w2 region
        let d12_1 = dot(w2, e12);
        if d12_1 <= 0.0 {
            // a1 <= 0, so we clamp it to 0.
            self.vertices[1].a = 1.0;
            self.vertices[0] = self.vertices[1];
            self.vertices.truncate(1);
            return;
        }

        // Must be in e12 region.
        let inv_d12 = 1.0 / (d12_1 + d12_2);
        self.vertices[0].a = d12_1 * inv_d12;
        self.vertices[1].a = d12_2 * inv_d12;
        self.vertices.truncate(2);
    }

    // Possible regions:
    // - points[2]
    // - edge points[0]-points[2]
    // - edge points[1]-points[2]
    // - inside the triangle
    fn solve3(&mut self) {
        let w1 = self.vertices[0].w;
        let w2 = self.vertices[1].w;
        let w3 = self.vertices[2].w;

        // Edge12
        // [1      1     ] * [a1] = [1]
        // [w1.e12 w2.e12]   [a2] = [0]
        // a3 = 0
        let e12 = w2 - w1;
        let w1_e12 = dot(w1, e12);
        let w2_e12 = dot(w2, e12);
        let d12_1 = w2_e12;
        let d12_2 = -w1_e12;

        // Edge13
        // [1      1     ] * [a1] = [1]
        // [w1.e13 w3.e13]   [a3] = [0]
        // a2 = 0
        let e13 = w3 - w1;
        let w1_e13 = dot(w1, e13);
        let w3_e13 = dot(w3, e13);
        let d13_1 = w3_e13;
        let d13_2 = -w1_e13;

        // Edge23
        // [1      1     ] * [a2] = [1]
        // [w2.e23 w3.e23]   [a2] = [0]
        // a1 = 0
        let e23 = w3 - w2;
        let w2_e23 = dot(w2, e23);
        let w3_e23 = dot(w3, e23);
        let d23_1 = w3_e23;
        let d23_2 = -w2_e23;

        // Triangle123
        let n123 = e12.perp_dot(e13);

        let d123_1 = w2.perp_dot(w3) * n123;
        let d123_2 = w3.perp_dot(w1) * n123;
        let d123_3 = w1.perp_dot(w2) * n123;

        // w1 region
        if d12_2 <= 0.0 && d13_2 <= 0.0 {
            self.vertices[0].a = 1.0;
            self.vertices.truncate(1);
            return;
        }

        // e12
        if d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0 {
            let inv_d12 = 1.0 / (d12_1 + d12_2);
            self.vertices[0].a = d12_1 * inv_d12;
            self.vertices[1].a = d12_2 * inv_d12;
            self.vertices.truncate(2);
            return;
        }

        // e13
        if d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0 {
            let inv_d13 = 1.0 / (d13_1 + d13_2);
            self.vertices[0].a = d13_1 * inv_d13;
            self.vertices[2].a = d13_2 * inv_d13;
            self.vertices[1] = self.vertices[2];
            self.vertices.truncate(2);
            return;
        }

        // w2 region
        if d12_1 <= 0.0 && d23_2 <= 0.0 {
            self.vertices[1].a = 1.0;
            self.vertices[0] = self.vertices[1];
            self.vertices.truncate(1);
            return;
        }

        // w3 region
        if d13_1 <= 0.0 && d23_1 <= 0.0 {
            self.vertices[2].a = 1.0;
            self.vertices[0] = self.vertices[2];
            self.vertices.truncate(1);
            return;
        }

        // e23
        if d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0 {
            let inv_d23 = 1.0 / (d23_1 + d23_2);
            self.vertices[1].a = d23_1 * inv_d23;
            self.vertices[2].a = d23_2 * inv_d23;
            self.vertices[0] = self.vertices[1];
            self.vertices.truncate(2);
            return;
        }

        // Must be in triangle123.
        let inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
        self.vertices[0].a = d123_1 * inv_d123;
        self.vertices[1].a = d123_2 * inv_d123;
        self.vertices[2].a = d123_3 * inv_d123;
    }
}

/// Output of `distance`
pub struct DistanceOutput {
    /// Closest point on shape A.
    point_a: Vector2<f32>,
    /// Closest point on shape B.
    point_b: Vector2<f32>,
    distance: f32,
    /// Number of GJK iterations used.
    iterations: u32,
}

/// Compute the closest points between two shapes. Supports any combination of:
/// CircleShape, PolygonShape, EdgeShape.
/// On the first call set cache.count to zero.
pub fn distance(cache: &mut SimplexCache, proxy_a: DistanceProxy, proxy_b: DistanceProxy,
                transform_a: Transform2d, transform_b: Transform2d, use_radii: bool) -> DistanceOutput {
    // Initialize the simplex with the cache.
    let mut simplex = Simplex::new(cache, &proxy_a, &proxy_b, &transform_a, &transform_b);

    // These store the vertices of the last simplex so that we
    // can check for duplicates and prevent cycling.
    let mut save = Vec::<(usize, usize)>::with_capacity(3);

    let mut distance_sqr1 = f32::MAX;
    let mut distance_sqr2 = f32::MAX;

    // Main iteration loop.
    let mut iterations = 0;
    for _ in 0..common::MAX_TOI_ITERATIONS {
        // Copy simplex so we can identify duplicates.
        for v in &simplex.vertices {
            save.push((v.index_a, v.index_b));
        }

        match simplex.vertices.len() {
            1 => break,
            2 => {
                simplex.solve2();
                break;
            }
            3 => {
                simplex.solve3();
                break;
            }
            _ => unreachable!("Invalid number of simplex vertices!"),
        }

        // If we have 3 points, then the origin is in the corresponding triangle.
        if simplex.vertices.len() == 3 {
            break;
        }

        // Compute closest point.
        let p = simplex.get_closest_point();
        distance_sqr2 = p.length2();

        // Ensure progress.
        /*if distance_sqr2 >= distance_sqr1 {
            break;
        }*/
        distance_sqr1 = distance_sqr2;

        // Get search direction.
        let d = simplex.get_search_direction();

        // Ensure the search direction is numerically fit.
        if d.length2() < f32::EPSILON * f32::EPSILON {
            // The origin is probably contained by a line segment
            // or triangle. Thus the shapes are overlapped.

            // We can't return zero here even though there may be overlap.
            // In case the simplex is a point, segment, or triangle it is difficult
            // to determine if the origin is contained in the CSO or very close to it.
            break;
        }

        // Compute a tentative new simplex vertex using support points.
        let index_a = proxy_a.get_support(&transform_a.rotation.apply_t(&(-d)));
        let index_b = proxy_b.get_support(&transform_b.rotation.apply_t(&d));
        let w_a = transform_a.apply(&proxy_a.vertices[index_a]);
        let w_b = transform_b.apply(&proxy_b.vertices[index_b]);
        let vertex = SimplexVertex {
            w_a: w_a,
            w_b: w_b,
            w: w_b - w_a,
            a: 0.0,
            index_a: index_a,
            index_b: index_b,
        };

        // Iteration count is equated to the number of support point calls.
        iterations += 1;

        // Check for duplicate support points. This is the main termination criteria.
        let mut duplicate = false;
        for &(index_a, index_b) in &save {
            if vertex.index_a == index_a && vertex.index_b == index_b {
                duplicate = true;
                break;
            }
        }

        // If we found a duplicate support point we must exit to avoid cycling.
        if duplicate {
            break;
        }

        // New vertex is ok and needed.
        simplex.vertices.push(vertex);
    }

    // Prepare output.
    let (mut point_a, mut point_b) = simplex.get_witness_points();
    let mut distance = (point_b - point_a).length();

    // Cache the simplex.
    simplex.write_cache(cache);

    // Apply radii if requested.
    if use_radii {
        let r_a = proxy_a.radius;
        let r_b = proxy_b.radius;

        if distance > r_a + r_b && distance > f32::EPSILON {
            // Shapes are still not overlapped.
            // Move the witness points to the outer surface.
            distance -= r_a + r_b;
            let mut normal = point_b - point_a;
            normal = normal.normalize();
            point_a = point_a + normal * r_a;
            point_b = point_b - normal * r_b;
        } else {
            // Shapes are overlapped when radii are considered.
            // Move the witness points to the middle.
            let p = (point_a + point_b) * 0.5;
            point_a = p;
            point_b = p;
            distance = 0.0;
        }
    }

    DistanceOutput {
        point_a: point_a,
        point_b: point_b,
        distance: distance,
        iterations: iterations,
    }
}
