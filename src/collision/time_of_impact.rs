use cgmath::*;
use ::collision;
use super::{DistanceProxy, SimplexCache};
use ::common;
use ::common::Sweep;

enum SeparationType {
    Points,
    FaceA,
    FaceB,
}

struct SeparationFunction {
    separation_type: SeparationType,
    local_point: Vector2<f32>,
    axis: Vector2<f32>,
}

impl SeparationFunction {
    fn new(cache: &SimplexCache, proxy_a: &DistanceProxy, proxy_b: &DistanceProxy,
           sweep_a: &Sweep, sweep_b: &Sweep, t1: f32) -> Self {
        assert!(0 < cache.indices.len() && cache.indices.len() < 3);

        let transform_a = sweep_a.get_transform(t1);
        let transform_b = sweep_b.get_transform(t1);

        let mut separation_type;
        let mut local_point = Vector2::<f32>::zero();
        let mut axis;

        if cache.indices.len() == 1 {
            separation_type = SeparationType::Points;
            let local_point_a = proxy_a.vertices[cache.indices[0].0];
            let local_point_b = proxy_b.vertices[cache.indices[0].1];
            let point_a = transform_a.apply(&local_point_a);
            let point_b = transform_b.apply(&local_point_b);
            axis = point_b - point_a;
            axis = axis.normalize();
        } else if cache.indices[0].0 == cache.indices[1].0 {
            // Two points on B and one on A.
            separation_type = SeparationType::FaceB;
            let local_point_b1 = proxy_b.vertices[cache.indices[0].1];
            let local_point_b2 = proxy_b.vertices[cache.indices[1].1];

            axis = common::cross_v_s(&(local_point_b2 - local_point_b1), 1.0);
            axis = axis.normalize();
            let normal = transform_b.rotation.apply(&axis);

            local_point = (local_point_b1 + local_point_b2) * 0.5;
            let point_b = transform_b.apply(&local_point);

            let local_point_a = proxy_a.vertices[cache.indices[0].0];
            let point_a = transform_a.apply(&local_point_a);

            let mut s = dot(point_a - point_b, normal);
            if s < 0.0 {
                axis = -axis;
                s = -s;
            }
        } else {
            // Two points on A and one or two points on B.
            separation_type = SeparationType::FaceA;
            let local_point_a1 = proxy_a.vertices[cache.indices[0].0];
            let local_point_a2 = proxy_a.vertices[cache.indices[1].0];

            axis = common::cross_v_s(&(local_point_a2 - local_point_a1), 1.0);
            axis = axis.normalize();
            let normal = transform_a.rotation.apply(&axis);

            local_point = (local_point_a1 + local_point_a2) * 0.5;
            let point_a = transform_a.apply(&local_point);

            let local_point_b = proxy_b.vertices[cache.indices[0].1];
            let point_b = transform_b.apply(&local_point_b);

            let mut s = dot(point_b - point_a, normal);
            if s < 0.0 {
                axis = -axis;
                s = -s;
            }
        }

        SeparationFunction {
            separation_type: separation_type,
            local_point: local_point,
            axis: axis,
        }
    }

    fn find_min_separation(&self, proxy_a: &DistanceProxy, proxy_b: &DistanceProxy,
                           sweep_a: &Sweep, sweep_b: &Sweep, t: f32) -> (usize, usize, f32) {
        let transform_a = sweep_a.get_transform(t);
        let transform_b = sweep_b.get_transform(t);

        let mut index_a;
        let mut index_b;
        let mut separation;

        match self.separation_type {
            SeparationType::Points => {
                let axis_a = transform_a.rotation.apply_t(&self.axis);
                let axis_b = transform_b.rotation.apply_t(&(-self.axis));

                index_a = proxy_a.get_support(&axis_a);
                index_b = proxy_b.get_support(&axis_b);

                let local_point_a = proxy_a.vertices[index_a];
                let local_point_b = proxy_b.vertices[index_b];

                let point_a = transform_a.apply(&local_point_a);
                let point_b = transform_b.apply(&local_point_b);

                separation = dot(point_b - point_a, self.axis);
            }

            SeparationType::FaceA => {
                let normal = transform_a.rotation.apply(&self.axis);
                let point_a = transform_a.apply(&self.local_point);

                let axis_b = transform_b.rotation.apply_t(&(-normal));

                // Won't be used.
                index_a = 0;
                index_b = proxy_b.get_support(&axis_b);

                let local_point_b = proxy_b.vertices[index_b];
                let point_b = transform_b.apply(&local_point_b);

                separation = dot(point_b - point_a, normal);
            }

            SeparationType::FaceB => {
                let normal = transform_b.rotation.apply(&self.axis);
                let point_b = transform_b.apply(&self.local_point);

                let axis_a = transform_a.rotation.apply_t(&(-normal));

                // Won't be used.
                index_b = 0;
                index_a = proxy_a.get_support(&axis_a);

                let local_point_a = proxy_a.vertices[index_a];
                let point_a = transform_a.apply(&local_point_a);

                separation = dot(point_a - point_b, normal);
            }
        }

        (index_a, index_b, separation)
    }

    fn evaluate(&self, proxy_a: &DistanceProxy, proxy_b: &DistanceProxy,
                sweep_a: &Sweep, sweep_b: &Sweep, index_a: usize, index_b: usize, t: f32) -> f32 {
        let transform_a = sweep_a.get_transform(t);
        let transform_b = sweep_b.get_transform(t);

        match self.separation_type {
            SeparationType::Points => {
                let local_point_a = proxy_a.vertices[index_a];
                let local_point_b = proxy_b.vertices[index_b];

                let point_a = transform_a.apply(&local_point_a);
                let point_b = transform_b.apply(&local_point_b);

                return dot(point_b - point_a, self.axis);
            }

            SeparationType::FaceA => {
                let normal = transform_a.rotation.apply(&self.axis);
                let point_a = transform_a.apply(&self.local_point);

                let local_point_b = proxy_b.vertices[index_b];
                let point_b = transform_b.apply(&local_point_b);

                return dot(point_b - point_a, normal);
            }

            SeparationType::FaceB => {
                let normal = transform_b.rotation.apply(&self.axis);
                let point_b = transform_b.apply(&self.local_point);

                let local_point_a = proxy_a.vertices[index_a];
                let point_a = transform_a.apply(&local_point_a);

                return dot(point_a - point_b, normal);
            }
        }
    }
}

pub enum ToiState {
    Unknown,
    Failed,
    Overlapped,
    Touching,
    Separated,
}

/// Compute the upper bound on time before two shapes penetrate. Time is represented as a
/// fraction between [0, tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collision. If you change the time interval, you should call this function
/// again.
/// Note: Use `distance` to compute the contact point and normal at the time of impact.
pub fn time_of_impact(proxy_a: &DistanceProxy, proxy_b: &DistanceProxy,
                      mut sweep_a: Sweep, mut sweep_b: Sweep, t_max: f32) -> (ToiState, f32) {
    // CCD via the local separating axis method. This seeks progression
    // by computing the largest time at which separation is maintained.

    let mut state = ToiState::Unknown;
    let mut t = t_max;

    // Large rotations can make the root finder fail, so we normalize the sweep angle.
    sweep_a.normalize();
    sweep_b.normalize();

    let total_radius = proxy_a.radius + proxy_b.radius;
    let target = f32::max(common::LINEAR_SLOP, total_radius - 3.0 * common::LINEAR_SLOP);
    let tolerance = 0.25 * common::LINEAR_SLOP;
    assert!(target > tolerance);

    let mut t1 = 0.0;

    // Prepare input for distance query.
    let mut cache = SimplexCache::new();

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for i in 0..common::MAX_TOI_ITERATIONS {
        let transform_a = sweep_a.get_transform(t1);
        let transform_b = sweep_b.get_transform(t1);

        // Get the distance between shapes. We can also use the results
        // to get a separating axis.
        let distance_output = collision::distance(&mut cache, proxy_a, proxy_b, &transform_a, &transform_b, false);

        // If the shapes are overlapped, we give up on continuous collision.
        if distance_output.distance <= 0.0 {
            // Failure!
            state = ToiState::Overlapped;
            t = 0.0;
            break;
        }

        if distance_output.distance < target + tolerance {
            // Victory!
            state = ToiState::Touching;
            t = t1;
            break;
        }

        // Initialize the separating axis.
        let fcn = SeparationFunction::new(&cache, proxy_a, proxy_b, &sweep_a, &sweep_b, t1);

        // Dump the curve seen by the root finder.
        /*{
            let n = 100;
            let dx = 1.0 / n as f32;
            let mut xs = Vec::<f32>::with_capacity(n + 1);
            let mut fs = Vec::<f32>::with_capacity(n + 1);

            let mut x = 0.0;

            let mut transform_a;
            let mut transform_b;

            for i in 0..n {
                transform_a = sweep_a.get_transform(x);
                transform_b = sweep_b.get_transform(x);
                // TODO: fix
                let f = fcn.evaluate(transform_a, transform_b) - target;

                println!("{} {}", x, f);

                xs.push(x);
                fs.push(f);

                x += dx;
            }
        }*/

        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        let mut done = false;
        let mut t2 = t_max;
        for _ in 0..common::MAX_POLYGON_VERTICES {
            // Find the deepest point at t2. Store the witness point indices.
            let (index_a, index_b, mut s2) = fcn.find_min_separation(proxy_a, proxy_b, &sweep_a, &sweep_b, t2);

            // Is the final configuration separated?
            if s2 > target + tolerance {
                // Victory!
                state = ToiState::Separated;
                t = t_max;
                done = true;
                break;
            }

            // Has the separation reached tolerance?
            if s2 > target - tolerance {
                // Advance the sweeps.
                t1 = t2;
                break;
            }

            // Compute the initial separation of the witness points.
            let mut s1 = fcn.evaluate(proxy_a, proxy_b, &sweep_a, &sweep_b, index_a, index_b, t1);

            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            if s1 < target - tolerance {
                state = ToiState::Failed;
                t = t1;
                done = true;
                break;
            }

            // Check for touching.
            if s1 <= target + tolerance {
                // Victory! t1 should hold the TOI (could be 0.0).
                state = ToiState::Touching;
                t = t1;
                done = true;
                break;
            }

            // Compute 1D root of: f(x) - target = 0
            let mut root_iter_count = 0;
            let mut a1 = t1;
            let mut a2 = t2;
            for _ in 0..50 {
                // Use a mix of the secant rule and bisection.
                let t;
                if root_iter_count & 1 == 1 {
                    // Secant rule to improve convergence.
                    t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                } else {
                    // Bisection to guarantee progress.
                    t = 0.5 * (a1 + a2);
                }

                root_iter_count += 1;

                let s = fcn.evaluate(proxy_a, proxy_b, &sweep_a, &sweep_b, index_a, index_b, t);

                if (s - target).abs() < tolerance {
                    // t2 holds a tentative value for t1.
                    t2 = t;
                    break;
                }

                // Ensure we continue to bracket the root.
                if s > target {
                    a1 = t;
                    s1 = s;
                } else {
                    a2 = t;
                    s2 = s;
                }
            }
        }

        if done {
            break;
        }

        if i == common::MAX_TOI_ITERATIONS - 1 {
            // Root finder got stuck. Semi-victory.
            state = ToiState::Failed;
            t = t1;
            break;
        }
    }

    (state, t)
}
