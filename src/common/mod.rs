pub use self::index_pool::IndexPool;
pub use self::debug_draw::DebugDraw;
pub use self::timer::Timer;

use cgmath::*;
use std::ops::Mul;
use std::f32;

mod index_pool;
mod timer;
pub mod debug_draw;

//
// Global tuning constants based on meters-kilograms-seconds (MKS) units.
//

// Collision

/// The maximum number of vertices on a convex polygon.
pub const MAX_POLYGON_VERTICES: usize = 8;

/// This is used to fatten Aabbs in the dynamic tree. This allows proxies to move by a small
/// amount without triggering a tree adjustment. This is in meters
pub const AABB_EXTENSION: f32 = 0.1;

/// A small length used as a collision and constraint tolerance. Usually it is chosen to be
/// numerically significant, but visually insignificant.
pub const LINEAR_SLOP: f32 = 0.005;

/// The radius of the polygon/edge shape skin. This should not be modified. Making this smaller
/// means polygons will have an insufficient buffer for continues collision. Making it larger
/// may create artifacts for vertex collision.
pub const POLYGON_RADIUS: f32 = 2.0 * LINEAR_SLOP;

/// Maximum number of sub-steps per contact in continuous physics simulation.
pub const MAX_SUB_STEPS: u32 = 8;

// Dynamics

/// Maximum number of iterations per TOI impact.
pub const MAX_TOI_ITERATIONS: usize = 20;

/// Maximum number of contacts to be handled to solve a TOI impact.
pub const MAX_TOI_CONTACTS: usize = 32;

/// A velocity threshold for elastic collisions. Any collision with a relative linear velocity
/// below this threshold will be treated as inelasti
pub const VELOCITY_THRESHOLD: f32 = 1.0;

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
pub const MAX_LINEAR_CORRECTION: f32 = 0.2;

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
pub const MAX_TRANSLATION: f32 = 2.0;
pub const MAX_TRANSLATION_SQUARED: f32 = MAX_TRANSLATION * MAX_TRANSLATION;

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
pub const MAX_ROTATION: f32 = 0.5 * f32::consts::PI;
pub const MAX_ROTATION_SQUARED: f32 = MAX_ROTATION * MAX_ROTATION;

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However, using values close to 1 often lead
/// to overshoot.
pub const BAUMGARTE: f32 = 0.2;
pub const TOI_BAUMGARTE: f32 = 0.75;

/// Performs the cross product on a vector and a scalar. In 2D this produces a vector.
pub fn cross_v_s(v: &Vector2<f32>, s: f32) -> Vector2<f32> {
    Vector2::<f32> {
        x: s * v.y,
        y: -s * v.x,
    }
}

/// Performs the cross product on a scalar and a vector. In 2D this produces a vector.
pub fn cross_s_v(s: f32, v: &Vector2<f32>) -> Vector2<f32> {
    Vector2::<f32> {
        x: -s * v.y,
        y: s * v.x,
    }
}

pub fn clamp_f32(s: f32, low: f32, high: f32) -> f32 {
    f32::max(low, f32::min(s, high))
}

#[derive(Clone, Copy, Debug)]
pub struct Rotation2d {
    sin: f32,
    cos: f32,
}

impl Default for Rotation2d {
    /// Constructs a new identity rotation.
    fn default() -> Rotation2d {
        Rotation2d {
            sin: 0.0,
            cos: 1.0,
        }
    }
}

impl Rotation2d {
    /// Constructs a new rotation from an angle.
    pub fn new(angle: f32) -> Self {
        Rotation2d {
            sin: angle.sin(),
            cos: angle.cos(),
        }
    }

    /// Sets the rotation from an angle.
    pub fn set_angle(&mut self, angle: f32) {
        self.sin = angle.sin();
        self.cos = angle.cos();
    }

    /// Returns the angle in radians.
    pub fn get_angle(&self) -> f32 {
        f32::atan2(self.sin, self.cos)
    }

    /// Multiplies this rotation with the supplied one.
    pub fn mul(&self, rhs: &Rotation2d) -> Rotation2d {
        // q = self, r = rhs
        // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
    	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
        Rotation2d {
            sin: self.sin * rhs.cos + self.cos * rhs.sin,
            cos: self.cos * rhs.cos - self.sin * rhs.sin,
        }
    }

    /// Multiplies the transpose of this rotation with the supplied one
    pub fn mul_t(&self, rhs: &Rotation2d) -> Rotation2d {
        // q = self, r = rhs
        // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
    	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
        Rotation2d {
            sin: self.cos * rhs.sin - self.sin * rhs.cos,
            cos: self.cos * rhs.cos + self.sin * rhs.sin,
        }
    }

    /// Rotates a vector
    pub fn apply(&self, v: &Vector2<f32>) -> Vector2<f32> {
        // q = self
        // [qc -qs] * [x] = [qc*x - qs*y]
    	// [qs  qc]   [y]   [qs*x + qc*y]
        Vector2::<f32> {
            x: self.cos * v.x - self.sin * v.y,
            y: self.sin * v.x + self.cos * v.y,
        }
    }

    /// Inverse rotates a vector
    pub fn apply_t(&self, v: &Vector2<f32>) -> Vector2<f32> {
        // q = self
        // [ qc qs] * [x] = [qc*x + qs*y]
    	// [-qs qc]   [y]   [qs*x + qc*y]
        Vector2::<f32> {
            x: self.cos * v.x + self.sin * v.y,
            y: -self.sin * v.x + self.cos * v.y,
        }
    }
}

/// A transform contains translation and rotation. It is used to represent the position
/// and orientation of rigid frames.
#[derive(Clone, Copy, Debug)]
pub struct Transform2d {
    pub position: Vector2<f32>,
    pub rotation: Rotation2d,
}

impl Default for Transform2d {
    /// Constructs a new identity transform.
    fn default() -> Transform2d {
        Transform2d {
            position: Vector2::zero(),
            rotation: Default::default(),
        }
    }
}

impl Transform2d {
    /// Constructs a new transform with the given position and rotation.
    pub fn new(position: Vector2<f32>, rotation: Rotation2d) -> Self {
        Transform2d {
            position: position,
            rotation: rotation,
        }
    }

    pub fn mul(&self, rhs: &Transform2d) -> Transform2d {
        Transform2d {
            position: self.rotation.apply(&rhs.position) + self.position,
            rotation: self.rotation.mul(&rhs.rotation),
        }
    }

    pub fn mul_t(&self, rhs: &Transform2d) -> Transform2d {
        Transform2d {
            position: self.rotation.apply_t(&(rhs.position - self.position)),
            rotation: self.rotation.mul_t(&rhs.rotation),
        }
    }

    pub fn apply(&self, v: &Vector2<f32>) -> Vector2<f32> {
        Vector2::<f32> {
            x: self.rotation.cos * v.x - self.rotation.sin * v.y + self.position.x,
            y: self.rotation.sin * v.x + self.rotation.cos * v.y + self.position.y,
        }
    }

    pub fn apply_t(&self, v: &Vector2<f32>) -> Vector2<f32> {
        let p = v - self.position;
        Vector2::<f32> {
            x: self.rotation.cos * p.x + self.rotation.sin * p.y,
            y: -self.rotation.sin * p.x + self.rotation.cos * p.y,
        }
    }
}

/// This describes the motion of a body/shape for TOI computation. Shapes are defined
/// with respect to the body origin, which may not coincide with the center of mass.
/// However, to support dynamics we must interpolate the center of mass position.
#[derive(Clone, Copy)]
pub struct Sweep {
    /// Local center of mass position
    pub local_center: Vector2<f32>,
    /// center world position at `alpha0`
    pub c0: Vector2<f32>,
    /// center world position
    pub c: Vector2<f32>,
    /// world angle at `alpha0`
    pub a0: f32,
    /// world angle
    pub a: f32,
    /// Fraction of the current time step in the range [0, 1]
    pub alpha0: f32,
}

impl Default for Sweep {
    fn default() -> Sweep {
        Sweep {
            local_center: Vector2::zero(),
            c0: Vector2::zero(),
            c: Vector2::zero(),
            a0: 0.0,
            a: 0.0,
            alpha0: 0.0,
        }
    }
}

impl Sweep {
    /// Get the interpolated transform at a specific time. `beta` is a factor in [0, 1],
    /// where 0 indicates `alpha0`
    pub fn get_transform(&self, beta: f32) -> Transform2d {
        let mut result = Transform2d::new(
            self.c0 * (1.0 - beta) + self.c * beta,
            Rotation2d::new(self.a0 * (1.0 - beta) + self.a * beta),
        );

        /*let mut result = Transform2d::default();
        result.position = self.c0 * (1.0 - beta) + self.c * beta;
        result.rotation.set_angle(self.a0 * (1.0 - beta) + self.a * beta);*/

        // Shift to origin.
        result.position -= result.rotation.apply(&self.local_center);
        result
    }

    /// Advance the sweep forward, yielding a new initial state. `alpha` is the new
    /// initial time.
    pub fn advance(&mut self, alpha: f32) {
        assert!(self.alpha0 < 1.0);
        let beta = (alpha - self.alpha0) / (1.0 - self.alpha0);
        self.c0 += (self.c - self.c0) * beta;
        self.a0 += (self.a - self.a0) * beta;
        self.alpha0 = alpha;
    }

    /// Normalize the angles.
    pub fn normalize(&mut self) {
        let two_pi = 2.0 * f32::consts::PI;
        let d = two_pi * (self.a0 / two_pi).floor();
        self.a0 -= d;
        self.a -= d;
    }
}
