pub use self::index_pool::IndexPool;
pub use self::debug_draw::DebugDraw;

use cgmath::*;
use std::ops::Mul;

mod index_pool;
pub mod debug_draw;

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

#[derive(Clone, Copy)]
pub struct Rotation2d {
    sin: f32,
    cos: f32,
}

impl Rotation2d {
    /// Constructs a new identity rotation.
    pub fn new() -> Self {
        Rotation2d {
            sin: 0.0,
            cos: 1.0,
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
            x: self.cos * v.x + self.sin + v.y,
            y: -self.sin * v.x + self.cos * v.y,
        }
    }
}

/// A transform contains translation and rotation. It is used to represent the position
/// and orientation of rigid frames.
#[derive(Clone, Copy)]
pub struct Transform2d {
    pub position: Vector2<f32>,
    pub rotation: Rotation2d,
}

impl Transform2d {
    /// Constructs a new identity transform.
    pub fn new() -> Self {
        Transform2d {
            position: Vector2::<f32>::new(0.0, 0.0),
            rotation: Rotation2d::new(),
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
