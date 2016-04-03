use super::DistanceProxy;
use ::common;
use ::common::Sweep;

pub enum State {
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
pub fn time_of_impact(proxy_a: DistanceProxy, proxy_b: DistanceProxy,
                      mut sweep_a: Sweep, mut sweep_b: Sweep, t_max: f32) -> (State, f32) {
    // CCD via the local separating axis method. This seeks progression
    // by computing the largest time at which separation is maintained.

    let mut state = State::Unknown;
    let mut t = t_max;

    // Large rotations can make the root finder fail, so we normalize the sweep angle.
    sweep_a.normalize();
    sweep_b.normalize();

    let total_radius = proxy_a.radius + proxy_b.radius;
    let target = f32::max(common::LINEAR_SLOP, total_radius - 3.0 * common::LINEAR_SLOP);
    let tolerance = 0.25 * common::LINEAR_SLOP;
    assert!(target > tolerance);

    let t1 = 0.0;

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    loop {

    }

    (state, t)
}
