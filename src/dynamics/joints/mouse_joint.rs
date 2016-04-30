use cgmath::*;
use ::dynamics::island::{Position, Velocity};
use ::dynamics::world::TimeStep;
use ::common;
use ::common::Rotation2d;
use super::JointData;
use std::f32;

pub struct MouseJointConfig {
    /// The initial world target point. This is assumed to coincide with the body anchor initially.
    target: Vector2<f32>,

    /// The maximum constraint force that can be exerted to move the candidate body. Usually
    /// you will express as some multiple of the weight (multiplier * mass * gravity).
    max_force: f32,

    /// The response speed.
    frequency_hz: f32,

    /// The damping ratio. 0 = no damping, 1 = critical damping.
    damping_ratio: f32,
}

impl Default for MouseJointConfig {
    fn default() -> MouseJointConfig {
        MouseJointConfig {
            target: Vector2::zero(),
            max_force: 0.0,
            frequency_hz: 5.0,
            damping_ratio: 0.7,
        }
    }
}

/// A mouse joint is used to make a point on a body track a specific world point. This is a soft
/// constraint with a maximum force. This allows the constraint to stretch and without applying
/// a huge force.
pub struct MouseJoint<'a> {
    pub joint_data: JointData<'a>,

    local_anchor_b: Vector2<f32>,
    target_a: Vector2<f32>,
    frequency_hz: f32,
    damping_ratio: f32,
    beta: f32,

    // Solver shared.
    impulse: Vector2<f32>,
    max_force: f32,
    gamma: f32,

    // Solver temp.
    index_a: usize,
    index_b: usize,
    r_b: Vector2<f32>,
    local_center_b: Vector2<f32>,
    inv_mass_b: f32,
    inv_inertia_b: f32,
    mass: Matrix2<f32>,
    c: Vector2<f32>,
}

impl<'a> MouseJoint<'a> {
    pub fn new(joint_config: &MouseJointConfig, joint_data: JointData<'a>) -> MouseJoint<'a> {
        let body_b = joint_data.body_b.upgrade().unwrap();
        let body_b = body_b.borrow();
        let target_a = joint_config.target;

        MouseJoint {
            joint_data: joint_data,
            local_anchor_b: body_b.get_transform().apply(&target_a),
            target_a: target_a,
            frequency_hz: joint_config.frequency_hz,
            damping_ratio: joint_config.damping_ratio,
            beta: 0.0,

            impulse: Vector2::zero(),
            max_force: joint_config.max_force,
            gamma: 0.0,

            index_a: 0,
            index_b: 0,
            r_b: Vector2::zero(),
            local_center_b: Vector2::zero(),
            inv_mass_b: 0.0,
            inv_inertia_b: 0.0,
            mass: Matrix2::zero(),
            c: Vector2::zero(),
        }
    }

    pub fn initialize_velocity_constraints(&mut self, step: TimeStep, positions: &Vec<Position>, velocities: &mut Vec<Velocity>) {
        let body_b = self.joint_data.body_b.upgrade().unwrap();
        self.index_b = body_b.borrow().island_index;
        self.local_center_b = body_b.borrow().sweep.local_center;
        self.inv_mass_b = body_b.borrow().inv_mass;
        self.inv_inertia_b = body_b.borrow().inv_inertia;

        let c_b = positions[self.index_b].c;
        let a_b = positions[self.index_b].a;
        let mut v_b = velocities[self.index_b].v;
        let mut w_b = velocities[self.index_b].w;

        let q_b = Rotation2d::new(a_b);

        let mass = body_b.borrow().get_mass();

        // Frequency.
        let omega = 2.0 * f32::consts::PI * self.frequency_hz;

        // Damping coefficient.
        let d = 2.0 * mass * self.damping_ratio * omega;

        // Spring stiffness.
        let k = mass * (omega * omega);

        // Magic formulas.
        // Gamma has units of inverse mass.
        // Beta has units of inverse time.
        let dt = step.dt;
        assert!(d + dt * k > f32::EPSILON);
        self.gamma = dt * (d + dt * k);
        if self.gamma != 0.0 {
            self.gamma = 1.0 / self.gamma;
        }
        self.beta = dt * k * self.gamma;

        // Compute the effective mass matrix.
        self.r_b = q_b.apply(&(self.local_anchor_b - self.local_center_b));

        // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    	//   = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    	//     [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        let k = Matrix2::<f32> {
            x: Vector2 {
                x: self.inv_mass_b + self.inv_inertia_b * self.r_b.y * self.r_b.y + self.gamma,
                y: -self.inv_inertia_b * self.r_b.x * self.r_b.y,
            },
            y: Vector2 {
                x: -self.inv_inertia_b * self.r_b.x * self.r_b.y,
                y: self.inv_mass_b + self.inv_inertia_b * self.r_b.x * self.r_b.x + self.gamma,
            },
        };

        self.mass = k.invert().unwrap();

        self.c = c_b + self.r_b - self.target_a;
        self.c *= self.beta;

        // Cheat with some damping.
        w_b *= 0.98;

        if step.warm_starting {
            self.impulse *= step.dt_ratio;
            v_b += self.impulse * self.inv_mass_b;
            w_b += self.inv_inertia_b * self.r_b.perp_dot(self.impulse);
        } else {
            self.impulse = Vector2::zero();
        }

        velocities[self.index_b].v = v_b;
        velocities[self.index_b].w = w_b;
    }

    pub fn solve_velocity_constraints(&mut self, step: TimeStep, velocities: &mut Vec<Velocity>) {
        let mut v_b = velocities[self.index_b].v;
        let mut w_b = velocities[self.index_b].w;

        // c_dot = v + cross(w, r)
        let c_dot = v_b + common::cross_s_v(w_b, &self.r_b);
        let mut impulse = self.mass * -(c_dot + self.c + self.gamma * self.impulse);

        let old_impulse = self.impulse;
        self.impulse += impulse;
        let max_impulse = step.dt * self.max_force;
        if self.impulse.magnitude2() > max_impulse * max_impulse {
            self.impulse *= max_impulse / self.impulse.magnitude();
        }
        impulse = self.impulse - old_impulse;

        v_b += self.inv_mass_b * impulse;
        w_b += self.inv_inertia_b * self.r_b.perp_dot(impulse);

        velocities[self.index_b].v = v_b;
        velocities[self.index_b].w = w_b;
    }
}
