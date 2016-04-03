use ::collision::{ManifoldType, WorldManifold};
use ::common;
use ::common::{Rotation2d, Transform2d};
use super::ContactHandleWeak;
use super::world::TimeStep;
use super::island::{Position, Velocity};
use cgmath::*;

const DEBUG_SOLVER: bool = false;
const BLOCK_SOLVE: bool = true;

struct ContactPositionConstraint {
    local_points: Vec<Vector2<f32>>,
    local_normal: Vector2<f32>,
    local_point: Vector2<f32>,
    index_a: usize,
    index_b: usize,
    inv_mass_a: f32,
    inv_mass_b: f32,
    local_center_a: Vector2<f32>,
    local_center_b: Vector2<f32>,
    inv_inertia_a: f32,
    inv_inertia_b: f32,
    manifold_type: ManifoldType,
    radius_a: f32,
    radius_b: f32,
    //point_count: u32,
}

struct VelocityConstraintPoint {
    r_a: Vector2<f32>,
    r_b: Vector2<f32>,
    normal_impulse: f32,
    tangent_impulse: f32,
    normal_mass: f32,
    tangent_mass: f32,
    velocity_bias: f32,
}

struct ContactVelocityConstraint {
    points: Vec<VelocityConstraintPoint>,
    normal: Vector2<f32>,
    normal_mass: Matrix2<f32>,
    k: Matrix2<f32>,
    index_a: usize,
    index_b: usize,
    inv_mass_a: f32,
    inv_mass_b: f32,
    inv_inertia_a: f32,
    inv_inertia_b: f32,
    friction: f32,
    restitution: f32,
    tangent_speed: f32,
    contact_index: usize,
}

struct PositionSolverManifold {
    normal: Vector2<f32>,
    point: Vector2<f32>,
    separation: f32,
}

impl PositionSolverManifold {
    fn new(pc: &ContactPositionConstraint, transform_a: &Transform2d, transform_b: &Transform2d,
           index: usize) -> Self {
        assert!(pc.local_points.len() > 0);

        let mut normal;
        let point;
        let separation;

        match pc.manifold_type {
            ManifoldType::FaceA => {
                normal = transform_a.rotation.apply(&pc.local_normal);
                let plane_point = transform_a.apply(&pc.local_point);

                let clip_point = transform_b.apply(&pc.local_points[index]);
                separation = dot(clip_point - plane_point, normal) - pc.radius_a - pc.radius_b;
                point = clip_point;
            }
            ManifoldType::FaceB => {
                normal = transform_b.rotation.apply(&pc.local_normal);
                let plane_point = transform_b.apply(&pc.local_point);

                let clip_point = transform_a.apply(&pc.local_points[index]);
                separation = dot(clip_point - plane_point, normal) - pc.radius_a - pc.radius_b;
                point = clip_point;

                // Ensure normal points from A to B.
                normal = -normal;
            }
        }

        PositionSolverManifold {
            normal: normal,
            point: point,
            separation: separation,
        }
    }
}

pub struct ContactSolver {
    step: TimeStep,
    position_constraints: Vec<ContactPositionConstraint>,
    velocity_constraints: Vec<ContactVelocityConstraint>,
}

impl ContactSolver {
    pub fn new(step: TimeStep, contacts: &Vec<ContactHandleWeak>) -> Self {
        let contact_count = contacts.len();
        let mut result = ContactSolver {
            step: step,
            position_constraints: Vec::with_capacity(contact_count),
            velocity_constraints: Vec::with_capacity(contact_count),
        };

        for (i, c) in contacts.iter().enumerate() {
            let c = c.upgrade().unwrap();
            let c = c.borrow();
            let body_a = c.fixture_a.borrow().body.upgrade().unwrap();
            let body_a = body_a.borrow();
            let body_b = c.fixture_b.borrow().body.upgrade().unwrap();
            let body_b = body_b.borrow();
            let manifold = &c.manifold;

            let mut vc = ContactVelocityConstraint {
                points: Vec::with_capacity(2),
                normal: Vector2::zero(),
                normal_mass: Matrix2::zero(),
                k: Matrix2::zero(),
                index_a: body_a.island_index,
                index_b: body_b.island_index,
                inv_mass_a: body_a.inv_mass,
                inv_mass_b: body_b.inv_mass,
                inv_inertia_a: body_a.inv_inertia,
                inv_inertia_b: body_b.inv_inertia,
                friction: c.friction,
                restitution: c.restitution,
                tangent_speed: c.tangent_speed,
                contact_index: i,
            };

            let mut pc = ContactPositionConstraint {
                local_points: Vec::with_capacity(2),
                local_normal: manifold.local_normal,
                local_point: manifold.local_point,
                index_a: body_a.island_index,
                index_b: body_b.island_index,
                inv_mass_a: body_a.inv_mass,
                inv_mass_b: body_b.inv_mass,
                local_center_a: body_a.sweep.local_center,
                local_center_b: body_b.sweep.local_center,
                inv_inertia_a: body_a.inv_inertia,
                inv_inertia_b: body_b.inv_inertia,
                manifold_type: manifold.manifold_type,
                radius_a: c.fixture_a.borrow().shape.get_radius(),
                radius_b: c.fixture_b.borrow().shape.get_radius(),
            };

            for mp in &manifold.points {
                let vcp = if step.warm_starting {
                    VelocityConstraintPoint {
                        r_a: Vector2::zero(),
                        r_b: Vector2::zero(),
                        normal_impulse: step.dt_ratio * mp.normal_impulse,
                        tangent_impulse: step.dt_ratio * mp.tangent_impulse,
                        normal_mass: 0.0,
                        tangent_mass: 0.0,
                        velocity_bias: 0.0,
                    }
                } else {
                    VelocityConstraintPoint {
                        r_a: Vector2::zero(),
                        r_b: Vector2::zero(),
                        normal_impulse: 0.0,
                        tangent_impulse: 0.0,
                        normal_mass: 0.0,
                        tangent_mass: 0.0,
                        velocity_bias: 0.0,
                    }
                };
                vc.points.push(vcp);
                pc.local_points.push(mp.local_point);
            }

            result.velocity_constraints.push(vc);
            result.position_constraints.push(pc);
        }

        result
    }

    pub fn initialize_velocity_constraints(&mut self, contacts: &Vec<ContactHandleWeak>,
                                           positions: &Vec<Position>, velocities: &Vec<Velocity>) {
        for i in 0..contacts.len() {
            let vc = &mut self.velocity_constraints[i];
            let pc = &mut self.position_constraints[i];

            let c = contacts[vc.contact_index].upgrade().unwrap();
            let manifold = &c.borrow().manifold;

            let index_a = vc.index_a;
            let index_b = vc.index_b;

            let m_a = vc.inv_mass_a;
            let i_a = vc.inv_inertia_a;
            let m_b = vc.inv_mass_b;
            let i_b = vc.inv_inertia_b;

            let c_a = positions[index_a].c;
            let a_a = positions[index_a].a;
            let v_a = velocities[index_a].v;
            let w_a = velocities[index_a].w;

            let c_b = positions[index_b].c;
            let a_b = positions[index_b].a;
            let v_b = velocities[index_b].v;
            let w_b = velocities[index_b].w;

            let rotation_a = Rotation2d::new(a_a);
            let rotation_b = Rotation2d::new(a_b);
            let position_a = c_a - rotation_a.apply(&pc.local_center_a);
            let position_b = c_b - rotation_b.apply(&pc.local_center_b);
            let transform_a = Transform2d::new(position_a, rotation_a);
            let transform_b = Transform2d::new(position_b, rotation_b);

            let mut world_manifold = WorldManifold::default();
            world_manifold.initialize(manifold, &transform_a, pc.radius_a, &transform_b, pc.radius_b);

            vc.normal = world_manifold.normal;

            for (vcp, wmp) in vc.points.iter_mut().zip(world_manifold.points.iter()) {
                vcp.r_a = wmp - c_a;
                vcp.r_b = wmp - c_b;

                let rn_a = vcp.r_a.perp_dot(vc.normal);
                let rn_b = vcp.r_b.perp_dot(vc.normal);

                let k_normal = m_a + m_b + i_a * rn_a * rn_a + i_b * rn_b * rn_b;

                vcp.normal_mass = if k_normal > 0.0 {
                    1.0 / k_normal
                } else {
                    0.0
                };

                let tangent = common::cross_v_s(&vc.normal, 1.0);

                let rt_a = vcp.r_a.perp_dot(tangent);
                let rt_b = vcp.r_b.perp_dot(tangent);

                let k_tangent = m_a + m_b + i_a * rt_a * rt_a + i_b * rt_b * rt_b;

                vcp.tangent_mass = if k_tangent > 0.0 {
                    1.0 / k_tangent
                } else {
                    0.0
                };

                // Set up a velocity bias for restitution.
                vcp.velocity_bias = 0.0;
                let v_rel = dot(vc.normal,
                                v_b + common::cross_s_v(w_b, &vcp.r_b) -
                                v_a - common::cross_s_v(w_a, &vcp.r_a));
                if v_rel < -::common::VELOCITY_THRESHOLD {
                    vcp.velocity_bias = -vc.restitution * v_rel;
                }
            }

            // If we have two points, then prepare the block solver.
            if vc.points.len() == 2 && BLOCK_SOLVE {
                let rn1_a;
                let rn1_b;
                let rn2_a;
                let rn2_b;
                {
                    let vcp1 = &vc.points[0];
                    let vcp2 = &vc.points[1];

                    rn1_a = vcp1.r_a.perp_dot(vc.normal);
                    rn1_b = vcp1.r_b.perp_dot(vc.normal);
                    rn2_a = vcp2.r_a.perp_dot(vc.normal);
                    rn2_b = vcp2.r_b.perp_dot(vc.normal);
                }


                let k11 = m_a + m_b + i_a * rn1_a * rn1_a + i_b * rn1_b * rn1_b;
                let k22 = m_a + m_b + i_a * rn2_a * rn2_a + i_b * rn2_b * rn2_b;
                let k12 = m_a + m_b + i_a * rn1_a * rn2_a + i_b * rn1_b * rn2_b;

                // Ensure a reasonable condition number.
                let k_max_condition_number = 1000.0;
                if k11 * k11 < k_max_condition_number * (k11 * k22 - k12 * k12) {
                    // k is safe to invert.
                    vc.k.x = vec2(k11, k12);
                    vc.k.y = vec2(k12, k22);
                    vc.normal_mass = vc.k.invert().unwrap();
                } else {
                    // The constraints are redundant, just use one.
                    // TODO_ERIN use deepest?
                    vc.points.truncate(1);
                }
            }
        }
    }

    pub fn warm_start(&mut self, velocities: &mut Vec<Velocity>) {
        for vc in &self.velocity_constraints {
            let index_a = vc.index_a;
            let index_b = vc.index_b;

            let m_a = vc.inv_mass_a;
            let m_b = vc.inv_mass_b;
            let i_a = vc.inv_inertia_a;
            let i_b = vc.inv_inertia_b;

            let mut v_a = velocities[index_a].v;
            let mut w_a = velocities[index_a].w;
            let mut v_b = velocities[index_b].v;
            let mut w_b = velocities[index_b].w;

            let normal = vc.normal;
            let tangent = common::cross_v_s(&normal, 1.0);

            for vcp in &vc.points {
                let p = normal * vcp.normal_impulse + tangent * vcp.tangent_impulse;
                w_a -= i_a * vcp.r_a.perp_dot(p);
                v_a = v_a - p * m_a;
                w_b += i_b * vcp.r_b.perp_dot(p);
                v_b = v_b + p * m_b;
            }

            velocities[index_a].v = v_a;
            velocities[index_a].w = w_a;
            velocities[index_b].v = v_b;
            velocities[index_b].w = w_b;
        }
    }

    pub fn solve_velocity_constraints(&mut self, velocities: &mut Vec<Velocity>) {
        for vc in &mut self.velocity_constraints {
            let index_a = vc.index_a;
            let index_b = vc.index_b;

            let m_a = vc.inv_mass_a;
            let i_a = vc.inv_inertia_a;
            let m_b = vc.inv_mass_b;
            let i_b = vc.inv_inertia_b;

            let mut v_a = velocities[index_a].v;
            let mut w_a = velocities[index_a].w;
            let mut v_b = velocities[index_b].v;
            let mut w_b = velocities[index_b].w;

            let normal = vc.normal;
            let tangent = common::cross_v_s(&normal, 1.0);
            let friction = vc.friction;

            // Solve tangent constraints first because non-penetration is more important
            // than friction.
            for vcp in &mut vc.points {
                // Relative velocity at contact.
                let dv = v_b + common::cross_s_v(w_b, &vcp.r_b) -
                         v_a - common::cross_s_v(w_a, &vcp.r_a);

                // Compute tangent impulse.
                let vt = dot(dv, tangent) - vc.tangent_speed;
                let mut lambda = vcp.tangent_mass * (-vt);

                // Clamp the accumulated impulse.
                let max_friction = friction * vcp.normal_impulse;
                let new_impulse = common::clamp_f32(vcp.tangent_impulse + lambda, -max_friction, max_friction);
                lambda = new_impulse - vcp.tangent_impulse;
                vcp.tangent_impulse = new_impulse;

                // Apply contact impulse.
                let p = tangent * lambda;

                v_a = v_a - p * m_a;
                w_a -= i_a * vcp.r_a.perp_dot(p);

                v_b = v_b + p * m_b;
                w_b += i_b * vcp.r_b.perp_dot(p);
            }

            // Solve normal constraints.
            if vc.points.len() == 1 || BLOCK_SOLVE == false {
                for vcp in &mut vc.points {
                    // Relative velocity at contact.
                    let dv = v_b + common::cross_s_v(w_b, &vcp.r_b) -
                             v_a - common::cross_s_v(w_a, &vcp.r_a);

                    // Compute normal impulse.
                    let vn = dot(dv, normal);
                    let mut lambda = -vcp.normal_mass * (vn - vcp.velocity_bias);

                    // Clamp the accumulated impulse.
                    let new_impulse = f32::max(vcp.normal_impulse + lambda, 0.0);
                    lambda = new_impulse - vcp.normal_impulse;
                    vcp.normal_impulse = new_impulse;

                    // Apply contact impulse.
                    let p = normal * lambda;

                    v_a = v_a - p * m_a;
                    w_a -= i_a * vcp.r_a.perp_dot(p);

                    v_b = v_b + p * m_b;
                    w_b += i_b * vcp.r_b.perp_dot(p);
                }
            } else {
                // From Box2D:
                //
                // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                // Build the mini LCP for this contact patch.
                //
                // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                //
                // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                // b = vn0 - velocityBias
                //
                // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                // solution that satisfies the problem is chosen.
                //
                // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                //
                // Substitute:
                //
                // x = a + d
                //
                // a := old total impulse
                // x := new total impulse
                // d := incremental impulse
                //
                // For the current iteration we extend the formula for the incremental impulse
                // to compute the new total impulse:
                //
                // vn = A * d + b
                //    = A * (x - a) + b
                //    = A * x + b - A * a
                //    = A * x + b'
                // b' = b - A * a

                //let vcp1 = &vc.points[0];
                //let vcp2 = &vc.points[1];

                let a = vec2(vc.points[0].normal_impulse, vc.points[1].normal_impulse);
                assert!(a.x >= 0.0 && a.y >= 0.0);

                // Relative velocity at contact.
                let dv1 = v_b + common::cross_s_v(w_b, &vc.points[0].r_b) -
                              v_a - common::cross_s_v(w_a, &vc.points[0].r_a);
                let dv2 = v_b + common::cross_s_v(w_b, &vc.points[1].r_b) -
                              v_a - common::cross_s_v(w_a, &vc.points[1].r_a);

                // Compute normal velocity.
                let mut vn1 = dot(dv1, normal);
                let mut vn2 = dot(dv2, normal);

                let mut b = vec2(vn1 - vc.points[0].velocity_bias, vn2 - vc.points[1].velocity_bias);

                // Compute b'
                b = b - vc.k * a;

                let k_error_tol = 1e-3;

                loop {
                    //
                    // Case 1: vn1 = 0 and vn2 = 0
                    //
                    // 0 = A * x + b'
                    //
                    // Solve for x:
                    //
                    // x = -inv(A) * b'
                    //
                    let mut x = -(vc.normal_mass * b);

                    if x.x >= 0.0 && x.y >= 0.0 {
                        // Accumulate.
                        vc.points[0].normal_impulse = x.x;
                        vc.points[1].normal_impulse = x.y;

                        {
                            let vcp1 = &vc.points[0];
                            let vcp2 = &vc.points[1];

                            // Get the incremental impulse.
                            let d = x - a;

                            // Apply incremental impulse.
                            let p1 = normal * d.x;
                            let p2 = normal * d.y;
                            v_a = v_a - (p1 + p2) * m_a;
                            w_a -= i_a * (vcp1.r_a.perp_dot(p1) + vcp2.r_a.perp_dot(p2));
                            v_b = v_b + (p1 + p2) * m_b;
                            w_b += i_b * (vcp1.r_b.perp_dot(p1) + vcp2.r_b.perp_dot(p2));

                            if DEBUG_SOLVER {
                                // Postconditions
                                /*dv1 = v_b + common::cross_s_v(w_b, &vcp1.r_b) -
                                      v_a - common::cross_s_v(w_a, &vcp1.r_a);
                                dv2 = v_b + common::cross_s_v(w_b, &vcp2.r_b) -
                                      v_a - common::cross_s_v(w_a, &vcp2.r_a);*/

                                // Compute normal velocity.
                                /*vn1 = dot(dv1, normal);
                                vn2 = dot(dv2, normal);*/

                                assert!(f32::abs(vn1 - vcp1.velocity_bias) < k_error_tol);
                                assert!(f32::abs(vn2 - vcp2.velocity_bias) < k_error_tol);
                            }
                        }
                        break;
                    }

                    //
                    // Case 2: vn1 = 0 and x2 = 0
                    //
                    //   0 = a11 * x1 + a12 * 0 + b1'
                    // vn2 = a21 * x1 + a22 * 0 + b2'
                    //
                    x.x = -vc.points[0].normal_mass * b.x;
                    x.y = 0.0;
                    vn1 = 0.0;
                    vn2 = vc.k.x.y * x.x + b.y;

                    if x.x >= 0.0 && vn2 >= 0.0 {
                        // Accumulate.
                        vc.points[0].normal_impulse = x.x;
                        vc.points[1].normal_impulse = x.y;
                        {
                            let vcp1 = &vc.points[0];
                            let vcp2 = &vc.points[1];

                            // Get the incremental impulse.
                            let d = x - a;

                            // Apply incremental impulse.
                            let p1 = normal * d.x;
                            let p2 = normal * d.y;
                            v_a = v_a - (p1 + p2) * m_a;
                            w_a -= i_a * (vcp1.r_a.perp_dot(p1) + vcp2.r_a.perp_dot(p2));
                            v_b = v_b + (p1 + p2) * m_b;
                            w_b += i_b * (vcp1.r_b.perp_dot(p1) + vcp2.r_b.perp_dot(p2));

                            if DEBUG_SOLVER {
                                // Postconditions
                                /*dv1 = v_b + common::cross_s_v(w_b, &vcp1.r_b) -
                                      v_a - common::cross_s_v(w_a, &vcp1.r_a);*/

                                // Compute normal velocity.
                                //vn1 = dot(dv1, normal);

                                assert!(f32::abs(vn1 - vcp1.velocity_bias) < k_error_tol);
                            }
                        }
                        break;
                    }

                    //
                    // Case 3: vn2 = 0 and x1 = 0
                    //
                    // vn1 = a11 * 0 + a12 * x2 + b1'
                    //   0 = a21 * 0 + a22 * x2 + b2'
                    //
                    x.x = 0.0;
                    x.y = -vc.points[1].normal_mass * b.y;
                    vn1 = vc.k.y.x * x.y + b.x;
                    vn2 = 0.0;

                    if x.y >= 0.0 && vn1 >= 0.0 {
                        // Accumulate.
                        vc.points[0].normal_impulse = x.x;
                        vc.points[1].normal_impulse = x.y;
                        {
                            let vcp1 = &vc.points[0];
                            let vcp2 = &vc.points[1];

                            // Resubstitute for the incremental impulse.
                            let d = x - a;

                            // Apply incremental impulse.
                            let p1 = normal * d.x;
                            let p2 = normal * d.y;
                            v_a = v_a - (p1 + p2) * m_a;
                            w_a -= i_a * (vcp1.r_a.perp_dot(p1) + vcp2.r_a.perp_dot(p2));
                            v_b = v_b + (p1 + p2) * m_b;
                            w_b += i_b * (vcp1.r_b.perp_dot(p1) + vcp2.r_b.perp_dot(p2));

                            if DEBUG_SOLVER {
                                // Postconditions
                                /*dv2 = v_b + common::cross_s_v(w_b, &vcp2.r_b) -
                                      v_a - common::cross_s_v(w_a, &vcp2.r_a);*/

                                // Compute normal velocity.
                                //vn2 = dot(dv2, normal);

                                assert!(f32::abs(vn2 - vcp2.velocity_bias) < k_error_tol);
                            }
                        }
                        break;
                    }

                    //
                    // Case 4: x1 = 0 and x2 = 0
                    //
                    // vn1 = b1
                    // vn2 = b2
                    //
                    x.x = 0.0;
                    x.y = 0.0;
                    vn1 = b.x;
                    vn2 = b.y;

                    if vn1 >= 0.0 && vn2 >= 0.0 {
                        // Accumulate.
                        vc.points[0].normal_impulse = x.x;
                        vc.points[1].normal_impulse = x.y;
                        {
                            let vcp1 = &vc.points[0];
                            let vcp2 = &vc.points[1];

                            // Resubstitute for the incremental impulse.
                            let d = x - a;

                            // Apply incremental impulse.
                            let p1 = normal * d.x;
                            let p2 = normal * d.y;
                            v_a = v_a - (p1 + p2) * m_a;
                            w_a -= i_a * (vcp1.r_a.perp_dot(p1) + vcp2.r_a.perp_dot(p2));
                            v_b = v_b + (p1 + p2) * m_b;
                            w_b += i_b * (vcp1.r_b.perp_dot(p1) + vcp2.r_b.perp_dot(p2));
                        }
                        break;
                    }

                    // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                    break;
                }
            }

            velocities[index_a].v = v_a;
            velocities[index_a].w = w_a;
            velocities[index_b].v = v_b;
            velocities[index_b].w = w_b;
        }
    }

    pub fn store_impulses(&self, contacts: &Vec<ContactHandleWeak>) {
        for vc in &self.velocity_constraints {
            let c = contacts[vc.contact_index].upgrade().unwrap();
            let manifold = &mut c.borrow_mut().manifold;

            /*for (i, vcp) in vc.points.iter().enumerate() {
                manifold.points[i].normal_impulse = vcp.normal_impulse;
                manifold.points[i].tangent_impulse = vcp.tangent_impulse;
            }*/

            for (vcp, mp) in vc.points.iter().zip(manifold.points.iter_mut()) {
                // TODO: vc.points.len == 1 && manifold.points.len() == 2
                mp.normal_impulse = vcp.normal_impulse;
                mp.tangent_impulse = vcp.tangent_impulse;
            }
        }
    }

    // Sequential solver.
    pub fn solve_position_constraints(&self, positions: &mut Vec<Position>) -> bool {
        let mut min_separation = 0.0;

        for pc in &self.position_constraints {
            let index_a = pc.index_a;
            let index_b = pc.index_b;

            let m_a = pc.inv_mass_a;
            let i_a = pc.inv_inertia_a;
            let m_b = pc.inv_mass_b;
            let i_b = pc.inv_inertia_b;

            let mut c_a = positions[index_a].c;
            let mut a_a = positions[index_a].a;
            let mut c_b = positions[index_b].c;
            let mut a_b = positions[index_b].a;

            // Solve normal constraints.
            for i in 0..pc.local_points.len() {
                let rotation_a = Rotation2d::new(a_a);
                let rotation_b = Rotation2d::new(a_b);
                let position_a = c_a - rotation_a.apply(&pc.local_center_a);
                let position_b = c_b - rotation_b.apply(&pc.local_center_b);
                let transform_a = Transform2d::new(position_a, rotation_a);
                let transform_b = Transform2d::new(position_b, rotation_b);

                let psm = PositionSolverManifold::new(&pc, &transform_a, &transform_b, i);
                let normal = psm.normal;
                let point = psm.point;
                let separation = psm.separation;

                let r_a = point - c_a;
                let r_b = point - c_b;

                // Track max constraint error.
                min_separation = f32::min(min_separation, separation);

                // Prevent large corrections and allow slop.
                let c = common::clamp_f32(common::BAUMGARTE * (separation + common::LINEAR_SLOP), -common::MAX_LINEAR_CORRECTION, 0.0);

                // Compute the effective mass.
                let rn_a = r_a.perp_dot(normal);
                let rn_b = r_b.perp_dot(normal);
                let k = m_a + m_b + i_a * rn_a * rn_a + i_b * rn_b * rn_b;

                // Compute normal impulse.
                let impulse = if k > 0.0 {
                    -c / k
                } else {
                    0.0
                };

                let p = normal * impulse;

                c_a = c_a - p * m_a;
                a_a -= i_a * r_a.perp_dot(p);

                c_b = c_b + p * m_b;
                a_b += i_b * r_b.perp_dot(p);
            }

            positions[index_a].c = c_a;
            positions[index_a].a = a_a;
            positions[index_b].c = c_b;
            positions[index_b].a = a_b;
        }

        // We can't expect min_separation >= -LINEAR_SLOP because we don't
        // push the separation above -LINEAR_SLOP.
        min_separation >= -3.0 * common::LINEAR_SLOP
    }

    // Sequential position solver for position constraints.
    pub fn solve_toi_position_constraints(&self, positions: &mut Vec<Position>,
           toi_index_a: usize, toi_index_b: usize) -> bool {
        let mut min_separation = 0.0;

        for pc in &self.position_constraints {
            let index_a = pc.index_a;
            let index_b = pc.index_b;

            let mut m_a = 0.0;
            let mut i_a = 0.0;
            if index_a == toi_index_a || index_a == toi_index_b {
                m_a = pc.inv_mass_a;
                i_a = pc.inv_inertia_a;
            }

            let mut m_b = 0.0;
            let mut i_b = 0.0;
            if index_b == toi_index_a || index_b == toi_index_b {
                m_b = pc.inv_mass_b;
                i_b = pc.inv_inertia_b;
            }

            let mut c_a = positions[index_a].c;
            let mut a_a = positions[index_a].a;
            let mut c_b = positions[index_b].c;
            let mut a_b = positions[index_b].a;

            // Solve normal constraints.
            for j in 0..pc.local_points.len() {
                let rotation_a = Rotation2d::new(a_a);
                let rotation_b = Rotation2d::new(a_b);
                let position_a = c_a - rotation_a.apply(&pc.local_center_a);
                let position_b = c_b - rotation_b.apply(&pc.local_center_b);
                let transform_a = Transform2d::new(position_a, rotation_a);
                let transform_b = Transform2d::new(position_b, rotation_b);

                let psm = PositionSolverManifold::new(&pc, &transform_a, &transform_b, j);
                let normal = psm.normal;
                let point = psm.point;
                let separation = psm.separation;

                let r_a = point - c_a;
                let r_b = point - c_b;

                // Track max constraint error.
                min_separation = f32::min(min_separation, separation);

                // Prevent large corrections and allow slop.
                let c = common::clamp_f32(common::TOI_BAUMGARTE * (separation + common::LINEAR_SLOP), -common::MAX_LINEAR_CORRECTION, 0.0);

                // Compute the effective mass.
                let rn_a = r_a.perp_dot(normal);
                let rn_b = r_b.perp_dot(normal);
                let k = m_a + m_b + i_a * rn_a * rn_a + i_b * rn_b * rn_b;

                // Compute normal impulse.
                let impulse = if k > 0.0 {
                    -c / k
                } else {
                    0.0
                };

                let p = normal * impulse;

                c_a = c_a - p * m_a;
                a_a -= i_a * r_a.perp_dot(p);

                c_b = c_b + p * m_b;
                a_b += i_b * r_b.perp_dot(p);
            }

            positions[index_a].c = c_a;
            positions[index_a].a = a_a;
            positions[index_b].c = c_b;
            positions[index_b].a = a_b;
        }

        // We can't expect min_separation >= -LINEAR_SLOP because we don't
        // push the separation above -LINEAR_SLOP.
        min_separation >= -1.5 * common::LINEAR_SLOP
    }
}
