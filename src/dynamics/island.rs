use super::{BodyHandleWeak, ContactHandleWeak, ContactSolver};
use super::world::{Profile, TimeStep};
use ::common::Timer;
use cgmath::*;

/*
From Box2D:

Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity vector error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is recomputed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
recomputed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than LINEAR_SLOP.

Full NGS or just NGS - Like Modified NGS except the effective masses are recomputed
each time a constraint is solved.

Here are the results:

Baumgarte - This is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - This is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on in.

Modified NGS - This algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - Stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations

Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefid of Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta)]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

pub struct Position {
    c: Vector2<f32>,
    a: f32,
}

pub struct Velocity {
    v: Vector2<f32>,
    w: f32,
}

pub struct Island<'a> {
    bodies: Vec<BodyHandleWeak<'a>>,
    contacts: Vec<ContactHandleWeak<'a>>,

    positions: Vec<Position>,
    velocities: Vec<Velocity>,
}

impl<'a> Island<'a> {
    pub fn new(body_capacity: usize, contact_capacity: usize) -> Island<'a> {
        Island {
            bodies: Vec::with_capacity(body_capacity),
            contacts: Vec::with_capacity(contact_capacity),
            positions: Vec::with_capacity(body_capacity),
            velocities: Vec::with_capacity(body_capacity),
        }
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.contacts.clear();
    }

    pub fn add_body(&mut self, body: BodyHandleWeak<'a>) {
        self.bodies.push(body);
    }

    pub fn add_contact(&mut self, contact: ContactHandleWeak<'a>) {
        self.contacts.push(contact);
    }

    pub fn solve(&mut self, profile: &mut Profile, step: &TimeStep, gravity: &Vector2<f32>, allow_sleep: bool) {
        let dt = step.dt;

        // Integrate velocities and apply damping. Initialize the body state.
        for b in &self.bodies {
            let b = b.upgrade().unwrap();

            let c = b.borrow().sweep.c;
            let a = b.borrow().sweep.a;
            let mut v = b.borrow().get_linear_velocity();
            let mut w = b.borrow().get_angular_velocity();

            // Store positions for continuous collision.
            b.borrow_mut().sweep.c0 = c;
            b.borrow_mut().sweep.a0 = a;

            // TODO: only for dynamic bodies
            // Integrate velocities.
            v = v + (gravity * b.borrow().gravity_scale + b.borrow().force * b.borrow().inv_mass) * dt;
            w += dt * b.borrow().inv_inertia * b.borrow().torque;

            // Apply damping.
            // ODE: dv/dt + c * v = 0
            // Solution: v(t) = v0 * exp(-c * t)
            // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
            // v2 = exp(-c * dt) * v1
            // Pade approximation:
            // v2 = v1 * 1 / (1 + c * dt)
            v = v * 1.0 / (1.0 + dt * b.borrow().linear_damping);
            w *= 1.0 / (1.0 + dt * b.borrow().angular_damping);

            self.positions.push(Position {
                c: c,
                a: a,
            });
            self.velocities.push(Velocity {
                v: v,
                w: w,
            });
        }

        let mut timer = Timer::new();

        let mut contact_solver = ContactSolver::new(*step, &self.contacts);

        if step.warm_starting {
            contact_solver.warm_start();
        }

        profile.solve_init = timer.get_milliseconds();

        // Solve velocity constraints.
        timer.reset();
        for i in 0..step.velocity_iterations {
            contact_solver.solve_velocity_constraints();
        }

        // Store impulses for warm starting.
        contact_solver.store_impulses();
    }
}
