use ::collision::ManifoldType;
use super::ContactHandleWeak;
use super::world::TimeStep;
use cgmath::*;

struct ContactPositionConstraint {
    local_points: Vec<Vector2<f32>>,
    local_normal: Vector2<f32>,
    local_point: Vector2<f32>,
    index_a: u32,
    index_b: u32,
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
    tangent_mass: f32,
    velocity_bias: f32,
}

struct ContactVelocityConstraint {
    points: Vec<VelocityConstraintPoint>,
    normal: Vector2<f32>,
}

pub struct ContactSolver {
    step: TimeStep,
    position_constraints: Vec<ContactPositionConstraint>,
    velocity_constraints: Vec<ContactVelocityConstraint>,
}

impl ContactSolver {
    pub fn new(step: TimeStep, contacts: &Vec<ContactHandleWeak>) -> Self {
        let contact_count = contacts.len();
        let result = ContactSolver {
            step: step,
            position_constraints: Vec::with_capacity(contact_count),
            velocity_constraints: Vec::with_capacity(contact_count),
        };

        for c in contacts {
            /*let vc = ContactVelocityConstraint {
                points: Vec::with_capacity(2),

            }*/
        }

        result
    }

    pub fn initialize_velocity_constraints(&mut self) {

    }

    pub fn warm_start(&mut self) {

    }

    pub fn solve_velocity_constraints(&mut self) {

    }

    pub fn store_impulses(&self) {

    }
}
