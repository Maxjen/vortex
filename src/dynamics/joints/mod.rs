pub use self::mouse_joint::{MouseJointConfig, MouseJoint};

use std::rc::{Rc, Weak};
use std::cell::RefCell;
use std::mem;
use std::ptr;
use super::{Body, BodyHandleWeak};
use super::island::{Position, Velocity};
use ::dynamics::world::TimeStep;

mod mouse_joint;

pub type JointHandle<'a> = Rc<RefCell<Joint<'a>>>;
pub type JointHandleWeak<'a> = Weak<RefCell<Joint<'a>>>;

/// A joint edge is used to connect bodies and joints together in a joint graph where
/// each body is a node and each joint is an edge. Each joint has two joint nodes,
/// one for each attached body.
pub struct JointEdge<'a> {
    pub body: BodyHandleWeak<'a>,
    pub joint: JointHandleWeak<'a>,
}

pub enum JointType {
    Mouse(MouseJointConfig),
}

/// `JointConfig`s are used to construct joints.
pub struct JointConfig<'a> {
    pub joint_type: JointType,

    /// The first attached body.
    pub body_a: BodyHandleWeak<'a>,

    /// The second attached body.
    pub body_b: BodyHandleWeak<'a>,

    /// Set this flag to true if the attached bodies should collide.
    pub collide_connected: bool,
}

pub struct JointData<'a> {
    body_a: BodyHandleWeak<'a>,
    body_b: BodyHandleWeak<'a>,

    is_island: bool,
    is_collide_connected: bool,
}

pub enum Joint<'a> {
    Mouse(MouseJoint<'a>),
}

impl<'a> Joint<'a> {
    /*pub fn new(joint_config: &JointConfig<'a>) -> JointHandle<'a> {
        let result: JointHandle<'a>;
        unsafe {
            result = Rc::new(RefCell::new(mem::uninitialized()));
            let edge_to_a = JointEdge {
                body: joint_config.body_a.clone(),
                joint: Rc::downgrade(&result),
            };
            let edge_to_b = JointEdge {
                body: joint_config.body_b.clone(),
                joint: Rc::downgrade(&result),
            };
            let joint_data = JointData {
                edge_to_a: edge_to_a,
                edge_to_b: edge_to_b,
                is_island: false,
                is_collide_connected: joint_config.collide_connected,
            };
            match joint_config.joint_type {
                JointType::Mouse(ref joint_config) => {
                    ptr::write(&mut *result.borrow_mut(), Joint::Mouse(MouseJoint::new(joint_config, joint_data)));
                }
            }
        }

        result
    }*/
    pub fn new(joint_config: &JointConfig<'a>) -> JointHandle<'a> {
        let joint_data = JointData {
            body_a: joint_config.body_a.clone(),
            body_b: joint_config.body_b.clone(),
            is_island: false,
            is_collide_connected: joint_config.collide_connected,
        };
        let result;
        result = match joint_config.joint_type {
            JointType::Mouse(ref joint_config) => Rc::new(RefCell::new(Joint::Mouse(MouseJoint::new(joint_config, joint_data)))),
        };

        result
    }

    fn get_joint_data(&self) -> &JointData<'a> {
        match self {
            &Joint::Mouse(ref joint) => &joint.joint_data,
        }
    }

    fn get_joint_data_mut(&mut self) -> &mut JointData<'a> {
        match self {
            &mut Joint::Mouse(ref mut joint) => &mut joint.joint_data,
        }
    }

    pub fn get_other_body(&self, body: BodyHandleWeak<'a>) -> Option<BodyHandleWeak<'a>> {
        let b = body.upgrade().unwrap();
        let pb = &(*b) as *const RefCell<Body>;

        let b_a = self.get_joint_data().body_a.upgrade().unwrap();
        let pb_a = &(*b_a) as *const RefCell<Body>;
        if pb == pb_a {
            return Some(self.get_joint_data().body_b.clone());
        }

        let b_b = self.get_joint_data().body_b.upgrade().unwrap();
        let pb_b = &(*b_b) as *const RefCell<Body>;
        if pb == pb_b {
            return Some(self.get_joint_data().body_a.clone());
        }

        None
    }

    pub fn set_island(&mut self, is_island: bool) {
        self.get_joint_data_mut().is_island = is_island;
    }

    pub fn is_island(&self) -> bool {
        self.get_joint_data().is_island
    }

    pub fn initialize_velocity_constraints(&mut self, step: TimeStep, positions: &Vec<Position>, velocities: &mut Vec<Velocity>) {
        match self {
            &mut Joint::Mouse(ref mut joint) => joint.initialize_velocity_constraints(step, positions, velocities),
        }
    }

    pub fn solve_velocity_constraints(&mut self, step: TimeStep, velocities: &mut Vec<Velocity>) {
        match self {
            &mut Joint::Mouse(ref mut joint) => joint.solve_velocity_constraints(step, velocities),
        }
    }

    /// This returns true if the position errors are within tolerance.
    pub fn solve_position_constraints(&mut self, step: TimeStep, positions: &mut Vec<Position>) -> bool {
        true
    }
}
