pub use self::mouse_joint::MouseJoint;

use std::rc::{Rc, Weak};
use std::cell::RefCell;
use super::{BodyHandleWeak};
use super::island::{Position, Velocity};

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
    Mouse,
}

pub struct JointConfig<'a> {
    joint_type: JointType,
    body_a: BodyHandleWeak<'a>,
    body_b: BodyHandleWeak<'a>,
    collide_connected: bool,
}

enum JointInternal {
    Mouse(MouseJoint),
}

pub struct Joint<'a> {
    joint: JointInternal,
    edge_to_a: JointEdge<'a>,
    edge_to_b: JointEdge<'a>,

    is_island: bool,
    is_collide_connected: bool,
}

impl<'a> Joint<'a> {
    pub fn new() {
        
    }
}
