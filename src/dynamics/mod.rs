pub use self::contact_manager::ContactManager;
pub use self::body::{Body, BodyHandle};
pub use self::fixture::{Fixture, FixtureHandle};
pub use self::world::{World, WorldHandle};

mod contact_manager;
mod body;
mod fixture;
mod world;
