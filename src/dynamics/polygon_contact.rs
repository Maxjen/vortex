use super::{FixtureHandle, World};
use ::collision::Manifold;
use ::collision;

macro_rules! try_some {
    ($e:expr) => (match $e { Some(s) => s, None => return })
}

pub struct PolygonContact {
    pub fixture_a: FixtureHandle,
    pub fixture_b: FixtureHandle,

    manifold: Manifold,

    touching: bool,
}

impl PolygonContact {
    pub fn new(fixture_a: FixtureHandle, fixture_b: FixtureHandle) -> Self {
        PolygonContact {
            fixture_a: fixture_a,
            fixture_b: fixture_b,
            manifold: Manifold::new(),
            touching: false,
        }
    }

    pub fn update(&mut self, world: &World) {
        let old_manifold = self.manifold.clone();

        let body_id_a = self.fixture_a.borrow().body_id;
        let body_id_b = self.fixture_b.borrow().body_id;

        let body_a = try_some!(world.get_body(body_id_a));
        let body_b = try_some!(world.get_body(body_id_b));

        let transform_a = body_a.borrow().get_transform();
        let transform_b = body_a.borrow().get_transform();

        let was_touching = self.touching;

        self.manifold = collision::collide_polygons(&self.fixture_a.borrow().shape, &transform_a, &self.fixture_b.borrow().shape, &transform_b);
        self.touching = self.manifold.points.len() > 0;

        // Match old contact ids to new contact ids and copy the
        // stored impulses to warm start the solver.
        for mp in &self.manifold.points {
            for mp_old in &old_manifold.points {
                
            }
        }
    }
}
