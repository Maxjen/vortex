pub struct PolygonContact {
    fixture_a: FixtureHandle,
    fixture_b: FixtureHandle,
}

impl PolygonContact {
    pub fn new(fixture_a: FixtureHandle, fixture_b: FixtureHandle) -> Self {
        PolygonContact {
            fixture_a: fixture_a,
            fixture_b: fixture_b,
        }
    }
}
