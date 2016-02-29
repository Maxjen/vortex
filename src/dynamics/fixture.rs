use ::collision::PolygonShape;

pub struct Fixture {
    //density: f32,
    shape: PolygonShape,
    /*friction: f32,
    restitution: f32,*/
}

impl Fixture {
    pub fn new(shape: PolygonShape) -> Self {
        Fixture {
            shape: shape,
        }
    }

    pub fn get_shape(&self) -> &PolygonShape {
        &self.shape
    }
}
