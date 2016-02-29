use cgmath::*;

pub trait DebugDraw {
    fn draw_polygon(&mut self, vertices: &Vec<Vector2<f32>>);
}
