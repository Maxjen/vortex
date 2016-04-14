use cgmath::*;

pub trait DebugDraw {
    fn draw_polygon(&mut self, vertices: &Vec<Vector2<f32>>);
    fn draw_segment(&mut self, v1: &Vector2<f32>, v2: &Vector2<f32>);
}
