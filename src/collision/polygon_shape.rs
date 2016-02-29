use cgmath::*;
use ::common;
use ::common::{Transform2d};
use super::Aabb;

pub struct PolygonShape {
    pub centroid: Vector2<f32>,
    pub vertices: Vec<Vector2<f32>>,
    pub normals: Vec<Vector2<f32>>,
}

impl PolygonShape {
    pub fn new() -> PolygonShape {
        PolygonShape {
            centroid: Vector2::<f32>::new(0.0, 0.0),
            vertices: Vec::new(),
            normals: Vec::new(),
        }
    }

    fn compute_centroid(&mut self) {
        self.centroid.x = 0.0;
        self.centroid.y = 0.0;
        let mut area = 0.0;

        let p_ref = Vector2::<f32>::new(0.0, 0.0);
        let inv3 = 1.0 / 3.0;
        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = if i + 1 < self.vertices.len() {
                self.vertices[i + 1]
            } else {
                self.vertices[0]
            };
            let e1 = p1 - p_ref;
            let e2 = p2 - p_ref;
            let triangle_area = 0.5 * e1.perp_dot(e2);
            area += triangle_area;
            self.centroid = self.centroid + (p_ref + p1 + p2) * triangle_area * inv3;
        }
        self.centroid = self.centroid * 1.0 / area;
    }

    pub fn set(&mut self, vertices: &Vec<Vector2<f32>>) {
        if vertices.len() < 3 {
            return;
        }

        let mut vertices_tmp = Vec::<Vector2<f32>>::new();

        'outer: for v in vertices {
            for v_tmp in &vertices_tmp {
                if (v - v_tmp).length2() < (0.5 * common::LINEAR_SLOP) * (0.5 * common::LINEAR_SLOP) {
                    continue 'outer;
                }
            }
            vertices_tmp.push(*v);
        }

        if vertices_tmp.len() < 3 {
            return;
        }

        let mut i0 = 0;
        //let mut x0 = vertices_tmp.first().unwrap().x;
        //let mut y0 = vertices_tmp.first().unwrap().x;
        for (i, v) in vertices_tmp.iter().enumerate().skip(1) {
            if v.x > vertices_tmp[i0].x || (v.x == vertices_tmp[i0].x && v.y < vertices_tmp[i0].y) {
                i0 = i;
            }
        }

        let mut hull = Vec::<usize>::new();
        let m = 0;
        let mut ih = i0;

        loop {
            hull.push(ih);

            let mut ie = 0;
            if ie == ih {
                ie = 1;
            }

            for (i, v) in vertices_tmp.iter().enumerate().skip(1) {
                let r = vertices_tmp[ie] - vertices_tmp[*hull.last().unwrap()];
                let v = vertices_tmp[i] - vertices_tmp[*hull.last().unwrap()];
                let c = r.perp_dot(v);
                if c < 0.0 {
                    ie = i;
                }
                if c == 0.0 && v.length2() > r.length2() {
                    ie = i;
                }
            }

            ih = ie;
            if ie == i0 {
                break;
            }
        }

        if hull.len() < 3 {
            return;
        }

        for i in &hull {
            self.vertices.push(vertices_tmp[*i]);
        }

        for i in 0..self.vertices.len() {
            let i2 = if i + 1 < self.vertices.len() {
                i + 1
            } else {
                0
            };
            let edge = self.vertices[i2] - self.vertices[i];
            let mut normal = Vector2::<f32>::new(edge.y, -edge.x);
            normal = normal.normalize();
            self.normals.push(normal);
        }

        self.compute_centroid();

        println!("{:?}, {:?}", self.vertices, self.centroid);
    }

    pub fn set_as_box(&mut self, half_width: f32, half_height: f32) {
        self.vertices.clear();
        self.vertices.push(Vector2::new(-half_width, -half_height));
        self.vertices.push(Vector2::new(half_width, -half_height));
        self.vertices.push(Vector2::new(half_width, half_height));
        self.vertices.push(Vector2::new(-half_width, half_height));

        self.normals.clear();
        self.normals.push(Vector2::new(0.0, -1.0));
        self.normals.push(Vector2::new(1.0, 0.0));
        self.normals.push(Vector2::new(0.0, 1.0));
        self.normals.push(Vector2::new(-1.0, 0.0));
    }

    pub fn compute_aabb(&self, transform: Transform2d) -> Aabb {
        let mut min = transform.apply_to_vector(&self.vertices[0]);
        let mut max = min;
        for v in &self.vertices {
            let v = transform.apply_to_vector(&v);
            min.x = f32::min(min.x, v.x);
            min.y = f32::min(min.y, v.y);
            max.x = f32::max(max.x, v.x);
            max.y = f32::max(max.y, v.y);
        }
        let r = Vector2::<f32>::new(common::POLYGON_RADIUS, common::POLYGON_RADIUS);
        Aabb {
            min: min - r,
            max: max + r,
        }
    }
}
