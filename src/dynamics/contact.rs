use super::PolygonContact;

pub enum Contact {
    Polygon(PolygonContact<'a>),
}
