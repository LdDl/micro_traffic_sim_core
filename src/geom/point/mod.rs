//! Euclidean and WGS84 point types, their shared interface, and point construction.

mod point;

pub use self::point::{EuclideanPoint, Point, PointType, SRID, WGS84Point, new_point};

#[cfg(test)]
mod tests;
