//! Geometric primitives and spatial calculations for Euclidean and WGS84 coordinates.
//!
//! Exports point types and their shared interface, distance and bearing calculations, and coordinate conversions between EPSG:4326 and EPSG:3857.

mod math;
mod point;
mod spatial;

pub use self::{math::gc_distance, point::*, spatial::*};
