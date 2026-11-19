//! Great-circle distances, geographic bearings, and EPSG:4326/EPSG:3857 coordinate conversions.

mod spatial;

pub use self::spatial::{
    convert_epsg3857_to_4326, convert_epsg4326_to_3857, gc_distance_pt, get_bearing,
};

#[cfg(test)]
mod tests;
