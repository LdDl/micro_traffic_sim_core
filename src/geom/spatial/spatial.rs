//! This module provides functions for geographic calculations, such as
//! calculating bearings between two points and converting coordinates
//! between EPSG:4326 and EPSG:3857.

use crate::geom::math::gc_distance;
use crate::geom::point::{Point, PointType};
use std::f64::consts::PI;

const EQUATOR_HALF: f64 = 20037508.34; // Half-length of the equator, (PI * 6378137) basically

/// Calculates the bearing between two geographic points.
///
/// The bearing is the angle measured clockwise from the North direction, from `pt1` to `pt2`.
/// The result is given in degrees.
///
/// # Arguments
///
/// * `pt1` - The first point (longitude and latitude in degrees for WGS84).
/// * `pt2` - The second point (longitude and latitude in degrees for WGS84).
///
/// # Returns
///
/// Returns the bearing from `pt1` to `pt2` in degrees.
///
/// # Example
///
/// ```
/// use micro_traffic_sim_core::geom::{new_point, get_bearing, SRID};
/// let pt1 = new_point(35.90434, 56.89028, Some(SRID::WGS84));
/// let pt2 = new_point(35.90430, 56.89033, Some(SRID::WGS84));
/// let bearing = get_bearing(&pt1, &pt2);
/// ```
///
/// ```rust
/// use micro_traffic_sim_core::geom::{new_point, SRID, get_bearing, gc_distance};
///
/// // Moscow
/// let pt1 = new_point(37.6176, 55.7558, Some(SRID::WGS84));
/// // St. Petersburg
/// let pt2 = new_point(30.3141, 59.9386, Some(SRID::WGS84));
///
/// // Get bearing from Moscow to St. Petersburg
/// // Degrees from North
/// let bearing = get_bearing(&pt1, &pt2);
///
/// // Calculate great-circle distance
/// // Meters
/// let distance = gc_distance(37.6176, 55.7558, 30.3141, 59.9386);
/// ```
pub fn get_bearing(pt1: &PointType, pt2: &PointType) -> f64 {
    let lon1 = pt1.x();
    let lat1 = pt1.y();
    let lon2 = pt2.x();
    let lat2 = pt2.y();

    let λ1 = lon1 * PI / 180.0;
    let φ1 = lat1 * PI / 180.0;
    let λ2 = lon2 * PI / 180.0;
    let φ2 = lat2 * PI / 180.0;

    let y = (λ2 - λ1).sin() * φ2.cos();
    let x = φ1.cos() * φ2.sin() - φ1.sin() * φ2.cos() * (λ2 - λ1).cos();
    let θ = y.atan2(x);
    θ * 180.0 / PI
}

/// Converts geographic coordinates from EPSG:4326 (latitude/longitude in degrees)
/// to EPSG:3857 (Web Mercator projection in meters).
///
/// # Arguments
///
/// * `lat` - Latitude in degrees (EPSG:4326).
/// * `lng` - Longitude in degrees (EPSG:4326).
///
/// # Returns
///
/// Returns a tuple containing the converted latitude and longitude in EPSG:3857 projection (in meters).
///
/// # Example
///
/// ```
/// use micro_traffic_sim_core::geom::convert_epsg4326_to_3857;
/// let (lat_3857, lng_3857) = convert_epsg4326_to_3857(40.748817, -73.985428);
/// ```
///
/// ```rust
/// use micro_traffic_sim_core::geom::{convert_epsg4326_to_3857, convert_epsg3857_to_4326};
///
/// // Convert from WGS84 to Web Mercator (for web mapping)
/// let (x_mercator, y_mercator) = convert_epsg4326_to_3857(37.6176, 55.7558);
///
/// // Convert back to WGS84
/// let (lon, lat) = convert_epsg3857_to_4326(x_mercator, y_mercator);
/// ```
pub fn convert_epsg4326_to_3857(lon: f64, lat: f64) -> (f64, f64) {
    let x = lon * EQUATOR_HALF / 180.0;
    let y = ((90.0+lat)*PI/360.0).tan().ln() / (PI / 180.0);
    let y = y * EQUATOR_HALF / 180.0;
    (x, y)
}

/// Converts geographic coordinates from EPSG:3857 (Web Mercator projection in meters)
/// to EPSG:4326 (latitude/longitude in degrees).
///
/// This conversion is the reverse of `convert_epsg4326_to_3857` and used to
/// transform coordinates back into the geographic coordinate system.
///
/// # Arguments
///
/// * `lat` - Latitude in meters (EPSG:3857).
/// * `lng` - Longitude in meters (EPSG:3857).
///
/// # Returns
///
/// Returns a tuple containing the converted latitude and longitude in EPSG:4326 (in degrees).
///
/// # Example
///
/// ```
/// use micro_traffic_sim_core::geom::convert_epsg3857_to_4326;
/// let (lat_4326, lng_4326) = convert_epsg3857_to_4326(20037508.34, 20037508.34);
/// ```
pub fn convert_epsg3857_to_4326(lat: f64, lng: f64) -> (f64, f64) {
    let new_lat = lat * 180.0 / EQUATOR_HALF;
    let new_lng = (lng * PI / EQUATOR_HALF).exp().atan() * 360.0 / PI - 90.0;
    (new_lat, new_lng)
}

/// gc_distance_pt
/// 
/// Computes the great-circle distance (orthodromic distance) between two points on the Earth's surface using the Haversine formula.
/// This version is named to suggest it could be used in a computer vision context (e.g., OpenCV). It wraps around the `haversine` function.
/// 
/// # References
/// For detailed explanation of the Haversine formula, see:
/// [Great-circle distance - Wikipedia](https://en.wikipedia.org/wiki/Great-circle_distance#:~:text=The%20great%2Dcircle%20distance%2C%20orthodromic,line%20through%20the%20sphere's%20interior)
/// 
/// # Arguments
/// 
/// * `src` - A `Point` representing the source point, which contains longitude (`x`) and latitude (`y`) in degrees.
/// * `dst` - A `Point` representing the destination point, which also contains longitude (`x`) and latitude (`y`) in degrees.
/// 
/// # Returns
/// 
/// A `f64` value representing the great-circle distance in meters between the source and destination points.
///
/// # Example
/// ```
/// use micro_traffic_sim_core::geom::{new_point, SRID, gc_distance_pt};
/// let src = new_point(35.90434, 56.89028, Some(SRID::WGS84));
/// let dst = new_point(35.90434, 56.89028, Some(SRID::WGS84));
/// let distance = gc_distance_pt(&src, &dst);
/// println!("Great-circle distance: {} km", distance);
/// ```
pub fn gc_distance_pt(src: &PointType, dst: &PointType) -> f64 {
    gc_distance(src.x(), src.y(), dst.x(), dst.y())
}
