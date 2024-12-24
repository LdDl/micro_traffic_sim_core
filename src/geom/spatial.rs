//! This module provides functions for geographic calculations, such as
//! calculating bearings between two points and converting coordinates
//! between EPSG:4326 and EPSG:3857.

use crate::geom::Point;
use std::f64::consts::PI;
const EARTH_RADIUS_METERS: f64 = 6371000.0;
const EQUATOR_HALF: f64 = 20037508.34; // Half-length of the equator, (PI * 6378137) basically

/// Calculates the bearing between two geographic points.
///
/// The bearing is the angle measured clockwise from the North direction, from `pt1` to `pt2`.
/// The result is given in degrees.
///
/// # Arguments
///
/// * `pt1` - The first point (longitude and latitude in degrees).
/// * `pt2` - The second point (longitude and latitude in degrees).
///
/// # Returns
///
/// Returns the bearing from `pt1` to `pt2` in degrees.
///
/// # Example
///
/// ```
/// use micro_traffic_sim_core::geom::Point;
/// use micro_traffic_sim_core::geom::get_bearing;
/// let pt1 = Point::new(35.90434, 56.89028);
/// let pt2 = Point::new(35.90430, 56.89033);
/// let bearing = get_bearing(pt1, pt2);
/// ```
pub fn get_bearing(pt1: Point, pt2: Point) -> f64 {
    let lon1 = pt1.x;
    let lat1 = pt1.y;
    let lon2 = pt2.x;
    let lat2 = pt2.y;

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
/// use micro_traffic_sim_core::geom::Point;
/// use micro_traffic_sim_core::geom::gc_distance_pt;
/// let src = Point::new(35.90434, 56.89028);
/// let dst = Point::new(35.90434, 56.89028);
/// let distance = gc_distance_pt(src, dst);
/// println!("Great-circle distance: {} km", distance);
/// ```
pub fn gc_distance_pt(src: Point, dst: Point) -> f64 {
    gc_distance(src.x, src.y, dst.x, dst.y)
}

/// gc_distance
/// 
/// Computes the great-circle distance (orthodromic distance) between two points on the Earth's surface using the Haversine formula.
/// This formula calculates the shortest distance over the Earth's surface, assuming a perfect spherical Earth model.
///
/// # References
/// For detailed explanation of the Haversine formula, see:
/// [Great-circle distance - Wikipedia](https://en.wikipedia.org/wiki/Great-circle_distance#:~:text=The%20great%2Dcircle%20distance%2C%20orthodromic,line%20through%20the%20sphere's%20interior)
/// 
/// # Arguments
/// 
/// * `src_lon` - The longitude of the source point in degrees.
/// * `src_lat` - The latitude of the source point in degrees.
/// * `dst_lon` - The longitude of the destination point in degrees.
/// * `dst_lat` - The latitude of the destination point in degrees.
/// 
/// # Returns
/// 
/// A `f64` value representing the great-circle distance in meters between the source and destination points.
/// 
/// # Example
/// ```
/// use micro_traffic_sim_core::geom::gc_distance;
/// let distance = gc_distance(35.90434, 56.89028, 35.90434, 56.89028);
/// println!("Great-circle distance: {} km", distance);
/// ```
pub fn gc_distance(src_lon: f64, src_lat: f64, dst_lon: f64, dst_lat: f64) -> f64 {
    let lat1 = src_lat.to_radians();
	let lat2 = dst_lat.to_radians();
    let diff_lat = (dst_lat - src_lat).to_radians();
	let diff_lon = (dst_lon - src_lon).to_radians();
    let a = f64::powi(f64::sin(diff_lat / 2.0), 2) + f64::cos(lat1)*f64::cos(lat2)*f64::powi(f64::sin(diff_lon/2.0), 2);
    let c = 2.0 * f64::atan2(f64::sqrt(a), f64::sqrt(1.0 - a));
	c * EARTH_RADIUS_METERS
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_get_bearing() {
        let pt_from = Point::new(35.90434, 56.89028);
        let pt_to = Point::new(35.90430, 56.89033);
        let bearing = get_bearing(pt_from, pt_to);
        let correct_bearing = -23.605068777443574;

        // Assert that the absolute difference is less than a small threshold
        assert!(
            (bearing - correct_bearing).abs() < 0.001,
            "Bearing should be {}, but got {}",
            correct_bearing,
            bearing
        );
    }
    #[test]
    fn test_epsg_converter() {
        let precision = 10e-5;

        // Test point conversion from EPSG:4326 to EPSG:3857 and back
        let given_point_4326 = Point::new(37.61655751319856, 55.75163877328629);
        let expected_point_3857 = Point::new(4187456.027182254, 7509131.996742569);
        
        // Convert the point from EPSG:4326 to EPSG:3857
        let ans_point_3857 = convert_epsg4326_to_3857(given_point_4326.x, given_point_4326.y);
        
        // Check if the conversion to EPSG:3857 is correct
        assert!((expected_point_3857.x - ans_point_3857.0).abs() < precision, "Wrong X (longitude) in EPSG:3857. Should: {}. Got: {}", expected_point_3857.x, ans_point_3857.0);
        assert!((expected_point_3857.y - ans_point_3857.1).abs() < precision, "Wrong Y (latitude) in EPSG:3857. Should: {}. Got: {}", expected_point_3857.y, ans_point_3857.1);

        // // Reverse the conversion back to EPSG:4326
        let ans_reversed_point_4326 = convert_epsg3857_to_4326(ans_point_3857.0, ans_point_3857.1);
        
        // Check if the reverse conversion to EPSG:4326 is correct
        assert!((given_point_4326.x - ans_reversed_point_4326.0).abs() < precision, "Wrong X (longitude) in EPSG:3857. Should: {}. Got: {}", given_point_4326.x, ans_reversed_point_4326.0);
        assert!((given_point_4326.y - ans_reversed_point_4326.1).abs() < precision, "Wrong Y (latitude) in EPSG:3857. Should: {}. Got: {}", given_point_4326.y, ans_reversed_point_4326.1);

        // Test line conversion from EPSG:4326 to EPSG:3857 and back
        let given_line_4326 = vec![
            Point::new(37.61655751319856, 55.75163877328629),
            Point::new(37.61617406590727, 55.751456041561624),
        ];

        let expected_line_3857 = vec![
            Point::new(4187456.027182254, 7509131.996742569),
            Point::new(4187413.342025048, 7509095.852052931),
        ];

        // Convert the line from EPSG:4326 to EPSG:3857
        let ans_line_3857: Vec<Point> = given_line_4326.iter().map(|pt| {
            let (x, y) = convert_epsg4326_to_3857(pt.x, pt.y);
            Point::new(x, y)
        }).collect();

        // Check if the conversion for each point in the line is correct
        for (i, pt) in ans_line_3857.iter().enumerate() {
            assert!((expected_line_3857[i].x - pt.x).abs() < precision, 
                    "Wrong X (longitude) in EPSG:3857 at pos #{}. Should: {}. Got: {}", i, expected_line_3857[i].x, pt.x);
            assert!((expected_line_3857[i].y - pt.y).abs() < precision, 
                    "Wrong Y (latitude) in EPSG:3857 at pos #{}. Should: {}. Got: {}", i, expected_line_3857[i].y, pt.y);
        }

        // Reverse the line conversion from EPSG:3857 to EPSG:4326
        let ans_reversed_line_4326: Vec<Point> = ans_line_3857.iter().map(|pt| {
            let (x, y) = convert_epsg3857_to_4326(pt.x, pt.y);
            Point::new(x, y)
        }).collect();

        // Check if the reverse conversion for each point in the line is correct
        for (i, pt) in ans_reversed_line_4326.iter().enumerate() {
            assert!((given_line_4326[i].x - pt.x).abs() < precision, 
                    "Wrong X (longitude) in EPSG:3857 at pos #{}. Should: {}. Got: {}", i, given_line_4326[i].x, pt.x);
            assert!((given_line_4326[i].y - pt.y).abs() < precision, 
                    "Wrong Y (latitude) in EPSG:3857 at pos #{}. Should: {}. Got: {}", i, given_line_4326[i].y, pt.y);
        }
    }
    #[test]
    fn test_gc_distance() {
        let correct_distance = 634430.92026;
        let distance = gc_distance(37.61556, 55.75222, 30.31413, 59.93863);
        // Assert that the absolute difference is less than a small threshold
        assert!(
            (distance - correct_distance).abs() < 0.001,
            "Distance should be {}, but got {}",
            correct_distance,
            distance
        );

        let distance = gc_distance_pt(Point::new(37.61556, 55.75222), Point::new(30.31413, 59.93863));
        // Assert that the absolute difference is less than a small threshold
        assert!(
            (distance - correct_distance).abs() < 0.001,
            "Distance should be {}, but got {}",
            correct_distance,
            distance
        );
    }
}
