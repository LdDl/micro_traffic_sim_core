//! Scalar geometry calculations independent of point types.

const EARTH_RADIUS_METERS: f64 = 6371000.0;

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
    let lat1: f64 = src_lat.to_radians();
    let lat2 = dst_lat.to_radians();
    let diff_lat = (dst_lat - src_lat).to_radians();
    let diff_lon = (dst_lon - src_lon).to_radians();
    let a = f64::powi(f64::sin(diff_lat / 2.0), 2)
        + f64::cos(lat1) * f64::cos(lat2) * f64::powi(f64::sin(diff_lon / 2.0), 2);
    let c = 2.0 * f64::atan2(f64::sqrt(a), f64::sqrt(1.0 - a));
    c * EARTH_RADIUS_METERS
}
