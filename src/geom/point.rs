//! This module provides Point structure and its methods

use std::fmt;

/// A structure representing a point on the Earth's surface.
/// This struct holds the geographic coordinates (longitude and latitude).
///
/// # Basic usage:
///
/// ```
/// use micro_traffic_sim_core::geom::Point;
/// let point = Point::new(35.90434, 56.89028);
/// println!("{}", point);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Point {
    // Longitude
    pub x: f64,
    // Latitude
    pub y: f64,
}

impl Point {
    /// Creates a new `Point` with the given longitude and latitude.
    ///
    /// # Arguments
    ///
    /// * `x` - The longitude of the point in degrees.
    /// * `y` - The latitude of the point in degrees.
    ///
    /// # Returns
    ///
    /// A new `Point` representing the geographic coordinates.
    /// 
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::geom::Point;
    /// let point = Point::new(35.90434, 56.89028);
    /// ```
    pub fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[x: {:.5} y: {:.5}]", self.x, self.y)
    }
}
