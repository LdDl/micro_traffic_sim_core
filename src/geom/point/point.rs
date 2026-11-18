//! This module provides Point structure and its methods
use std::fmt;
use crate::geom::math::gc_distance;

/// Spatial Reference System Identifier
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SRID {
    Euclidean = 0,    // Default Euclidean space
    WGS84 = 4326,     // WGS84 coordinate system
}

#[derive(Debug, Clone, Copy)]
pub enum PointType {
    Euclidean(EuclideanPoint),
    WGS84(WGS84Point),
}

/// Point trait defines common operations for all point types
pub trait Point: fmt::Display {
    /// Get x coordinate (longitude for WGS84)
    fn x(&self) -> f64;
    
    /// Get y coordinate (latitude for WGS84)
    fn y(&self) -> f64;
    
    /// Calculate distance to another point
    fn distance_to(&self, other: &Self) -> f64;
    
    /// Get point's SRID
    fn srid(&self) -> SRID;
}

impl Point for PointType {
    /// Returns x coordinate (longitude for WGS84 points)
    ///
    /// # Examples
    /// 
    /// ```
    /// use micro_traffic_sim_core::geom::{Point, new_point};
    /// let point = new_point(4.0, 2.0, None);
    /// println!("X: {}", point.x());
    /// ```
    fn x(&self) -> f64 {
        match self {
            Self::Euclidean(p) => p.x(),
            Self::WGS84(p) => p.x(),
        }
    }
    
    /// Returns y coordinate (latitude for WGS84 points)
    ///     
    /// # Examples
    /// 
    /// ```
    /// use micro_traffic_sim_core::geom::{Point, new_point};
    /// let point = new_point(4.0, 2.0, None);
    /// println!("Y: {}", point.y());
    /// ```
    fn y(&self) -> f64 {
        match self {
            Self::Euclidean(p) => p.y(),
            Self::WGS84(p) => p.y(),
        }
    }
    
    /// Calculates distance to another point
    /// 
    /// # Arguments
    /// * `other` - Another point to calculate distance to.
    /// 
    /// # Returns
    /// A floating-point value representing the distance between two points.
    /// For Euclidean points, returns Euclidean distance. For WGS84 points, 
    /// returns great-circle distance in meters.
    /// 
    /// # Panics
    /// 
    /// Panics if attempting to calculate distance between points with different SRIDs.
    /// 
    /// # Examples
    /// 
    /// ```
    /// use micro_traffic_sim_core::geom::{Point, new_point, SRID};
    /// 
    /// // Euclidean distance
    /// let point1 = new_point(0.0, 0.0, None);
    /// let point2 = new_point(3.0, 4.0, None);
    /// let distance = point1.distance_to(&point2); // Returns 5.0
    /// 
    /// // Great-circle distance
    /// let moscow = new_point(37.6176, 55.7558, Some(SRID::WGS84));
    /// let spb = new_point(30.3141, 59.9386, Some(SRID::WGS84));
    /// let distance = moscow.distance_to(&spb); // Returns ~635km in meters
    /// ```
    fn distance_to(&self, other: &Self) -> f64 {
        match (self, other) {
            (Self::Euclidean(p1), Self::Euclidean(p2)) => p1.distance_to(p2),
            (Self::WGS84(p1), Self::WGS84(p2)) => p1.distance_to(p2),
            _ => panic!("Cannot calculate distance between points with different SRIDs"),
        }
    }
    
    /// Returns the spatial reference system identifier (SRID) of the point.
    /// 
    /// # Returns
    /// The SRID of the point.
    /// 
    /// # Examples
    /// 
    /// ```
    /// use micro_traffic_sim_core::geom::{Point, new_point};
    /// let point = new_point(4.0, 2.0, None);
    /// let srid = point.srid();
    /// println!("SRID: {:?}", srid);
    fn srid(&self) -> SRID {
        match self {
            Self::Euclidean(p) => p.srid(),
            Self::WGS84(p) => p.srid(),
        }
    }
}

impl fmt::Display for PointType {
    /// Formats the point with given SRID for display.
    /// 
    /// Returns a short, lowercase string representation suitable for
    /// logging, debugging, and user interfaces.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// use micro_traffic_sim_core::geom::{new_point, SRID};
    ///
    /// assert_eq!(format!("{}", new_point(4.0, 2.0, None)), "[x: 4.00000 y: 2.00000]");
    /// assert_eq!(format!("{}", new_point(37.61756, 55.75583, Some(SRID::WGS84))), "[lon: 37.61756 lat: 55.75583]");
    /// ```
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Euclidean(p) => write!(f, "[x: {:.5} y: {:.5}]", p.x(), p.y()),
            Self::WGS84(p) => write!(f, "[lon: {:.5} lat: {:.5}]", p.x(), p.y()),
        }
    }
}

/// Creates a new `Point` with the given longitude and latitude.
///
/// # Arguments
///
/// * `x` - Either just X in Euclidean space or the longitude of the point in degrees for WGS84.
/// * `y` - Either just Y in Euclidean space or the latitude of the point in degrees for WGS84.
/// * `srid` - The spatial reference system identifier (SRID) of the point. Default is `SRID::Euclidean`.
///
/// # Returns
///
/// A new `Point` representing the geographic coordinates.
/// 
/// # Example
///
/// ```
/// use micro_traffic_sim_core::geom::{new_point, SRID};
/// let point_wgs84 = new_point(35.90434, 56.89028, Some(SRID::WGS84));
/// let point_eucliden_explicit = new_point(4.0, 2.0, Some(SRID::Euclidean));
/// let point_eucliden_implicit = new_point(4.0, 2.0, None);
/// ```
///
/// ```rust
/// use micro_traffic_sim_core::geom::{new_point, SRID, Point};
///
/// // Euclidean point (default)
/// let grid_point = new_point(10.0, 20.0, None);
///
/// // Geographic point (WGS84)
/// let moscow = new_point(37.6176, 55.7558, Some(SRID::WGS84));
/// let spb = new_point(30.3141, 59.9386, Some(SRID::WGS84));
///
/// // Calculate distance
/// // Great-circle distance in meters
/// let distance = moscow.distance_to(&spb);
/// ```
pub fn new_point(x: f64, y: f64, srid: Option<SRID>) -> PointType {
    match srid.unwrap_or(SRID::Euclidean) {
        SRID::WGS84 => PointType::WGS84(WGS84Point { lon: x, lat: y }),
        _ => PointType::Euclidean(EuclideanPoint { x, y }),
    }
}

/// A structure representing a point in the Euclidean space.
#[derive(Debug, Clone, Copy)]
pub struct EuclideanPoint {
    pub x: f64,
    pub y: f64,
}

impl Point for EuclideanPoint {
    fn x(&self) -> f64 { self.x }
    fn y(&self) -> f64 { self.y }
    fn srid(&self) -> SRID { SRID::Euclidean }
    fn distance_to(&self, other: &Self) -> f64 {
        ((self.x - other.x()).powi(2) + (self.y - other.y()).powi(2)).sqrt()
    }
}

impl fmt::Display for EuclideanPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[x: {:.5} y: {:.5}]", self.x, self.y)
    }
}

/// A structure representing a point in the WGS84 coordinate system.
#[derive(Debug, Clone, Copy)]
pub struct WGS84Point {
    lon: f64,
    lat: f64,
}

impl Point for WGS84Point {
    fn x(&self) -> f64 { self.lon }
    fn y(&self) -> f64 { self.lat }
    fn srid(&self) -> SRID { SRID::WGS84 }
    fn distance_to(&self, other: &Self) -> f64 {
        gc_distance(self.lon, self.lat, other.x(), other.y())
    }
}

impl fmt::Display for WGS84Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[lon: {:.5} lat: {:.5}]", self.lon, self.lat)
    }
}
