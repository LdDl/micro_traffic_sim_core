use super::*;
use crate::geom::SRID;

// Omitting the SRID creates a Euclidean point and preserves both coordinates.
#[test]
fn test_euclidean_point_creation() {
    let point = new_point(3.0, 4.0, None);
    assert_eq!(point.x(), 3.0);
    assert_eq!(point.y(), 4.0);
    assert_eq!(point.srid(), SRID::Euclidean);
}

// An explicit WGS84 SRID preserves longitude, latitude, and the geographic point type.
#[test]
fn test_wgs84_point_creation() {
    let point = new_point(37.6176, 55.7558, Some(SRID::WGS84));
    assert_eq!(point.x(), 37.6176);
    assert_eq!(point.y(), 55.7558);
    assert_eq!(point.srid(), SRID::WGS84);
}

// The displacement (3, 4) has Euclidean distance 5, within the stated tolerance.
#[test]
fn test_euclidean_distance() {
    let point1 = new_point(0.0, 0.0, None);
    let point2 = new_point(3.0, 4.0, None);
    let distance = point1.distance_to(&point2);
    assert!(
        (distance - 5.0).abs() < 0.001,
        "Expected 5.0, got {}",
        distance
    );
}

// The Moscow-to-Saint Petersburg coordinates produce about 634.4 km, with a 1 km
// tolerance.
#[test]
fn test_wgs84_distance() {
    // Moscow to St. Petersburg
    let moscow = new_point(37.6176, 55.7558, Some(SRID::WGS84));
    let spb = new_point(30.3141, 59.9386, Some(SRID::WGS84));
    let distance = moscow.distance_to(&spb);
    // Expected distance ~635km
    assert!(
        (distance - 634430.92).abs() < 1000.0,
        "Expected ~634430m, got {}",
        distance
    );
}

// A distance between Euclidean and WGS84 points is rejected with the expected panic
// message.
#[test]
#[should_panic(expected = "Cannot calculate distance between points with different SRIDs")]
fn test_mixed_srid_distance_panics() {
    let euclidean_point = new_point(0.0, 0.0, None);
    let wgs84_point = new_point(37.6176, 55.7558, Some(SRID::WGS84));
    euclidean_point.distance_to(&wgs84_point);
}

// Euclidean display uses x/y labels and the expected decimal representation.
#[test]
fn test_euclidean_point_display() {
    let point = new_point(3.14159, 2.71828, None);
    let display_str = format!("{}", point);
    assert_eq!(display_str, "[x: 3.14159 y: 2.71828]");
}

// Geographic display uses lon/lat labels and the expected decimal representation.
#[test]
fn test_wgs84_point_display() {
    let point = new_point(37.61756, 55.75583, Some(SRID::WGS84));
    let display_str = format!("{}", point);
    assert_eq!(display_str, "[lon: 37.61756 lat: 55.75583]");
}

// A directly constructed EuclideanPoint exposes the same coordinates and SRID as the point
// interface.
#[test]
fn test_euclidean_point_direct_creation() {
    let point = EuclideanPoint { x: 10.0, y: 20.0 };
    assert_eq!(point.x(), 10.0);
    assert_eq!(point.y(), 20.0);
    assert_eq!(point.srid(), SRID::Euclidean);
}

// The factory selects the enum variant corresponding to each explicit SRID.
#[test]
fn test_point_type_enum_matching() {
    let euclidean = new_point(1.0, 2.0, Some(SRID::Euclidean));
    let wgs84 = new_point(37.0, 55.0, Some(SRID::WGS84));

    match euclidean {
        PointType::Euclidean(_) => {}
        _ => panic!("Expected Euclidean variant"),
    }

    match wgs84 {
        PointType::WGS84(_) => {}
        _ => panic!("Expected WGS84 variant"),
    }
}

// The SRID discriminants remain 0 for Euclidean coordinates and 4326 for WGS84.
#[test]
fn test_srid_numeric_codes() {
    assert_eq!(SRID::Euclidean as i32, 0);
    assert_eq!(SRID::WGS84 as i32, 4326);
}

// Identical geographic coordinates have zero distance within the numerical tolerance.
#[test]
fn test_zero_distance() {
    let point1 = new_point(37.6176, 55.7558, Some(SRID::WGS84));
    let point2 = new_point(37.6176, 55.7558, Some(SRID::WGS84));
    let distance = point1.distance_to(&point2);
    assert!((distance).abs() < 0.001, "Expected ~0, got {}", distance);
}
