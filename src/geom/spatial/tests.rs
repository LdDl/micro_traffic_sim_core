use crate::geom::math::gc_distance;
use crate::geom::point::{Point, PointType, SRID, new_point};

use super::*;

// A short geographic displacement matches the signed reference bearing in degrees.
#[test]
fn test_get_bearing() {
    let pt_from = new_point(35.90434, 56.89028, Some(SRID::WGS84));
    let pt_to = new_point(35.90430, 56.89033, Some(SRID::WGS84));
    let bearing = get_bearing(&pt_from, &pt_to);
    let correct_bearing = -23.605068777443574;

    // Assert that the absolute difference is less than a small threshold
    assert!(
        (bearing - correct_bearing).abs() < 0.001,
        "Bearing should be {}, but got {}",
        correct_bearing,
        bearing
    );
}

// Point and two-point line coordinates match the EPSG:3857 references and recover their
// EPSG:4326 inputs after inverse conversion.
#[test]
fn test_epsg_converter() {
    let precision = 10e-5;

    // Test point conversion from EPSG:4326 to EPSG:3857 and back
    let given_point_4326 = new_point(37.61655751319856, 55.75163877328629, Some(SRID::WGS84));
    let expected_point_3857 = new_point(4187456.027182254, 7509131.996742569, None);

    // Convert the point from EPSG:4326 to EPSG:3857
    let ans_point_3857 = convert_epsg4326_to_3857(given_point_4326.x(), given_point_4326.y());

    // Check if the conversion to EPSG:3857 is correct
    assert!(
        (expected_point_3857.x() - ans_point_3857.0).abs() < precision,
        "Wrong X (longitude) in EPSG:3857. Should: {}. Got: {}",
        expected_point_3857.x(),
        ans_point_3857.0
    );
    assert!(
        (expected_point_3857.y() - ans_point_3857.1).abs() < precision,
        "Wrong Y (latitude) in EPSG:3857. Should: {}. Got: {}",
        expected_point_3857.y(),
        ans_point_3857.1
    );

    // Reverse the conversion back to EPSG:4326
    let ans_reversed_point_4326 = convert_epsg3857_to_4326(ans_point_3857.0, ans_point_3857.1);

    // Check if the reverse conversion to EPSG:4326 is correct
    assert!(
        (given_point_4326.x() - ans_reversed_point_4326.0).abs() < precision,
        "Wrong X (longitude) in EPSG:3857. Should: {}. Got: {}",
        given_point_4326.x(),
        ans_reversed_point_4326.0
    );
    assert!(
        (given_point_4326.y() - ans_reversed_point_4326.1).abs() < precision,
        "Wrong Y (latitude) in EPSG:3857. Should: {}. Got: {}",
        given_point_4326.y(),
        ans_reversed_point_4326.1
    );

    // Test line conversion from EPSG:4326 to EPSG:3857 and back
    let given_line_4326 = vec![
        new_point(37.61655751319856, 55.75163877328629, Some(SRID::WGS84)),
        new_point(37.61617406590727, 55.751456041561624, Some(SRID::WGS84)),
    ];

    let expected_line_3857 = vec![
        new_point(4187456.027182254, 7509131.996742569, None),
        new_point(4187413.342025048, 7509095.852052931, None),
    ];

    // Convert the line from EPSG:4326 to EPSG:3857
    let ans_line_3857: Vec<PointType> = given_line_4326
        .iter()
        .map(|pt| {
            let (x, y) = convert_epsg4326_to_3857(pt.x(), pt.y());
            new_point(x, y, None)
        })
        .collect();

    // Check if the conversion for each point in the line is correct
    for (i, pt) in ans_line_3857.iter().enumerate() {
        assert!(
            (expected_line_3857[i].x() - pt.x()).abs() < precision,
            "Wrong X (longitude) in EPSG:3857 at pos #{}. Should: {}. Got: {}",
            i,
            expected_line_3857[i].x(),
            pt.x()
        );
        assert!(
            (expected_line_3857[i].y() - pt.y()).abs() < precision,
            "Wrong Y (latitude) in EPSG:3857 at pos #{}. Should: {}. Got: {}",
            i,
            expected_line_3857[i].y(),
            pt.y()
        );
    }

    // Reverse the line conversion from EPSG:3857 to EPSG:4326
    let ans_reversed_line_4326: Vec<PointType> = ans_line_3857
        .iter()
        .map(|pt| {
            let (x, y) = convert_epsg3857_to_4326(pt.x(), pt.y());
            new_point(x, y, None)
        })
        .collect();

    // Check if the reverse conversion for each point in the line is correct
    for (i, pt) in ans_reversed_line_4326.iter().enumerate() {
        assert!(
            (given_line_4326[i].x() - pt.x()).abs() < precision,
            "Wrong X (longitude) in EPSG:3857 at pos #{}. Should: {}. Got: {}",
            i,
            given_line_4326[i].x(),
            pt.x()
        );
        assert!(
            (given_line_4326[i].y() - pt.y()).abs() < precision,
            "Wrong Y (latitude) in EPSG:3857 at pos #{}. Should: {}. Got: {}",
            i,
            given_line_4326[i].y(),
            pt.y()
        );
    }
}

// The scalar and point-based great-circle functions match the same distance reference in
// meters.
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

    let distance = gc_distance_pt(
        &new_point(37.61556, 55.75222, Some(SRID::WGS84)),
        &new_point(30.31413, 59.93863, Some(SRID::WGS84)),
    );
    // Assert that the absolute difference is less than a small threshold
    assert!(
        (distance - correct_distance).abs() < 0.001,
        "Distance should be {}, but got {}",
        correct_distance,
        distance
    );
}
