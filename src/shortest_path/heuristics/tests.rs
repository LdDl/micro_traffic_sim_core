use super::*;
use crate::geom::{SRID, new_point};
use crate::grid::cell::Cell;

// With maximum speed 1, the travel-time heuristic equals the reference geographic
// distance.
#[test]
fn test_unit_speed_heuristic_matches_geographic_distance() {
    let cell1 = Cell::new(1)
        // Moscow
        .with_point(new_point(37.61556, 55.75222, Some(SRID::WGS84)))
        .build();
    let cell2 = Cell::new(2)
        // Saint Petersburg
        .with_point(new_point(30.31413, 59.93863, Some(SRID::WGS84)))
        .build();

    // max_speed = 1.0 recovers the raw geometric distance.
    let distance = heuristic(&cell1, &cell2, 1.0);
    let correct_distance = 634430.92026;

    assert!(
        (distance - correct_distance).abs() < 0.001,
        "Heuristic value should be {}, but got {}",
        correct_distance,
        distance
    );
}
