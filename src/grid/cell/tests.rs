use super::*;
use crate::geom::SRID;
use crate::geom::new_point;

#[test]
fn test_cells_gc_distance() {
    let correct_distance = 634430.92026;
    let cell1 = Cell::new(1)
        .with_point(new_point(37.61556, 55.75222, Some(SRID::WGS84)))
        .build();
    let cell2 = Cell::new(2)
        .with_point(new_point(30.31413, 59.93863, Some(SRID::WGS84)))
        .build();
    let distance = cell1.distance_to(&cell2);
    // Assert that the absolute difference is less than a small threshold
    assert!(
        (distance - correct_distance).abs() < 0.001,
        "Distance should be {}, but got {}",
        correct_distance,
        distance
    );
}
