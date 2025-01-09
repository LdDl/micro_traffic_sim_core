use crate::grid::cell::Cell;

/// Calculates the heuristic distance (Euclidean) between two cells.
///
/// # Arguments
/// * `start` - The starting cell.
/// * `end` - The target cell.
///
/// # Returns
/// A floating-point value representing the heuristic distance.
/// # Example
///
/// ```
/// use micro_traffic_sim_core::geom::Point;
/// use micro_traffic_sim_core::grid::cell::Cell;
/// use micro_traffic_sim_core::shortest_path::heuristics::heuristic;
/// let cell1 = Cell::new(1)
///     .with_point(Point::new(37.61556, 55.75222))
///     .build();
/// let cell2 = Cell::new(2)
///     .with_point(Point::new(30.31413, 59.93863))
///     .build();
/// println!("heuristics value: {}", heuristic(&cell1, &cell2));
/// ```
pub fn heuristic(start: &Cell, end: &Cell) -> f64 {
    start.distance_to(end)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Point;
    #[test]
    fn test_heuristic() {
        let correct_distance = 634430.92026;
        let cell1 = Cell::new(1)
            .with_point(Point::new(37.61556, 55.75222))
            .build();
        let cell2 = Cell::new(2)
            .with_point(Point::new(30.31413, 59.93863))
            .build();
        let distance = heuristic(&cell1, &cell2);
        // Assert that the absolute difference is less than a small threshold
        assert!(
            (distance - correct_distance).abs() < 0.001,
            "Distance should be {}, but got {}",
            correct_distance,
            distance
        );
    }
}
