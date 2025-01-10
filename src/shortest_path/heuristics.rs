use crate::grid::cell::Cell;

/// Calculates the heuristic parameter between two cells.
///
/// # Arguments
/// * `start` - The starting cell.
/// * `end` - The target cell.
///
/// # Returns
/// A floating-point value representing the heuristic value. In current implmentation it is
/// shortcut to calling .distance_to for two given cells basically.
///
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
/// let heuristic_val = heuristic(&cell1, &cell2);
/// println!("Heuristic: {}", heuristic_val);
/// ```
pub fn heuristic(start: &Cell, end: &Cell) -> f64 {
    start.distance_to(end)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Point;
    use crate::grid::cell::Cell;

    #[test]
    fn test_heuristic() {
        let cell1 = Cell::new(1)
            .with_point(Point::new(37.61556, 55.75222)) // Moscow
            .build();
        let cell2 = Cell::new(2)
            .with_point(Point::new(30.31413, 59.93863)) // Saint Petersburg
            .build();

        let distance = heuristic(&cell1, &cell2);
        let correct_distance = 634430.92026;

        assert!(
            (distance - correct_distance).abs() < 0.001,
            "Heuristic value should be {}, but got {}",
            correct_distance,
            distance
        );
    }
}

