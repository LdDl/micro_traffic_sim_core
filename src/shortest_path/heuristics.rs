use crate::grid::cell::Cell;

/// Admissible A* heuristic in TRAVEL-TIME units: a lower bound on the time to get
/// from `start` to `end`.
///
/// Routing cost is travel time (see [`edge_time`]); to stay admissible the heuristic
/// must never overestimate. The straight-line distance divided by the network's
/// maximum speed is a valid lower bound, because no edge is traversed faster than
/// `max_speed`. Pass `max_speed = 1.0` to recover the raw geometric distance.
///
/// # Arguments
/// * `start` - The starting cell.
/// * `end` - The target cell.
/// * `max_speed` - The maximum speed across the network (cells/tick); use the value
///   from [`crate::grid::road_network::GridRoads::get_max_speed`].
///
/// # Example
///
/// ```
/// use micro_traffic_sim_core::geom::{new_point, SRID};
/// use micro_traffic_sim_core::grid::cell::Cell;
/// use micro_traffic_sim_core::shortest_path::heuristics::heuristic;
/// let cell1 = Cell::new(1)
///     .with_point(new_point(37.61556, 55.75222, Some(SRID::WGS84)))
///     .build();
/// let cell2 = Cell::new(2)
///     .with_point(new_point(30.31413, 59.93863, Some(SRID::WGS84)))
///     .build();
/// let heuristic_val = heuristic(&cell1, &cell2, 1.0);
/// println!("Heuristic: {}", heuristic_val);
/// ```
pub fn heuristic(start: &Cell, end: &Cell, max_speed: f64) -> f64 {
    start.distance_to(end) / max_speed
}

/// Travel time (free-flow) of the edge from `from` to `to`.
///
/// Cost = edge length / speed limit of the source cell. `speed_limit` is in
/// cells/tick (NaSch); an unset/zero limit is clamped to 1 so the edge stays
/// passable and the cost finite. This is the per-edge `g` cost used by A*; the
/// matching admissible heuristic is [`heuristic`].
pub fn edge_time(from: &Cell, to: &Cell) -> f64 {
    let speed = (from.get_speed_limit() as f64).max(1.0);
    from.distance_to(to) / speed
}

/// An A* heuristic: an **admissible** (never-overestimating) lower bound on the
/// travel time from `from` to `goal`. The router stays optimal as long as the
/// estimate never exceeds the true remaining travel time.
///
/// This is a trait so alternative heuristics (e.g. landmark/ALT, see
/// [`crate::shortest_path::landmarks`]) can be swapped in - and the landmark
/// machinery can later be extracted into a separate crate without touching the
/// router, which only depends on this trait.
pub trait Heuristic {
    /// Lower-bound travel time from `from` to `goal`.
    fn estimate(&self, from: &Cell, goal: &Cell) -> f64;
}

/// The built-in geometric heuristic: straight-line distance divided by the
/// network's maximum speed (so it never overestimates travel time).
#[derive(Debug, Clone, Copy)]
pub struct GeometricHeuristic {
    /// Maximum speed across the network (cells/tick); the divisor that keeps the
    /// estimate admissible.
    pub max_speed: f64,
}

impl GeometricHeuristic {
    /// Builds a geometric heuristic; `max_speed` is clamped to a small positive value.
    pub fn new(max_speed: f64) -> Self {
        GeometricHeuristic {
            max_speed: max_speed.max(1e-9),
        }
    }
}

impl Heuristic for GeometricHeuristic {
    fn estimate(&self, from: &Cell, goal: &Cell) -> f64 {
        heuristic(from, goal, self.max_speed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{new_point, SRID};
    use crate::grid::cell::Cell;

    #[test]
    fn test_heuristic() {
        let cell1 = Cell::new(1)
            .with_point(new_point(37.61556, 55.75222, Some(SRID::WGS84))) // Moscow
            .build();
        let cell2 = Cell::new(2)
            .with_point(new_point(30.31413, 59.93863, Some(SRID::WGS84))) // Saint Petersburg
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
}

