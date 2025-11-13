use crate::maneuver::LaneChangeType;
use crate::{
    grid::{
        cell::{Cell, CellID},
        road_network::GridRoads,
    },
    shortest_path::{heuristics::heuristic, path::Path},
};
use std::fmt;

/// Error types for handling no-route-found scenarios.
#[derive(Debug, Clone)]
pub enum NoRouteError {
    /// Indicates that the destination cell vertex was not found in the graph.
    NoVertexFound {
        /// The ID of the cell that could not be found
        cell_id: CellID
    },
    /// Indicates that the there is not exit path
    NoExitPath,
    /// Dead end reached
    DeadEndReached {
        /// The ID of the cell where dead end was reached
        cell_id: CellID
    }
}

impl fmt::Display for NoRouteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NoRouteError::NoVertexFound { cell_id } => {
                write!(
                    f,
                    "Can't find destination cell for vehicle in graph. Vertex ID is '{}'",
                    cell_id
                )
            }
            NoRouteError::NoExitPath => {
                write!(f, "Vehicle should have exited network in previous step")
            },
            NoRouteError::DeadEndReached { cell_id } => {
                write!(f, "Dead end reached at cell ID '{}'", cell_id)
            }
        }
    }
}

/// Handles cases where no route to the destination is found.
///
/// Returns a single-step path in a possible direction (forward, left, or right),
/// or an error if no exit is possible.
pub fn process_no_route_found<'a>(
    current_cell: &'a Cell,
    net: &'a GridRoads,
) -> Result<Path<'a>, NoRouteError> {
    // Try to find possible move direction
    let (destination_cell_id, maneuver) = if current_cell.get_forward_id() >= 0 {
        (current_cell.get_forward_id(), LaneChangeType::NoChange)
    } else if current_cell.get_right_id() >= 0 {
        (current_cell.get_right_id(), LaneChangeType::ChangeRight)
    } else if current_cell.get_left_id() >= 0 {
        (current_cell.get_left_id(), LaneChangeType::ChangeLeft)
    } else {
        return Err(NoRouteError::DeadEndReached {
            cell_id: current_cell.get_id(),
        });
    };

    // Get destination cell from grid
    let destination_cell =
        net.get_cell(&destination_cell_id)
            .ok_or(NoRouteError::NoVertexFound {
                cell_id: destination_cell_id,
            })?;

    // Create path with single step
    Ok(Path::new(
        vec![current_cell, destination_cell],
        // Maneuvers is a slice of maneuvers between cells, so it's len = len(Vertices) - 1
        vec![maneuver],
        heuristic(current_cell, destination_cell), // Default cost for single step
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::cell::Cell;
    use crate::grid::road_network::GridRoads;

    #[test]
    fn test_process_no_route_found_deadend() {
        // Create a cell with no exits
        let cell = Cell::new(1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_left_node(-1)
            .build();
        let mut net = GridRoads::new();
        net.add_cell(cell.clone());

        // Should return DeadEndReached error
        let result = process_no_route_found(&cell, &net);
        match result {
            Err(NoRouteError::DeadEndReached { cell_id }) => {
                assert_eq!(cell_id, cell.get_id());
            }
            other => panic!("Expected DeadEndReached error, got {:?}", other),
        }
    }
}