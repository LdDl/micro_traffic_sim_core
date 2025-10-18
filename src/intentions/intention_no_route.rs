use crate::maneuver::LaneChangeType;
use crate::{
    grid::{
        cell::{Cell, CellID},
        road_network::GridRoads,
    },
    shortest_path::{heuristics::heuristic, path::Path},
};
use std::fmt;

#[derive(Debug, Clone)]
pub enum NoRouteError {
    NoVertexFound { cell_id: CellID },
    NoExitPath,
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
            }
        }
    }
}

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
        return Err(NoRouteError::NoExitPath);
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
