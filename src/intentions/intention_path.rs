use crate::maneuver::LaneChangeType;
use crate::{
    agents::VehicleID,
    grid::{
        cell::{Cell, CellID, CellState},
    },
    shortest_path::path::Path,
};
use std::collections::HashMap;

/// Contains details about a vehicle's observable movement along a path.
///
/// Includes the desired maneuver, last cell state, trimmed path,
/// and flags for obstacles, speed limits, and maneuvers.
#[derive(Debug, Clone)]
pub struct ObservablePath<'a> {
    /// Desired lane change maneuver
    pub wanted_maneuver: LaneChangeType,
    /// State of the last cell in path
    pub last_cell_state: CellState,
    /// Path vertices (cells) after trimming
    pub trimmed_path: Vec<&'a Cell>,
    /// Indicates if another vehicle is on the path
    pub has_vehicle_on_path: bool,
    /// Indicates if speed limit was reached
    pub speed_limit_reached: bool,
    /// Indicates if movement was stopped due to a maneuver
    pub stopped_on_maneuver: bool,
    /// Indicates if movement was stopped due to speed restrictions
    pub stopped_speed_possible: bool,
}

/// Analyzes a path and determines how far a vehicle can move.
///
/// Trims the path based on speed, obstacles, maneuvers, and cell states.
/// Returns details about the possible movement and any stopping conditions.
///
/// # Arguments
/// * `shortest_path` - Reference to path to process
/// * `speed_possible` - Maximum possible speed
/// * `current_state` - Current occupancy state mapping cells to vehicles
///
/// # Returns
/// Returns ObservablePath containing movement analysis
///
/// # Examples
///
/// ```
/// use micro_traffic_sim_core::{
///    grid::{cell::{Cell, CellState}},
///    shortest_path::path::Path,
///    maneuver::LaneChangeType,
///    intentions::{process_path, ObservablePath},
/// };
/// use std::collections::HashMap;
/// let cell1 = Cell::new(1).with_speed_limit(2).build();
/// let cell2 = Cell::new(2).with_speed_limit(2).build();
/// let cell3 = Cell::new(3).with_speed_limit(2).build();
/// let cell4 = Cell::new(4).with_speed_limit(2).build();
///
/// let mut path = Path::new(
///     vec![&cell1, &cell2, &cell3, &cell4],
///     vec![LaneChangeType::NoChange, LaneChangeType::NoChange, LaneChangeType::ChangeLeft],
///     10.0
/// );
///
/// let mut current_state = HashMap::new();
/// let observable_path = process_path(&mut path, 2, 4, &current_state);
///
/// println!("Observable path without obstacles: {:?}", observable_path); // Should print 3 cells forward
///
/// current_state.insert(cell3.get_id(), 1);
/// let observable_path = process_path(&mut path, 2, 4, &current_state);
///
/// println!("Observable path with obstacles: {:?}", observable_path); // Should print 1 cell forward
/// ```
pub fn process_path<'a>(
    shortest_path: &'a mut Path,
    speed_possible: i32,
    destination: CellID,
    current_state: &HashMap<CellID, VehicleID>,
) -> ObservablePath<'a> {
    // Remove first cell from path since it's vehicle's position
    shortest_path.vertices_mut().remove(0);
    let speed_limit = (speed_possible as usize - 1).min(shortest_path.vertices().len() - 1);
    // Remove last cell if path has more than 1 cell and last cell in possible path (untill speed
    // limit would has been reached) is not vehicle's destination
    if shortest_path.vertices().len() > 1
        && shortest_path.vertices()[speed_limit].get_id() != destination
    {
        // @todo: is this neccessary?
        // shortest_path.vertices_mut().pop();
        // shortest_path.maneuvers_mut().pop();
    }

    let maneuvers = shortest_path.maneuvers();
    let vertices = shortest_path.vertices();
    let mut wanted_maneuver = LaneChangeType::NoChange;
    let mut last_cell_state = CellState::Free;

    // Find max number of cells vehicle can move forward
    let mut success_forward_movement = 0;

    let mut has_vehicle_on_path = false;
    let mut speed_limit_reached = false;
    let mut stopped_on_maneuver = false;
    let mut stopped_speed_possible = false;

    for (i, cell) in vertices.iter().enumerate() {
        let maneuver = maneuvers[i];
        if maneuver != LaneChangeType::NoChange {
            if success_forward_movement == 0 {
                wanted_maneuver = maneuver;
            }
            stopped_on_maneuver = true;
            break;
        }

        // Check if cell is occupied by another vehicle
        if let Some(&vehicle_id) = current_state.get(&cell.get_id()) {
            has_vehicle_on_path = true;
            break;
        }

        // Check traffic light state
        if cell.get_state() != CellState::Free {
            last_cell_state = cell.get_state();
            break;
        }

        // Check speed limit
        if speed_possible > cell.get_speed_limit() {
            if success_forward_movement == 0 {
                // Ensure at least one cell is moved into if speed limit is lower than possible speed
                // Happens when very first cell has lower speed limit than vehicle's speed
                success_forward_movement = 1;
            }
            speed_limit_reached = true;
            break;
        }

        success_forward_movement += 1;
        if success_forward_movement >= speed_possible {
            break;
        }
    }
    ObservablePath {
        wanted_maneuver,
        last_cell_state,
        trimmed_path: vertices[..success_forward_movement as usize].to_vec(),
        has_vehicle_on_path,
        speed_limit_reached,
        stopped_on_maneuver,
        stopped_speed_possible,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_observable_path() {
        let speed_limit = 4;
        let cell1 = Cell::new(1).with_speed_limit(speed_limit).build();
        let cell2 = Cell::new(2).with_speed_limit(speed_limit).build();
        let mut cell3 = Cell::new(3).with_speed_limit(speed_limit).build();
        let cell4 = Cell::new(4).with_speed_limit(speed_limit).build();
        let cell5 = Cell::new(5).with_speed_limit(speed_limit).build();

        let mut path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::ChangeLeft,
            ],
            10.0,
        );

        let mut current_state = HashMap::new();
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Free,
            trimmed_path: vec![&cell2, &cell3, &cell4],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: false,
            stopped_speed_possible: false,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }

        // Add vehicle as obstacle
        current_state.insert(cell4.get_id(), 1);
        // Update path since it has been mutated in process_path function
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::ChangeLeft,
            ],
            10.0,
        );
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Free,
            trimmed_path: vec![&cell2, &cell3],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: false,
            stopped_speed_possible: false,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }
        // Clean current states
        current_state.remove(&cell4.get_id());

        // Add maneuver in the middle of the path
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::ChangeLeft,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
            ],
            10.0,
        );
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Free,
            trimmed_path: vec![&cell2],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: true,
            stopped_speed_possible: false,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }

        // Add manuever to the start of the path
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::ChangeLeft,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
            ],
            10.0,
        );
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::ChangeLeft,
            last_cell_state: CellState::Free,
            trimmed_path: vec![],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: true,
            stopped_speed_possible: false,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }

        // Check speed restriction
        let vehicle_speed = 1;
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
            ],
            10.0,
        );
        let observable_path =
            process_path(&mut path, vehicle_speed, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Free,
            trimmed_path: vec![&cell2],
            has_vehicle_on_path: false,
            speed_limit_reached: true,
            stopped_on_maneuver: false,
            stopped_speed_possible: false,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }

        // Check traffic light banned state
        cell3.set_state(CellState::Banned);
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
            ],
            10.0,
        );
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Banned,
            trimmed_path: vec![&cell2],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: false,
            stopped_speed_possible: false,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }
    }
}
