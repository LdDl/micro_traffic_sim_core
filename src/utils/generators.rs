use crate::geom::{new_point};
use crate::grid::cell::Cell;
use crate::grid::zones::ZoneType;

/// Generates a multi-lane straight road with lane change connections.
///
/// Creates a series of connected lanes with proper forward movement and lane changing
/// capabilities. Each lane has Birth (start), Common (middle), and Death (end) zones.
///
/// # Arguments
///
/// - `length_meters` - Total road length in meters
/// - `step` - Length of each cell in meters  
/// - `lanes_num` - Number of parallel lanes to create
///
/// # Returns
///
/// Vector of [`Cell`] objects representing the complete road network with:
/// - Forward connections within each lane
/// - Left/right connections between adjacent lanes
/// - Proper zone types (Birth → Common → Death)
///
/// # Layout
///
/// ```text
/// Lane 2: [B] → [C] → [C] → ... → [D]
///            ↘ ↗   ↘ ↗  ↘ ↗ 
///            ↗ ↘   ↗ ↘  ↗ ↘       
/// Lane 1: [B] → [C] → [C] → ... → [D]
/// 
/// B = Birth zone, C = Common zone, D = Death zone
/// → = Forward connection, ↘, ↗ = Lane change connection
/// ```
///
/// # Examples
///
/// ```rust
/// use micro_traffic_sim_core::utils::generators::generate_one_lane_cells;
/// 
/// // Single lane, 50m road, 5m per cell = 10 cells
/// let single_lane = generate_one_lane_cells(50.0, 5.0, 1);
/// assert_eq!(single_lane.len(), 10);
/// 
/// // Two lanes, 100m road, 10m per cell = 20 cells total
/// let dual_lane = generate_one_lane_cells(100.0, 10.0, 2);
/// assert_eq!(dual_lane.len(), 20); // 2 lanes × 10 cells each
/// ```
///
/// # Lane Change Connections
///
/// - **Left connections**: Right (lower) lane cells can change to left (upper) lane
/// - **Right connections**: Left (upper) lane cells can change to right (lower) lane
/// - **No connections at ends**: Birth and Death cells don't allow lane changes
///
/// # Cell Properties
///
/// - **Cell IDs**: Sequential numbering (1, 2, 3, ...)
/// - **Coordinates**: (x, y) where x = distance, y = lane number
/// - **Speed limit**: Fixed at 3 cells/step
/// - **Meso links**: Each lane gets unique link ID
pub fn generate_one_lane_cells(length_meters: f64, step: f64, lanes_num: usize) -> Vec<Cell> {
    let single_lane_cells_num = (length_meters / step).ceil() as usize;
    let mut cells_counter: i64 = 1;
    let mut lanes_counter: i64 = 1;
    let mut last_added_lane: Vec<Cell> = Vec::new();
    let mut all_lanes_cells: Vec<Cell> = Vec::with_capacity(lanes_num * single_lane_cells_num);

    for j in 0..lanes_num {
        let mut lane_cells = Vec::with_capacity(single_lane_cells_num);

        // First cell in the lane
        lane_cells.push(
            Cell::new(cells_counter)
                .with_point(new_point(0.0, lanes_counter as f64, None))
                .with_zone_type(ZoneType::Birth)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(cells_counter + 1)
                .with_right_node(-1)
                .with_meso_link(lanes_counter)
                .build(),
        );
        cells_counter += 1;

        // Middle cells in the lane
        for i in 1..single_lane_cells_num - 1 {
            lane_cells.push(
                Cell::new(cells_counter)
                    .with_point(new_point(i as f64, lanes_counter as f64, None))
                    .with_zone_type(ZoneType::Common)
                    .with_speed_limit(3)
                    .with_left_node(-1)
                    .with_forward_node(cells_counter + 1)
                    .with_right_node(-1)
                    .with_meso_link(lanes_counter)
                    .build(),
            );
            cells_counter += 1;
        }

        // Last cell in the lane
        lane_cells.push(
            Cell::new(cells_counter)
                .with_point(new_point(
                    (single_lane_cells_num - 1) as f64,
                    lanes_counter as f64,
                    None
                ))
                .with_zone_type(ZoneType::Death)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(-1)
                .with_right_node(-1)
                .with_meso_link(lanes_counter)
                .build(),
        );
        cells_counter += 1;
        lanes_counter += 1;

        all_lanes_cells.extend_from_slice(&lane_cells);

        if last_added_lane.is_empty() {
            last_added_lane = lane_cells;
            continue;
        }

        // Connect lanes with left/right nodes
        for i in 0..single_lane_cells_num - 1 {
            if let Some(last_cell) = all_lanes_cells.get_mut(i + (j - 1) * single_lane_cells_num) {
                last_cell.set_left_id(lane_cells[i + 1].get_id());
            }
            if lanes_counter > 1 {
                if let Some(last_cell) = all_lanes_cells.get_mut(i + j * single_lane_cells_num) {
                    last_cell.set_right_id(last_added_lane[i + 1].get_id());
                }
            }
        }
        last_added_lane = lane_cells;
    }

    all_lanes_cells
}
