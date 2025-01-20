use crate::grid::{
    cell::{Cell, CellID},
    lane_change_type::LaneChangeType,
    road_network::GridRoads,
};
use crate::shortest_path::{heuristics::heuristic, path::Path};
use std::{
    cell::RefCell,
    cmp::Ordering,
    collections::BinaryHeap,
    fmt,
    rc::Rc,
};
use indexmap::IndexMap;
// Define custom error types
#[derive(Debug)]
pub enum AStarError {
    BadData,
    NoPathFound { start_id: CellID, end_id: CellID },
}

impl fmt::Display for AStarError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AStarError::BadData => write!(f, "Bad data"),
            AStarError::NoPathFound { start_id, end_id } => {
                write!(
                    f,
                    "No shortest path has been found bentween IDs {} and {}",
                    start_id, end_id
                )
            }
        }
    }
}

// Struct for AStarNode
#[derive(Debug, Clone)]
pub struct AStarNode<'a> {
    pub cell: &'a Cell,
    pub g_cost: f64, // route cost
    pub f_cost: f64, // g + h
    // pub parent: Option<&'a AStarNode<'a>>,
    pub parent: Option<Rc<RefCell<AStarNode<'a>>>>,
    pub maneuver: LaneChangeType,
}

impl<'a> AStarNode<'a> {
    pub fn new(
        cell: &'a Cell,
        g_cost: f64,
        f_cost: f64,
        // parent: Option<&'a AStarNode<'a>>,
        parent: Option<Rc<RefCell<AStarNode<'a>>>>,
        maneuver: LaneChangeType,
    ) -> Self {
        AStarNode {
            cell,
            g_cost,
            f_cost,
            parent,
            maneuver,
        }
    }
}

// For implementing min-heap

impl<'a> PartialEq for AStarNode<'a> {
    fn eq(&self, other: &AStarNode) -> bool {
        self.f_cost == other.f_cost
    }
}

impl<'a> Eq for AStarNode<'a> {}

impl<'a> PartialOrd for AStarNode<'a> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<'a> Ord for AStarNode<'a> {
    fn cmp(&self, other: &AStarNode) -> Ordering {
        let f_cmp = other
            .f_cost
            .partial_cmp(&self.f_cost)
            .unwrap_or(Ordering::Equal);
        if f_cmp == Ordering::Equal {
            if f_cmp == Ordering::Equal {
                return other.cell.get_id().cmp(&self.cell.get_id()); // Break tie by ID (or another field)
            }
        }
        f_cmp
    }
}

// Shortest Path function (A* algorithm implementation)
pub fn shortest_path<'a>(
    start: &'a Cell,
    goal: &'a Cell,
    net: &'a GridRoads,
    maneuver_allowed: bool,
    max_depth_opt: Option<usize>,
) -> Result<Path<'a>, AStarError> {
    let max_depth = max_depth_opt.unwrap_or(0);
    let mut open_set = BinaryHeap::new();

    let start_node = Rc::new(RefCell::new(AStarNode::new(
        start,
        0.0,
        heuristic(start, goal),
        None,
        LaneChangeType::NoChange,
    )));

    open_set.push(start_node);

    let mut came_from = IndexMap::new();
    let mut g_score = IndexMap::new();
    g_score.insert(start.get_id(), 0.0);

    let mut research_vertices = 0;

    while let Some(current_node) = open_set.pop() {
        // println!("pop with id: {} {} {}", current_node.borrow().cell.get_id(), current_node.borrow().f_cost, current_node.borrow().g_cost);
        research_vertices += 1;

        if max_depth > 0 && research_vertices >= max_depth {
            return Ok(reconstruct_path(&current_node));
        }

        // If we've reached the goal
        let current_cell = current_node.borrow().cell;
        if current_cell.get_id() == goal.get_id() {
            return Ok(reconstruct_path(&current_node));
        }

        // Scan straightforward direction
        if let Some(fcell) = net.get_cell(&current_cell.get_forward_id()) {
            process_neighbor(
                goal,
                current_node.clone(),
                fcell,
                LaneChangeType::NoChange,
                &mut g_score,
                &mut open_set,
                &mut came_from,
            );
        }

        if !maneuver_allowed {
            continue;
        }

        // Scan left maneuver
        if let Some(lcell) = net.get_cell(&current_cell.get_left_id()) {
            process_neighbor(
                goal,
                current_node.clone(),
                lcell,
                LaneChangeType::ChangeLeft,
                &mut g_score,
                &mut open_set,
                &mut came_from,
            );
        }

        // Scan right maneuver
        if let Some(rcell) = net.get_cell(&current_cell.get_right_id()) {
            process_neighbor(
                goal,
                current_node.clone(),
                rcell,
                LaneChangeType::ChangeRight,
                &mut g_score,
                &mut open_set,
                &mut came_from,
            );
        }
    }

    Err(AStarError::NoPathFound {
        start_id: start.get_id(),
        end_id: goal.get_id(),
    })
}

fn process_neighbor<'a>(
    goal: &Cell,
    // current_node: AStarNode<'a>,
    current_node: Rc<RefCell<AStarNode<'a>>>,
    neighbor_cell: &'a Cell,
    neighbor_maneuver: LaneChangeType,
    g_score: &mut IndexMap<i64, f64>, // CellID type
    // open_set: &mut BinaryHeap<AStarNode<'a>>,
    // came_from: &mut HashMap<i64, AStarNode<'a>>,
    open_set: &mut BinaryHeap<Rc<RefCell<AStarNode<'a>>>>,
    came_from: &mut IndexMap<i64, Rc<RefCell<AStarNode<'a>>>>,
) {
    let tentative_g_score =
        current_node.borrow().g_cost + heuristic(current_node.borrow().cell, neighbor_cell);
    let neighbor_cell_id = neighbor_cell.get_id();
    if tentative_g_score < *g_score.get(&neighbor_cell_id).unwrap_or(&f64::INFINITY) {
        // println!("scan {} {}", neighbor_cell_id, tentative_g_score);
        g_score.insert(neighbor_cell_id, tentative_g_score);

        let neighbor = Rc::new(RefCell::new(AStarNode::new(
            neighbor_cell,
            tentative_g_score,
            tentative_g_score + heuristic(neighbor_cell, goal),
            // None,
            Some(current_node.clone()),
            neighbor_maneuver,
        )));

        // println!(
        //     "\tpush {} {} {}",
        //     neighbor.borrow().cell.get_id(),
        //     neighbor.borrow().f_cost,
        //     neighbor.borrow().g_cost
        // );
        open_set.push(neighbor.to_owned());
        came_from.insert(neighbor_cell_id, current_node);
    }
}

/// Reconstructs the path from the goal to the start by following the `Parent` pointers.
fn reconstruct_path<'a>(current_node: &Rc<RefCell<AStarNode<'a>>>) -> Path<'a> {
    let mut vertices = Vec::new();
    let mut maneuvers = Vec::new();
    let mut current = Some(current_node.clone());
    let cost = current_node.borrow().g_cost;
    while let Some(node) = current {
        vertices.push(node.borrow().cell);
        if let Some(parent) = &node.borrow().parent {
            maneuvers.push(node.borrow().maneuver);
        }
        current = node.borrow().parent.clone();
    }

    // Reverse the vectors to put the path in the correct order
    vertices.reverse();
    maneuvers.reverse();

    Path::new(vertices, maneuvers, cost)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Point;
    use crate::grid::zones::ZoneType;
    use crate::utils::generators::generate_one_lane_cells;

    #[test]
    fn test_reconstruct_path() {
        let cell1 = Cell::new(1).build();
        let cell2 = Cell::new(33).build();
        let cell3 = Cell::new(22).build();

        let current_node1 = Rc::new(RefCell::new(AStarNode::new(
            &cell1,
            12.98,
            30.78,
            None,
            LaneChangeType::Undefined,
        )));

        let current_node2 = Rc::new(RefCell::new(AStarNode::new(
            &cell2,
            30.78,
            42.15,
            Some(current_node1.clone()),
            LaneChangeType::NoChange,
        )));

        let current_node3 = Rc::new(RefCell::new(AStarNode::new(
            &cell3,
            42.15,
            42.15,
            Some(current_node2.clone()),
            LaneChangeType::ChangeLeft,
        )));

        // Reconstruct the path
        let path = reconstruct_path(&current_node3);

        // Correct expected values
        let correct_vertices = vec![1, 33, 22];
        let correct_maneuvers = vec![LaneChangeType::NoChange, LaneChangeType::ChangeLeft];

        // Assertions
        assert_eq!(
            correct_vertices.len(),
            path.vertices().len(),
            "Incorrect path.vertices length"
        );
        assert_eq!(
            correct_maneuvers.len(),
            path.maneuvers().len(),
            "Incorrect path.maneuvers length"
        );

        for (i, vertex) in path.vertices().iter().enumerate() {
            assert_eq!(
                correct_vertices[i],
                vertex.get_id(),
                "Incorrect vertex at position {}",
                i
            );
        }

        for (i, maneuver) in path.maneuvers().iter().enumerate() {
            assert_eq!(
                correct_maneuvers[i], *maneuver,
                "Incorrect maneuver at position {}",
                i
            );
        }

        assert_eq!(
            current_node3.borrow().g_cost,
            path.cost(),
            "Incorrect path cost"
        );
    }
    #[test]
    fn test_big_router() {
        let cells_data = generate_one_lane_cells(5000.0, 4.5, 2);
        let mut grid = GridRoads::new();
        for cell in &cells_data {
            grid.add_cell(cell.clone());
        }
        let start_cell = grid.get_cell(&1).unwrap();
        let target_cell = grid.get_cell(&2224).unwrap();
        let path_result = shortest_path(&start_cell, &target_cell, &grid, true, None);
        assert!(
            path_result.is_ok(),
            "Pathfinding failed: {:?}",
            path_result.err()
        );
        let path = path_result.unwrap();
        let correct_cost = 1111.414213562373;
        assert!(
            (path.cost() - correct_cost).abs() < 0.001,
            "Cost should be {}, but got {}",
            correct_cost,
            path.cost()
        );
    }
    #[test]
    fn test_shortest_path() {
        let mut grid = GridRoads::new();
        grid.add_cell(
            Cell::new(1)
                .with_point(Point::new(0.0, 1.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(2)
                .with_right_node(8)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(2)
                .with_point(Point::new(1.0, 1.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(3)
                .with_right_node(9)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(3)
                .with_point(Point::new(2.0, 1.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(4)
                .with_right_node(10)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(4)
                .with_point(Point::new(3.0, 1.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(5)
                .with_right_node(11)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(5)
                .with_point(Point::new(4.0, 1.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(6)
                .with_right_node(12)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(6)
                .with_point(Point::new(5.0, 1.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(13)
                .with_right_node(17)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(7)
                .with_point(Point::new(0.0, 0.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(2)
                .with_forward_node(8)
                .with_right_node(-1)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(8)
                .with_point(Point::new(1.0, 0.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(3)
                .with_forward_node(9)
                .with_right_node(-1)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(9)
                .with_point(Point::new(2.0, 0.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(4)
                .with_forward_node(10)
                .with_right_node(-1)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(10)
                .with_point(Point::new(3.0, 0.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(5)
                .with_forward_node(11)
                .with_right_node(-1)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(11)
                .with_point(Point::new(4.0, 0.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(6)
                .with_forward_node(12)
                .with_right_node(-1)
                .with_meso_link(999)
                .build(),
        );
        grid.add_cell(
            Cell::new(12)
                .with_point(Point::new(5.0, 0.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(13)
                .with_forward_node(17)
                .with_right_node(-1)
                .with_meso_link(999)
                .build(),
        );

        grid.add_cell(
            Cell::new(17)
                .with_point(Point::new(12.0, -1.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(14)
                .with_forward_node(18)
                .with_right_node(-1)
                .with_meso_link(100500)
                .build(),
        );
        grid.add_cell(
            Cell::new(18)
                .with_point(Point::new(12.0, -2.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(15)
                .with_forward_node(19)
                .with_right_node(-1)
                .with_meso_link(100500)
                .build(),
        );
        grid.add_cell(
            Cell::new(19)
                .with_point(Point::new(12.0, -3.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(16)
                .with_forward_node(20)
                .with_right_node(-1)
                .with_meso_link(100500)
                .build(),
        );
        grid.add_cell(
            Cell::new(20)
                .with_point(Point::new(12.0, -4.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(-1)
                .with_right_node(-1)
                .with_meso_link(100500)
                .build(),
        );

        grid.add_cell(
            Cell::new(13)
                .with_point(Point::new(13.0, -1.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(14)
                .with_right_node(18)
                .with_meso_link(100500)
                .build(),
        );
        grid.add_cell(
            Cell::new(14)
                .with_point(Point::new(13.0, -2.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(15)
                .with_right_node(19)
                .with_meso_link(100500)
                .build(),
        );
        grid.add_cell(
            Cell::new(15)
                .with_point(Point::new(13.0, -3.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(16)
                .with_right_node(20)
                .with_meso_link(100500)
                .build(),
        );
        grid.add_cell(
            Cell::new(16)
                .with_point(Point::new(13.0, -4.0))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(1)
                .with_left_node(-1)
                .with_forward_node(-1)
                .with_right_node(-1)
                .with_meso_link(100500)
                .build(),
        );

        // Run the shortest path algorithm for a specific source and target
        let start_cell = grid.get_cell(&7).unwrap();
        let target_cell = grid.get_cell(&15).unwrap();
        let path_result = shortest_path(&start_cell, &target_cell, &grid, true, None);
        assert!(
            path_result.is_ok(),
            "Pathfinding failed: {:?}",
            path_result.err()
        );
        let path = path_result.unwrap();

        let correct_cost = 14.485281374238571;
        let correct_vertices = vec![7, 8, 9, 10, 11, 12, 17, 14, 15];
        let correct_maneuvers = vec![
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::ChangeLeft,
            LaneChangeType::NoChange,
        ];

        assert!(
            (path.cost() - correct_cost).abs() < 0.001,
            "Cost should be {}, but got {}",
            correct_cost,
            path.cost()
        );

        assert_eq!(
            correct_vertices.len(),
            path.vertices().len(),
            "Incorrect path.vertices length"
        );

        assert_eq!(
            correct_maneuvers.len(),
            path.maneuvers().len(),
            "Incorrect path.maneuvers length"
        );

        for (i, vertex) in path.vertices().iter().enumerate() {
            assert_eq!(
                correct_vertices[i],
                vertex.get_id(),
                "Incorrect vertex at position {}",
                i
            );
        }

        for (i, maneuver) in path.maneuvers().iter().enumerate() {
            assert_eq!(
                correct_maneuvers[i], *maneuver,
                "Incorrect maneuver at position {}",
                i
            );
        }
    }
}
