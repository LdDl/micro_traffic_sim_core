use micro_traffic_sim_core::geom::{Point, new_point};
use micro_traffic_sim_core::grid::{cell::Cell, road_network::GridRoads, zones::ZoneType};
use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::verbose::init_logger;
use micro_traffic_sim_core::verbose::VerboseLevel;

fn main() {
    init_logger();

    let mut grid = GridRoads::new();
    let n = 8;
    for i in 0..n {
        let next = match i {
            x if x == n - 1 => -1,
            _ => i + 1,
        };
        let cell = Cell::new(i as i64)
            .with_point(new_point(i as f64, 1.0, None))
            .with_forward_node(next as i64)
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .build();
        grid.add_cell(cell);
    }

    // Add vehicles statically
    let vehicle1 = Vehicle::new(0)
        .with_speed(1)
        .with_speed_limit(1)
        .with_cell(0)
        .with_destination(-1) // No specific destination, just keep moving
        .with_slowdown(0.0) // Ensure no random slowdowns
        .build();
    let vehicles: Vec<Vehicle> = vec![vehicle1];

    // Setup simulation
    print_grid(&grid); // Print grid before borrow
    let grids_storage = GridsStorage::new()
        .with_vehicles_net(grid)
        .build();
    let mut session = Session::new(grids_storage, None);
    session.set_verbose_level(VerboseLevel::None);
    session.add_vehicles(vehicles);
    
    // Run simulation and collect data
    let steps_num = 15;
    let mut vehicles_states = vec![];
    // Initial state
    for (_vid, v) in session.get_vehicles() {
        let (x, y) = if let Some(cell) = session.get_cell(&v.cell_id) {
            let pt = cell.get_point();
            (pt.x(), pt.y())
        } else {
            (f64::NAN, f64::NAN)
        };
        vehicles_states.push((
            -1,
            v.id,
            v.vehicle_type,
            v.speed,
            0.0,
            "".to_string(),
            v.cell_id,
            x,
            y,
        ));
    }
    for step in 0..steps_num {
        match session.step() {
            Ok(automata_state) => {
                for v in automata_state.vehicles {
                    // Get coordinates from the grid by cell ID
                    let (x, y) = if let Some(cell) = session.get_cell(&v.last_cell) {
                        let pt = cell.get_point();
                        (pt.x(), pt.y())
                    } else {
                        (f64::NAN, f64::NAN)
                    };
                    // Join intermediate cells as comma-separated string
                    let intermediate_cells = v.last_intermediate_cells
                        .iter()
                        .map(|cell| cell.to_string())
                        .collect::<Vec<_>>()
                        .join(",");
                    vehicles_states.push((
                        step,
                        v.id,
                        v.vehicle_type,
                        v.last_speed,
                        v.last_angle,
                        intermediate_cells,
                        v.last_cell,
                        x,
                        y,
                    ));
                }
            }
            Err(e) => {
                eprintln!("Error during simulation step {}: {}", step, e);
                break;
            }
        }
    }
    println!("step;vehicle_id;vehicle_type;last_speed;last_angle;intermediate_cells;last_cell;x;y");
    for (step, vehicle_id, vehicle_type, last_speed, last_angle, intermediate_cells, last_cell, x, y) in vehicles_states {
        println!("{};{};{};{:.5};{:.5};{};{};{:.5};{:.5}", step, vehicle_id, vehicle_type, last_speed, last_angle, intermediate_cells, last_cell, x, y);
    }
}

pub fn print_grid(grid: &GridRoads) {
    // ==============================================================
    // Print data for visualization
    // ==============================================================
    println!("cell_id;x;y;forward_x;forward_y;connection_type");
    // First, print all cells (without connections) - this ensures all cells are visible
    for (cell_id, cell) in grid.iter() {
        let pt = cell.get_point();
        println!("{};{:.5};{:.5};{:.5};{:.5};cell", cell_id, pt.x(), pt.y(), pt.x(), pt.y());
    }
    // Then, print connections (arrows) only if they exist
    for (cell_id, cell) in grid.iter() {
        let pt = cell.get_point();
        // Forward connection (only if it exists)
        let fwd_id = cell.get_forward_id();
        if fwd_id != -1 {
            if let Some(fwd_cell) = grid.get_cell(&fwd_id) {
                let fwd_pt = fwd_cell.get_point();
                println!("{};{:.5};{:.5};{:.5};{:.5};forward", cell_id, pt.x(), pt.y(), fwd_pt.x(), fwd_pt.y());
            }
        }
        // Left connection (only if it exists)
        let left_id = cell.get_left_id();
        if left_id != -1 {
            if let Some(left_cell) = grid.get_cell(&left_id) {
                let left_pt = left_cell.get_point();
                println!("{};{:.5};{:.5};{:.5};{:.5};left", cell_id, pt.x(), pt.y(), left_pt.x(), left_pt.y());
            }
        }
        // Right connection (only if it exists)
        let right_id = cell.get_right_id();
        if right_id != -1 {
            if let Some(right_cell) = grid.get_cell(&right_id) {
                let right_pt = right_cell.get_point();
                println!("{};{:.5};{:.5};{:.5};{:.5};right", cell_id, pt.x(), pt.y(), right_pt.x(), right_pt.y());
            }
        }
    }
}