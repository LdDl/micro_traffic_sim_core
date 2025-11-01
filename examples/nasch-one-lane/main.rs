use micro_traffic_sim_core::geom::{Point, new_point};
use micro_traffic_sim_core::grid::{cell::Cell, road_network::GridRoads};
use micro_traffic_sim_core::agents::{Vehicle, VehicleRef};
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::verbose::init_logger;
use micro_traffic_sim_core::verbose::VerboseLevel;
use std::rc::Rc;
use std::cell::RefCell;

fn main() {
    init_logger();

    // 1. Create a one-lane road with N cells
    let n = 20;
    let mut grid = GridRoads::new();
    let lane_length = n;
    let last_cell_id = (lane_length - 1) as i64;
    println!("cell_id;x;y");
    for i in 0..lane_length {
        let mut cell = Cell::new(i as i64);
        if i < lane_length - 1 {
            cell = cell.with_forward_node((i + 1) as i64);
        }
        cell = cell
            .with_point(new_point(i as f64, 1.0, None))
            .with_speed_limit(4);
        let c = cell.build();
        let pt = c.get_point();
        println!("{};{:.5};{:.5}", c.get_id(), pt.x(), pt.y());
        grid.add_cell(c);
    }
    // 2. Create a vehicle at position 0, speed = 1, speed_limit = 1
    let vehicle = Vehicle::new(0)
        .with_speed(1)
        .with_speed_limit(1)
        .with_cell(0)
        .with_destination(last_cell_id)
        .build();
    let vehicles: Vec<VehicleRef> = vec![Rc::new(RefCell::new(vehicle))];

    // 3. Prepare simulation session
    let grids_storage = GridsStorage::new()
        .with_vehicles_net(grid)
        .build();
    let mut session = Session::new(grids_storage, None);
    session.set_verbose_level(VerboseLevel::None);
    session.add_vehicles(vehicles);

    // 4. Simulate steps
    let steps = n - 1;
    println!("step;vehicle_id;vehicle_type;last_speed;last_angle;intermediate_cells;last_cell;x;y");
    // Print initial state
    for (_vid, veh) in session.get_vehicles() {
        let v = veh.borrow();
        let (x, y) = if let Some(cell) = session.get_cell(&v.cell_id) {
            let pt = cell.get_point();
            (pt.x(), pt.y())
        } else {
            (f64::NAN, f64::NAN)
        };
        println!(
            "-1;{};{};{};{:.5};{};{};{:.5};{:.5}",
            v.id,
            v.vehicle_type,
            v.speed,
            0.0,
            "",
            v.cell_id,
            x,
            y
        );
    }
    for step in 0..steps {
        match session.step() {
            Ok(updated_vehicles) => {
                for v in updated_vehicles.vehicles {
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
                    println!(
                        "{};{};{};{};{:.5};{};{};{:.5};{:.5}",
                        step,
                        v.id,
                        v.vehicle_type,
                        v.last_speed,
                        v.last_angle,
                        intermediate_cells,
                        v.last_cell,
                        x,
                        y
                    );
                }
            }
            Err(e) => {
                eprintln!("Error during simulation step {}: {}", step, e);
                break;
            }
        }
    }
}