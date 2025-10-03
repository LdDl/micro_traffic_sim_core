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
    let N = 20;
    let mut grid = GridRoads::new();
    let lane_length = N;
    let last_cell_id = (lane_length - 1) as i64;
    for i in 0..lane_length {
        let mut cell = Cell::new(i as i64);
        if i < lane_length - 1 {
            cell = cell
                .with_forward_node((i + 1) as i64)
                .with_speed_limit(4);
        }
        grid.add_cell(cell.build());
    }
    // 2. Create a vehicle at position 0, speed = 1, speed_limit = 1
    let vehicle = Vehicle::new(0)
        .with_speed(2)
        .with_speed_limit(2)
        .with_cell(0)
        .with_destination(last_cell_id)
        .build();
    let vehicles: Vec<VehicleRef> = vec![Rc::new(RefCell::new(vehicle))];

    // 3. Prepare simulation session
    let grids_storage = GridsStorage::new()
        .with_vehicles_net(grid)
        .build();
    let mut session = Session::new(grids_storage, None);
    session.set_verbose_level(VerboseLevel::Main);
    session.add_vehicles(vehicles);

    // 4. Simulate steps
    let steps = N - 2;
    for step in 0..steps {
        match session.step() {
            Ok(updated_vehicles) => {
                println!("Step {}", step);
                for v in updated_vehicles.vehicles {
                    println!(
                        " -> Vehicle ID {} is at cell {}, speed {}",
                        v.id, v.last_cell, v.last_speed
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