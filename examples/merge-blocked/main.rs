//! Demonstration of the "keep rolling along a jammed lane" fallback in
//! find_alternate_intention, on the real "Pisareva" stage network.
//!
//! Setup: vehicle 1 drives in lane 666 -> 667 -> 668 (no right neighbor) with
//! destination 621, which is reachable ONLY through the left merges
//! 666->605, 667->606, 668->619 (after 668 comes the point of no return:
//! 681 -> 683 dead end). The left lane is jammed with parked vehicles.
//!
//! Scenario A - gap at the last merge (605, 606 parked, 619 free):
//! the vehicle rolls along the jam and merges into the gap -> COMPLETED.
//!
//! Scenario B - fully jammed (605, 606, 619 parked):
//! the vehicle rolls past the last merge, per-tick A* returns NoPathFound,
//! confusion drives it forward into the 683 death zone -> LOST.
//! The trip fails, but the road stays free - an accepted risk.
//!
//! A DEADLOCK verdict (standing next to the jam forever) is a regression:
//! it means the forward fallback in find_alternate_intention stopped working.
//!
//! Run: cargo run --example merge-blocked

use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::geom::{SRID, new_point};
use micro_traffic_sim_core::grid::{cell::Cell, road_network::GridRoads, zones::ZoneType};
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::verbose::VerboseLevel;

const NETWORK_JSON: &str = include_str!("network.json");

const START_CELL: i64 = 666;
const DESTINATION: i64 = 621;
const STEPS: usize = 80;
const DEADLOCK_THRESHOLD: usize = 15;

fn load_grid() -> GridRoads {
    let cells: serde_json::Value =
        serde_json::from_str(NETWORK_JSON).expect("network.json must be valid JSON");
    let mut grid = GridRoads::new();
    for c in cells.as_array().expect("array of cells") {
        let zone = match c["zone"].as_i64().unwrap() {
            1 => ZoneType::Birth,
            2 => ZoneType::Death,
            3 => ZoneType::Coordination,
            4 => ZoneType::Common,
            _ => ZoneType::Undefined,
        };
        let cell = Cell::new(c["id"].as_i64().unwrap())
            .with_point(new_point(
                c["lon"].as_f64().unwrap(),
                c["lat"].as_f64().unwrap(),
                Some(SRID::WGS84),
            ))
            .with_forward_node(c["fwd"].as_i64().unwrap())
            .with_left_node(c["left"].as_i64().unwrap())
            .with_right_node(c["right"].as_i64().unwrap())
            .with_zone_type(zone)
            .with_speed_limit(c["sl"].as_i64().unwrap() as i32)
            .build();
        grid.add_cell(cell);
    }
    grid
}

fn run_scenario(name: &str, parked_cells: &[i64]) {
    println!("Scenario {name}: parked at {parked_cells:?}");

    let traveller = Vehicle::new(1)
        .with_speed(1)
        .with_speed_limit(4)
        .with_cell(START_CELL)
        .with_destination(DESTINATION)
        .with_slowdown(0.0)
        .build();
    let mut vehicles = vec![traveller];
    for (i, &cell) in parked_cells.iter().enumerate() {
        vehicles.push(
            Vehicle::new(100 + i as u64)
                .with_cell(cell)
                .with_destination(-1)
                .with_behaviour(BehaviourType::Block)
                .build(),
        );
    }

    let grids_storage = GridsStorage::new().with_vehicles_net(load_grid()).build();
    let mut session = Session::new(grids_storage, None);
    session.set_verbose_level(VerboseLevel::None);
    session.add_vehicles(vehicles);

    let mut trajectory: Vec<i64> = vec![START_CELL];
    let mut stuck_ticks = 0usize;
    let mut completed = 0;
    let mut lost = 0;

    for _ in 0..STEPS {
        let state = session.step().expect("simulation step failed");
        completed = state.vehicles_completed;
        lost = state.vehicles_lost;

        let traveller = session
            .get_vehicles()
            .into_iter()
            .find(|(_, v)| v.id == 1)
            .map(|(_, v)| v);
        match traveller {
            Some(v) => {
                if *trajectory.last().unwrap() == v.cell_id {
                    if v.speed == 0 {
                        stuck_ticks += 1;
                    }
                } else {
                    trajectory.push(v.cell_id);
                    stuck_ticks = 0;
                }
            }
            None => break, // removed: either completed or lost
        }
        if stuck_ticks >= DEADLOCK_THRESHOLD {
            break;
        }
    }

    println!("trajectory: {trajectory:?}");
    if completed > 0 {
        println!("VERDICT: COMPLETED - merged into the gap and reached {DESTINATION}\n");
    } else if lost > 0 {
        println!(
            "VERDICT: LOST - rolled past the last merge, wandered to the death zone; \
             the lane stays free (accepted risk)\n"
        );
    } else if stuck_ticks >= DEADLOCK_THRESHOLD {
        println!(
            "VERDICT: DEADLOCK - stood {stuck_ticks} ticks at cell {} waiting for \
             the jammed lane; REGRESSION - the forward fallback did not kick in\n",
            trajectory.last().unwrap()
        );
    } else {
        println!("VERDICT: INCONCLUSIVE after {STEPS} steps\n");
    }
}

fn main() {
    let grid = load_grid();
    println!(
        "Loaded stage network: {} cells; route to {DESTINATION} exists only via left merges 605/606/619\n",
        grid.get_cells_num()
    );
    run_scenario("A (gap at the last merge)", &[605, 606]);
    run_scenario("B (fully jammed)", &[605, 606, 619]);
}
