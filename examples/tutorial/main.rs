use micro_traffic_sim_core::geom::{Point, new_point};
use micro_traffic_sim_core::grid::{cell::Cell, road_network::GridRoads, zones::ZoneType};
use micro_traffic_sim_core::conflict_zones::{ConflictWinnerType, ConflictEdge, ConflictZone};
use micro_traffic_sim_core::traffic_lights::lights::TrafficLight;
use micro_traffic_sim_core::traffic_lights::groups::TrafficLightGroup;
use micro_traffic_sim_core::traffic_lights::signals::SignalType;
use micro_traffic_sim_core::agents::{Vehicle, VehicleRef};
use micro_traffic_sim_core::agents_types::AgentType;
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::trips::trip::{Trip, TripType};
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::verbose::init_logger;
use micro_traffic_sim_core::verbose::VerboseLevel;
use std::rc::Rc;
use std::cell::RefCell;
use std::collections::HashMap;

fn main() {
    init_logger();
    // ==============================================================
    // STEP 1: CREATE GRID WITH HORIZONTAL AND VERTICAL ROADS
    // ==============================================================
    // Road layout:
    //
    //        V1 (vertical 1)    V2 (vertical 2)
    //          |                 |
    //    H ----+-----------------+---- H (horizontal)
    //          |                 |
    //          
    // Horizontal road cells: 0-9 (y=3.5, x=0..9)
    // Vertical road 1 cells: 10-19 (y=0..4, x=3.0)
    // Vertical road 2 cells: 20-29 (y=0..4, x=7.0)
    //
    // Intersections at:
    // - (3, 3.0) for H and V1
    // - (7, 3.0) for H and V2

    let mut grid = GridRoads::new();
    // ========== HORIZONTAL ROAD (cells 0-9) ==========
    for i in 0..10 {
        let mut cell = Cell::new(i as i64);
        if i < 9 {
            cell = cell.with_forward_node((i + 1) as i64);
        }
        if i == 3 {
            // Left maneuver to V1
            cell = cell.with_left_node(14);
        }
        if i == 6 {
            // Left maneuver to V2
            cell = cell.with_left_node(24);
        }
        let pt = new_point(i as f64, 3.5, None);
        cell = cell
            .with_point(pt)
            .with_zone_type(if i == 0 {
                ZoneType::Birth
            } else if i == 9 {
                ZoneType::Death
            } else {
                ZoneType::Common
            })
            .with_speed_limit(1);
        let c = cell.build();
        grid.add_cell(c);
    }

    // ========== VERTICAL ROAD 1 (cells 10-19, x=3.5) ==========
    for i in 0..10 {
        let cell_id = (10 + i) as i64;
        let mut cell = Cell::new(cell_id);
        if i < 9 {
            cell = cell.with_forward_node((cell_id + 1) as i64);
        }
        if i == 3 {
            // Right maneuver to H
            cell = cell.with_right_node(4);
        }
        cell = cell
            .with_point(new_point(3.5, i as f64, None))
            .with_zone_type(if i == 0 {
                ZoneType::Birth
            } else if i == 9 {
                ZoneType::Death
            } else {
                ZoneType::Common
            })
            .with_speed_limit(1);
        let c = cell.build();
        grid.add_cell(c);
    }

    // ========== VERTICAL ROAD 2 (cells 20-29, x=6.5) ==========
    for i in 0..10 {
        let cell_id = (20 + i) as i64;
        let mut cell = Cell::new(cell_id);
        if i < 9 {
            cell = cell.with_forward_node((cell_id + 1) as i64);
        }
        if i == 3 {
            // Right maneuver to H
            cell = cell.with_right_node(7);
        }
        cell = cell
            .with_point(new_point(6.5, i as f64, None))
            .with_zone_type(if i == 0 {
                ZoneType::Birth
            } else if i == 9 {
                ZoneType::Death
            } else {
                ZoneType::Common
            })
            .with_speed_limit(1);
        let c = cell.build();
        grid.add_cell(c);
    }

    // ==============================================================
    // STEP 2: ADD CONFLICT ZONES [OPTIONAL]
    // ==============================================================
    let mut conflict_zones = HashMap::new();
    let conflict_zone = ConflictZone::new(
            1,
            ConflictEdge {
                source: 3,
                target: 4,
            },
            ConflictEdge {
                source: 13,
                target: 14,
            },
        )
        // V1 has priority over H
        .with_winner_type(ConflictWinnerType::Second)
        .build();
    conflict_zones.insert(conflict_zone.get_id(), conflict_zone);

    // ==============================================================
    // STEP 3: ADD TRAFFIC LIGHTS [OPTIONAL]
    // ==============================================================
    let mut tls = HashMap::new();
    let group_h = TrafficLightGroup::new(100)
        .with_cells_ids(vec![6])
        .with_label("Group block H".to_string())
        .with_signal(vec![SignalType::Green, SignalType::Red])
        .build();
    let group_v2 = TrafficLightGroup::new(200)
        .with_cells_ids(vec![23])
        .with_label("Group block V2".to_string())
        .with_signal(vec![SignalType::Red, SignalType::Green])
        .build();
    let tl = TrafficLight::new(1)
        .with_coordinates(new_point(7.0, 4.0, None))
        // 5s green, 5s red
        .with_phases_times(vec![5, 5])
        .with_groups(vec![group_h, group_v2])
        .build();
    tls.insert(tl.get_id(), tl);
    
    // ==============================================================
    // STEP 4: Add vehicles statically (via initial positions)
    // ==============================================================
    let vehicle = Vehicle::new(0)
        .with_speed(1)
        .with_speed_limit(1)
        .with_cell(4)
        .with_destination(9)
        .build();
    let vehicles: Vec<VehicleRef> = vec![Rc::new(RefCell::new(vehicle))];

    // ==============================================================
    // STEP 5: Add vehicles dynamically via trips
    // ==============================================================
    let trips_h = Trip::new(1, 9, TripType::Random)
        .with_allowed_agent_type(AgentType::Car)
        .with_allowed_behaviour_type(BehaviourType::Cooperative)
        .with_probability(0.1)
        .build();
    let trips_v1 = Trip::new(10, 19, TripType::Random)
        .with_allowed_agent_type(AgentType::Car)
        .with_allowed_behaviour_type(BehaviourType::Cooperative)
        .with_probability(0.1)
        .build();
    let trips_v2 = Trip::new(20, 29, TripType::Random)
        .with_allowed_agent_type(AgentType::Car)
        .with_allowed_behaviour_type(BehaviourType::Cooperative)
        .with_probability(0.1)
        .build();
    let trips: Vec<Trip> = vec![trips_h, trips_v1, trips_v2];

    // ==============================================================
    // STEP 6: Setup simulation
    // ==============================================================
    print_grid_tls(&grid, &tls); // Print grid and TLS before borrow
    let grids_storage = GridsStorage::new()
        .with_vehicles_net(grid)
        .with_tls(tls)
        .build();
    let mut session = Session::new(grids_storage, None);
    session.set_verbose_level(VerboseLevel::Main);
    // session.add_vehicles(vehicles);
    for trip in trips.iter() {
        session.add_trip(trip.clone());
    }
    
    // ==============================================================
    // STEP 7: Run simulation
    // and STEP 8: Collect data
    // ==============================================================
    let steps_num = 50;
    println!("step;vehicle_id;vehicle_type;last_speed;last_angle;intermediate_cells;last_cell;x;y");
    // Print initial state
    for (vid, veh) in session.get_vehicles() {
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
    let mut tls_states = vec![];
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
                for tl in automata_state.tls {
                    // let tls_ref = session.get_tls_ref();
                    // let tl_ref = tls_ref.get(&tl.0).unwrap();
                    for group in tl.1 {
                        let group_id = group.group_id;
                        let signal = group.last_signal;
                        tls_states.push((step, tl.0, group_id, signal));
                    }
                }
            }
            Err(e) => {
                eprintln!("Error during simulation step {}: {}", step, e);
                break;
            }
        }
    }
    println!("tl_step;tl_id;group_id;cell_id;x;y;signal");
    let tls_ref = session.get_tls_ref();
    for (step, tl_id, group_id, signal) in tls_states {
        let tl_ref = tls_ref.get(&tl_id).unwrap();
        // Find group
        for groups in tl_ref.get_groups() {
            if groups.get_id() != group_id {
               continue;
            }
            let cells_ids = groups.get_cells_ids();
            for &cell_id in cells_ids {
                let cell = session.get_cell(&cell_id).unwrap();
                let pt = cell.get_point();
                println!("{};{};{};{};{:.5};{:.5};{}", step, tl_id, group_id, cell_id, pt.x(), pt.y(), signal);
            }
        }
    }
}

pub fn print_grid_tls(grid: &GridRoads, tls: &HashMap<i64, TrafficLight>) {
    // ==============================================================
    // STEP 9: Print data for visualization
    // ==============================================================
    println!("tl_id;x;y");
    for (tl_id, tl) in tls.iter() {
        let pt = tl.get_coordinates();
        println!("{};{:.5};{:.5}", tl.get_id(), pt.x(), pt.y());
    }
    println!("tl_id;controlled_cell;x;y");
    for (tl_id, tl) in tls.iter() {
        let pt = tl.get_coordinates();
        for group in tl.get_groups() {
            let cells_ids = group.get_cells_ids();
            for &cell_id in cells_ids {
                if let Some(cell) = grid.get_cell(&cell_id) {
                    let cell_pt = cell.get_point();
                    println!("{};{};{:.5};{:.5}", tl.get_id(), cell_id, cell_pt.x(), cell_pt.y());
                }
            }
        }
    }
    println!("cell_id;x;y;forward_x;forward_y;connection_type;zone");
    // First, print all cells (without connections) - this ensures all cells are visible
    for (cell_id, cell) in grid.iter() {
        let pt = cell.get_point();
        println!("{};{:.5};{:.5};{:.5};{:.5};cell;{}", cell_id, pt.x(), pt.y(), pt.x(), pt.y(), cell.get_zone_type());
    }
    // Then, print connections (arrows) only if they exist
    for (cell_id, cell) in grid.iter() {
        let pt = cell.get_point();
        // Forward connection (only if it exists)
        let fwd_id = cell.get_forward_id();
        if fwd_id != -1 {
            if let Some(fwd_cell) = grid.get_cell(&fwd_id) {
                let fwd_pt = fwd_cell.get_point();
                println!("{};{:.5};{:.5};{:.5};{:.5};forward;{}", cell_id, pt.x(), pt.y(), fwd_pt.x(), fwd_pt.y(), fwd_cell.get_zone_type());
            }
        }
        // Left connection (only if it exists)
        let left_id = cell.get_left_id();
        if left_id != -1 {
            if let Some(left_cell) = grid.get_cell(&left_id) {
                let left_pt = left_cell.get_point();
                println!("{};{:.5};{:.5};{:.5};{:.5};left;{}", cell_id, pt.x(), pt.y(), left_pt.x(), left_pt.y(), left_cell.get_zone_type());
            }
        }
        // Right connection (only if it exists)
        let right_id = cell.get_right_id();
        if right_id != -1 {
            if let Some(right_cell) = grid.get_cell(&right_id) {
                let right_pt = right_cell.get_point();
                println!("{};{:.5};{:.5};{:.5};{:.5};right;{}", cell_id, pt.x(), pt.y(), right_pt.x(), right_pt.y(), right_cell.get_zone_type());
            }
        }
    }
}