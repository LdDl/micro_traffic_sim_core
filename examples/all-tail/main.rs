use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::{cell::Cell, road_network::GridRoads};
use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::agents_types::AgentType;
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::trips::trip::{Trip, TripType};
use micro_traffic_sim_core::verbose::init_logger;
use micro_traffic_sim_core::verbose::VerboseLevel;
use std::collections::HashMap;
use std::fs::File;
use std::io::Write;

// Combined road layout with 9 isolated sub-networks:
//
// Sub-network 1 (L-shaped, cells 1-20): Vehicle 1 with tail=2 passes through
// [1]-[2]-[3]-[4]-[5]-[6]-[7]-[8]-[9]-[10]
//                                       \
//                                       [11]-[12]-[13]-[14]-[15]-[16]-[17]-[18]-[19]-[20]
//
// Sub-network 2 (Pingpong, cells 101-116): Vehicle 2 with tail=2 gets stuck at 111
// [101]-[102]-[103]-[104]-[105]-[106]-[107]-[108]-[109]-[110]    [112]-[113]-[114]-[115]-[116]
//                                                          \   /
//                                                          [111]
//
// Sub-network 3 (Extended zigzag, cells 201-216): Vehicle 3 with tail=1 passes through
// [201]-[202]-[203]-[204]-[205]-[206]-[207]-[208]-[209]-[210]          [213]-[214]-[215]-[216]
//                                                          \          /
//                                                          [211] - [212]
//
// Sub-network 4 (Extended zigzag, cells 301-316): Vehicle 4 with tail=2 gets stuck at 312
// [301]-[302]-[303]-[304]-[305]-[306]-[307]-[308]-[309]-[310]          [313]-[314]-[315]-[316]
//                                                          \          /
//                                                          [311] - [312]
//
// Sub-network 5 (Merge conflict, cells 401-415): V5 and V6 with tail=2 merge - CONFLICT!
// [401]-[402]-[403]-[404]-[405]
//                              \
//                              [406]-[407]-[408]-[409]-[410]
//                              /
// [411]-[412]-[413]-[414]-[415]
//
// Sub-network 6 (Crossing trajectories, cells 501-512): V7 and V8 cross paths - CONFLICT!
//       [502]
//         |
//         v
// [501]->[503]->[504]->[505]   V7: horizontal, does RIGHT at 503 to 509
//         ^
//         |
//       [506]               V8: vertical, does LEFT at 506 to 503
//         |
//        v
//       [507]-[508]-[509]-[510]-[511]-[512]
//
// Sub-network 7 (Following/rear-end, cells 601-620): V9 catches up to stopped V10
// [601]-...-[612]-[613]-[614]-[615]  [616]-...-[620]
//                             ^DEAD-END
// V10 starts at 612 (tail 610,611), reaches 615 and STOPS (destination 620 unreachable)
// V9 starts at 603 (tail 601,602) with speed=3, catches up and STOPS at 612 behind V10's tail
//
// Sub-network 8 (Triple merge, cells 701-718): V11, V12, V13 all merge at same point - CONFLICT!
// [701]-[702]-[703]-[704]
//                        \
// [705]-[706]-[707]-[708]-[709]-[710]-[711]-[712]
//                        /
// [713]-[714]-[715]-[716]
//
// Sub-network 9 (Straight line, cells 801-818): Trip generator spawns LargeBus (auto tail=2)
// [801]-[802]-[803]-...-[817]-[818]

fn main() {
    init_logger();

    let mut grid = GridRoads::new();
    let mut cell_positions: HashMap<i64, (f64, f64)> = HashMap::new();

    // === Sub-network 1: L-shaped road (cells 1-20) ===
    // Y base = 9
    for i in 1..=10 {
        let forward = if i < 10 { i + 1 } else { -1 };
        let right = if i == 10 { 11 } else { -1 };
        let x = i as f64;
        let y = 9.0;
        cell_positions.insert(i, (x, y));
        let cell = Cell::new(i)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }
    for i in 11..=20 {
        let forward = if i < 20 { i + 1 } else { -1 };
        let x = i as f64;
        let y = 8.0;
        cell_positions.insert(i, (x, y));
        let cell = Cell::new(i)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // === Sub-network 2: Pingpong road (cells 101-116) ===
    // Y base = 6
    for i in 1..=10 {
        let cell_id = 100 + i;
        let forward = if i < 10 { 100 + i + 1 } else { -1 };
        let right = if i == 10 { 111 } else { -1 };
        let x = i as f64;
        let y = 6.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }
    // Cell 111: transition cell with LEFT to 112
    cell_positions.insert(111, (11.0, 5.5));
    let cell = Cell::new(111)
        .with_point(new_point(11.0, 5.5, None))
        .with_speed_limit(3)
        .with_forward_node(-1)
        .with_left_node(112)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    for i in 12..=16 {
        let cell_id = 100 + i;
        let forward = if i < 16 { 100 + i + 1 } else { -1 };
        let x = i as f64;
        let y = 6.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // === Sub-network 3: Extended zigzag (cells 201-216) ===
    // Y base = 3
    for i in 1..=10 {
        let cell_id = 200 + i;
        let forward = if i < 10 { 200 + i + 1 } else { -1 };
        let right = if i == 10 { 211 } else { -1 };
        let x = i as f64;
        let y = 3.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }
    // Cell 211: transition cell with FORWARD to 212
    cell_positions.insert(211, (11.0, 2.5));
    let cell = Cell::new(211)
        .with_point(new_point(11.0, 2.5, None))
        .with_speed_limit(3)
        .with_forward_node(212)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    // Cell 212: transition cell with LEFT to 213
    cell_positions.insert(212, (12.0, 2.5));
    let cell = Cell::new(212)
        .with_point(new_point(12.0, 2.5, None))
        .with_speed_limit(3)
        .with_forward_node(-1)
        .with_left_node(213)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    for i in 13..=16 {
        let cell_id = 200 + i;
        let forward = if i < 16 { 200 + i + 1 } else { -1 };
        let x = i as f64;
        let y = 3.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // === Sub-network 4: Extended zigzag (cells 301-316) ===
    // Y base = 0
    for i in 1..=10 {
        let cell_id = 300 + i;
        let forward = if i < 10 { 300 + i + 1 } else { -1 };
        let right = if i == 10 { 311 } else { -1 };
        let x = i as f64;
        let y = 0.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }
    // Cell 311: transition cell with FORWARD to 312
    cell_positions.insert(311, (11.0, -0.5));
    let cell = Cell::new(311)
        .with_point(new_point(11.0, -0.5, None))
        .with_speed_limit(3)
        .with_forward_node(312)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    // Cell 312: transition cell with LEFT to 313
    cell_positions.insert(312, (12.0, -0.5));
    let cell = Cell::new(312)
        .with_point(new_point(12.0, -0.5, None))
        .with_speed_limit(3)
        .with_forward_node(-1)
        .with_left_node(313)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    for i in 13..=16 {
        let cell_id = 300 + i;
        let forward = if i < 16 { 300 + i + 1 } else { -1 };
        let x = i as f64;
        let y = 0.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // === Sub-network 5: Merge conflict (cells 401-415) ===
    // Y base for upper path = -3, merge = -3.5, lower path = -4
    // Upper path: 401-405
    for i in 1..=5 {
        let cell_id = 400 + i;
        let forward = if i < 5 { 400 + i + 1 } else { -1 };
        let right = if i == 5 { 406 } else { -1 };
        let x = i as f64;
        let y = -3.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }
    // Merge point and continuation: 406-410
    for i in 6..=10 {
        let cell_id = 400 + i;
        let forward = if i < 10 { 400 + i + 1 } else { -1 };
        let x = i as f64;
        let y = -3.5;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }
    // Lower path: 411-415
    for i in 1..=5 {
        let cell_id = 410 + i;
        let forward = if i < 5 { 410 + i + 1 } else { -1 };
        let left = if i == 5 { 406 } else { -1 };
        let x = i as f64;
        let y = -4.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(left)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // === Sub-network 6: Crossing trajectories (cells 501-514) ===
    // Cell 501: V7 start (west)
    cell_positions.insert(501, (1.0, -7.0));
    let cell = Cell::new(501)
        .with_point(new_point(1.0, -7.0, None))
        .with_speed_limit(3)
        .with_forward_node(503)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    // Cell 502: V8 start (north)
    cell_positions.insert(502, (3.0, -6.0));
    let cell = Cell::new(502)
        .with_point(new_point(3.0, -6.0, None))
        .with_speed_limit(3)
        .with_forward_node(504)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    // Cell 503: before intersection (V7's path)
    cell_positions.insert(503, (2.0, -7.0));
    let cell = Cell::new(503)
        .with_point(new_point(2.0, -7.0, None))
        .with_speed_limit(3)
        .with_forward_node(504)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    // Cell 504: INTERSECTION - both V7 and V8 cross here!
    cell_positions.insert(504, (3.0, -7.0));
    let cell = Cell::new(504)
        .with_point(new_point(3.0, -7.0, None))
        .with_speed_limit(3)
        .with_forward_node(507)
        .with_left_node(-1)
        .with_right_node(505)
        .build();
    grid.add_cell(cell);
    // Cell 505: V7 continues east
    cell_positions.insert(505, (4.0, -7.0));
    let cell = Cell::new(505)
        .with_point(new_point(4.0, -7.0, None))
        .with_speed_limit(3)
        .with_forward_node(506)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    // Cell 506: V7 end (east)
    cell_positions.insert(506, (5.0, -7.0));
    let cell = Cell::new(506)
        .with_point(new_point(5.0, -7.0, None))
        .with_speed_limit(3)
        .with_forward_node(-1)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    // Cell 507: V8 continues south after intersection
    cell_positions.insert(507, (3.0, -8.0));
    let cell = Cell::new(507)
        .with_point(new_point(3.0, -8.0, None))
        .with_speed_limit(3)
        .with_forward_node(509)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    // Cell 508: west end of bottom path
    cell_positions.insert(508, (2.0, -9.0));
    let cell = Cell::new(508)
        .with_point(new_point(2.0, -9.0, None))
        .with_speed_limit(3)
        .with_forward_node(509)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    // Cells 509-514: V8's final path going east
    // Compressed x spacing so all cells satisfy x+y < -4 for greedy pathfinder
    for i in 9..=14 {
        let cell_id = 500 + i;
        let forward = if i < 14 { 500 + i + 1 } else { -1 };
        let x = 3.0 + (i - 9) as f64 * 0.3;
        let y = -9.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // === Sub-network 7: Following/rear-end (cells 601-620) ===
    // Y base = -10
    // Extended straight road with dead-end at 615 - V10 stops there, V9 catches up
    for i in 1..=20 {
        let cell_id = 600 + i;
        let forward = if i < 15 {
            600 + i + 1
        } else if i == 15 {
            -1 // Cell 615 is a DEAD-END - V10 will stop here
        } else if i < 20 {
            600 + i + 1 // Cells 616-620 exist but are unreachable
        } else {
            -1
        };
        let x = i as f64;
        let y = -10.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(5)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // === Sub-network 8: Triple merge (cells 701-718) ===
    // Upper path: 701-704 (y=-12)
    for i in 1..=4 {
        let cell_id = 700 + i;
        let forward = if i < 4 { 700 + i + 1 } else { -1 };
        let right = if i == 4 { 709 } else { -1 };
        let x = i as f64;
        let y = -12.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }
    // Middle path: 705-712 (y=-13) - this is the main road
    for i in 5..=12 {
        let cell_id = 700 + i;
        let forward = if i < 12 { 700 + i + 1 } else { -1 };
        let x = (i - 4) as f64;
        let y = -13.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }
    // Lower path: 713-716 (y=-14)
    for i in 1..=4 {
        let cell_id = 712 + i;
        let forward = if i < 4 { 712 + i + 1 } else { -1 };
        let left = if i == 4 { 709 } else { -1 };
        let x = i as f64;
        let y = -14.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(left)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // === Sub-network 9: Straight line for trip generator (cells 801-818) ===
    // Y base = -16
    for i in 1..=18 {
        let cell_id = 800 + i;
        let forward = if i < 18 { 800 + i + 1 } else { -1 };
        let x = i as f64;
        let y = -16.0;
        cell_positions.insert(cell_id, (x, y));
        let cell = Cell::new(cell_id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // Create vehicles for each sub-network
    let vehicles: Vec<Vehicle> = vec![
        // Vehicle 1: L-shaped road, tail=2, will pass through
        Vehicle::new(1)
            .with_cell(3)
            .with_tail_size(2, vec![1, 2])
            .with_destination(20)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 2: Pingpong road, tail=2, will get stuck at 111
        Vehicle::new(2)
            .with_cell(103)
            .with_tail_size(2, vec![101, 102])
            .with_destination(116)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 3: Extended zigzag, tail=1, will pass through
        Vehicle::new(3)
            .with_cell(203)
            .with_tail_size(1, vec![202])
            .with_destination(216)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 4: Extended zigzag, tail=2, will get stuck at 312
        Vehicle::new(4)
            .with_cell(303)
            .with_tail_size(2, vec![301, 302])
            .with_destination(316)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 5: Merge conflict - upper path, tail=2, will conflict with V6 at merge point
        Vehicle::new(5)
            .with_cell(403)
            .with_tail_size(2, vec![401, 402])
            .with_destination(410)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 6: Merge conflict - lower path, tail=2, will conflict with V5 at merge point
        Vehicle::new(6)
            .with_cell(413)
            .with_tail_size(2, vec![411, 412])
            .with_destination(410)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 7: Crossing - west path, tail=1, goes SOUTH through intersection
        Vehicle::new(7)
            .with_cell(501)
            .with_tail_size(1, vec![-1])
            .with_destination(514)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 8: Crossing - north path, tail=5, enters intersection from above
        Vehicle::new(8)
            .with_cell(502)
            .with_tail_size(5, vec![-1, -1, -1, -1, -1])
            .with_destination(506)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 9: Will catch up to V10 which stops at dead-end (cell 615)
        Vehicle::new(9)
            .with_cell(603)
            .with_tail_size(2, vec![601, 602])
            .with_destination(615)
            .with_speed(3)
            .with_speed_limit(3)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 10: Starts near dead-end (615), will reach it and STAY STUCK
        Vehicle::new(10)
            .with_cell(612)
            .with_tail_size(2, vec![610, 611])
            .with_destination(620)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 11: Triple merge - upper path, tail=2
        Vehicle::new(11)
            .with_cell(702)
            .with_tail_size(2, vec![-1, 701])
            .with_destination(712)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 12: Triple merge - middle path, tail=2
        Vehicle::new(12)
            .with_cell(707)
            .with_tail_size(2, vec![705, 706])
            .with_destination(712)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
        // Vehicle 13: Triple merge - lower path, tail=2
        Vehicle::new(13)
            .with_cell(714)
            .with_tail_size(2, vec![-1, 713])
            .with_destination(712)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
    ];

    // Prepare simulation session
    let grids_storage = GridsStorage::new()
        .with_vehicles_net(grid)
        .build();
    let mut session = Session::new(grids_storage, None);
    session.set_verbose_level(VerboseLevel::None);
    session.add_vehicles(vehicles);

    // Add trip generator for sub-network 9: LargeBus with auto-resolved tail_size=2
    let trip = Trip::new(801, 818, TripType::Constant)
        .with_id(1)
        .with_allowed_agent_type(AgentType::LargeBus)
        .with_allowed_behaviour_type(BehaviourType::Aggressive)
        .with_time(5)
        .with_initial_speed(1)
        .with_speed_limit(1)
        .build();
    session.add_trip(trip);

    // Output directory
    let out_dir = "examples/all-tail/";

    // Write cells data
    write_cells_data(&cell_positions, out_dir);
    // Write edges data
    write_edges_data(&session, &cell_positions, out_dir);

    // Run simulation and collect states
    let time_sim = 25;
    let mut num_steps = 0;

    // Write initial state (step -1)
    write_vehicle_step(&session, &cell_positions, 0, out_dir);
    num_steps += 1;

    for step in 1..time_sim {
        match session.step() {
            Ok(_) => {
                write_vehicle_step(&session, &cell_positions, step as usize, out_dir);
                num_steps += 1;
            }
            Err(e) => {
                eprintln!("Error at step {}: {}", step, e);
                break;
            }
        }
    }

    // Write gnuplot config
    write_gnuplot_config(num_steps, out_dir);

    println!("Simulation complete ({} steps). Data written to:", num_steps);
    println!("  - cells.dat");
    println!("  - edges.dat");
    println!("  - vehicle_step_*.dat");
    println!("  - config.gnu");
    println!("\nRun 'gnuplot plot.gnu' to generate visualization");
}

fn write_cells_data(positions: &HashMap<i64, (f64, f64)>, out_dir: &str) {
    let filename = format!("{}cells.dat", out_dir);
    let mut file = File::create(&filename).expect("Failed to create cells.dat");
    writeln!(file, "# CellID X Y").unwrap();

    let mut sorted_ids: Vec<_> = positions.keys().collect();
    sorted_ids.sort();

    for cell_id in sorted_ids {
        let (x, y) = positions[cell_id];
        writeln!(file, "{} {:.1} {:.1}", cell_id, x, y).unwrap();
    }
}

fn write_edges_data(session: &Session, positions: &HashMap<i64, (f64, f64)>, out_dir: &str) {
    let filename = format!("{}edges.dat", out_dir);
    let mut file = File::create(&filename).expect("Failed to create edges.dat");
    writeln!(file, "# from_x from_y dx dy label").unwrap();

    let mut sorted_ids: Vec<_> = positions.keys().collect();
    sorted_ids.sort();

    for cell_id in sorted_ids {
        if let Some(cell) = session.get_cell(cell_id) {
            let (from_x, from_y) = positions[cell_id];

            let fwd_id = cell.get_forward_id();
            if fwd_id >= 0 {
                if let Some(&(to_x, to_y)) = positions.get(&fwd_id) {
                    let dx = to_x - from_x;
                    let dy = to_y - from_y;
                    writeln!(file, "{:.1} {:.1} {:.1} {:.1} F", from_x, from_y, dx, dy).unwrap();
                }
            }

            let right_id = cell.get_right_id();
            if right_id >= 0 {
                if let Some(&(to_x, to_y)) = positions.get(&right_id) {
                    let dx = to_x - from_x;
                    let dy = to_y - from_y;
                    writeln!(file, "{:.1} {:.1} {:.1} {:.1} R", from_x, from_y, dx, dy).unwrap();
                }
            }

            let left_id = cell.get_left_id();
            if left_id >= 0 {
                if let Some(&(to_x, to_y)) = positions.get(&left_id) {
                    let dx = to_x - from_x;
                    let dy = to_y - from_y;
                    writeln!(file, "{:.1} {:.1} {:.1} {:.1} L", from_x, from_y, dx, dy).unwrap();
                }
            }
        }
    }
}

fn write_vehicle_step(session: &Session, positions: &HashMap<i64, (f64, f64)>, step: usize, out_dir: &str) {
    let filename = format!("{}vehicle_step_{:02}.dat", out_dir, step);
    let mut file = File::create(&filename).expect("Failed to create vehicle step file");
    writeln!(file, "# X Y VehicleID Type").unwrap();

    for (_, vehicle) in session.get_vehicles() {
        // Write head position
        if let Some(&(x, y)) = positions.get(&vehicle.cell_id) {
            writeln!(file, "{:.1} {:.1} {} head", x, y, vehicle.id).unwrap();
        }

        // Write tail positions
        for tail_cell in &vehicle.tail_cells {
            if *tail_cell > 0 {
                if let Some(&(x, y)) = positions.get(tail_cell) {
                    writeln!(file, "{:.1} {:.1} {} tail", x, y, vehicle.id).unwrap();
                }
            }
        }
    }
}

fn write_gnuplot_config(num_steps: usize, out_dir: &str) {
    let filename = format!("{}config.gnu", out_dir);
    let mut file = File::create(&filename).expect("Failed to create config.gnu");
    writeln!(file, "# Auto-generated gnuplot config").unwrap();
    writeln!(file, "num_steps = {}", num_steps).unwrap();
}
