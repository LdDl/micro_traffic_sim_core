use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::{clearance::LongitudinalClearance, session::Session};

pub fn parallel_roads(policy: LongitudinalClearance, starts: &[i64]) -> Session {
    let mut session = Session::default(None);
    session.set_longitudinal_clearance(policy);
    let cells_per_road = 20;
    session.add_cells(
        starts
            .iter()
            .flat_map(|&start| {
                (start..start + cells_per_road).map(move |id| {
                    Cell::new(id)
                        .with_forward_node(if id == start + cells_per_road - 1 {
                            -1
                        } else {
                            id + 1
                        })
                        .with_left_node(-1)
                        .with_right_node(-1)
                        .with_speed_limit(5)
                        .build()
                })
            })
            .collect(),
    );
    session
}

pub fn traversed(vehicle: &Vehicle, start: i64, edge: (i64, i64)) -> bool {
    let mut previous = start;
    vehicle
        .intention
        .intermediate_cells
        .iter()
        .copied()
        .chain(std::iter::once(vehicle.cell_id))
        .any(|cell| {
            let matched = (previous, cell) == edge;
            previous = cell;
            matched
        })
}
