use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::session::Session;

pub fn road(speed_limit: i32) -> Session {
    let mut session = Session::default(None);
    session.add_cells(
        (1..=20)
            .map(|x| {
                Cell::new(x)
                    .with_point(new_point(x as f64, 0.0, None))
                    .with_forward_node(if x < 20 { x + 1 } else { -1 })
                    .with_left_node(-1)
                    .with_right_node(-1)
                    .with_speed_limit(speed_limit)
                    .build()
            })
            .collect(),
    );
    session
}
