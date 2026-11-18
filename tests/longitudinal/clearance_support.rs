use micro_traffic_sim_core::agents::{SynchronizationParameters, Vehicle};
use micro_traffic_sim_core::geom::{SRID, new_point};
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::zones::ZoneType;
use micro_traffic_sim_core::simulation::session::Session;

pub fn road(end: i64) -> Session {
    let mut session = Session::default(Some(SRID::WGS84));
    session.add_cells(
        (0..=end)
            .map(|x| {
                Cell::new(x)
                    .with_forward_node(if x < end { x + 1 } else { -1 })
                    .with_left_node(-1)
                    .with_right_node(-1)
                    .with_speed_limit(5)
                    .with_zone_type(ZoneType::Common)
                    .with_meso_link(0)
                    .with_point(new_point(x as f64 * 1e-4, 0.0, Some(SRID::WGS84)))
                    .build()
            })
            .collect(),
    );
    session
}

pub fn car(id: u64, x: i64, speed: i32, gap: i32, sync: bool, end: i64) -> Vehicle {
    let mut car = Vehicle::new(id)
        .with_cell(x)
        .with_speed(speed)
        .with_speed_limit(5)
        .with_destination(-1)
        .with_trip_destination(end)
        .with_standstill_gap(gap)
        .with_slowdown(0.0)
        .with_slow_to_start(0.0)
        .build();
    if sync {
        car.synchronization = SynchronizationParameters {
            noise: 0.0,
            over_acceleration: 0.0,
            over_acceleration_low: 0.0,
            free_speed_over_acceleration: Some(0.0),
            ..SynchronizationParameters::COARSE_GRID
        };
    }
    car
}
