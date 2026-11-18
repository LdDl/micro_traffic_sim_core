use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::trips::trip::{Trip, TripType};

#[test]
fn native_source_admission_matches_independent_distance_oracle() {
    let populations = [
        (BehaviourType::Aggressive, 0, 5),
        (BehaviourType::Cooperative, 1, 4),
        (BehaviourType::LimitSpeedByTrip, 1, 3),
        (BehaviourType::Undefined, 1, 2),
    ];
    let mut checked = 0;
    let mut accepted = 0;
    for (behaviour, native_gap, desired) in populations {
        for configured in -1..=2 {
            let gap = if configured < 0 { native_gap } else { configured };
            for rear_gap in 0..=3 {
                for rear in 0..=6 {
                    for leader in 6..=13 {
                        if rear == leader {
                            continue;
                        }
                        let mut session = Session::default(None);
                        session.add_cells((1..=20).map(|x| Cell::new(x)
                            .with_point(new_point(x as f64, 0.0, None))
                            .with_forward_node(if x < 20 { x + 1 } else { -1 })
                            .with_left_node(-1).with_right_node(-1).with_speed_limit(5)
                            .build()).collect());
                        let mut cars = Vec::new();
                        if rear > 0 {
                            cars.push(Vehicle::new(10).with_cell(rear)
                                .with_destination(-1).with_standstill_gap(rear_gap).build());
                        }
                        if leader < 13 {
                            cars.push(Vehicle::new(20).with_cell(leader)
                                .with_destination(-1).with_standstill_gap(0).build());
                        }
                        session.add_vehicles(cars);
                        let count = session.get_vehicles().len();
                        let next = session.get_last_vehicle_id();
                        session.add_trip(Trip::new(6, 20, TripType::Constant)
                            .with_id(1).with_time(1).with_allowed_behaviour_type(behaviour)
                            .with_standstill_gap(configured).build());
                        // This oracle uses scalar distances, without traversing the grid or cached route.
                        let admit = rear != 6 && leader != 6
                            && (rear == 0 || 6 - rear - 1 >= i64::from(rear_gap))
                            && (leader == 13 || leader - 6 - 1 >= i64::from(gap));
                        session.generate_vehicles();
                        assert_eq!(session.get_vehicles().len(), count + usize::from(admit),
                            "behaviour={behaviour:?}, configured={configured}, rear_gap={rear_gap}, rear={rear}, leader={leader}");
                        assert_eq!(session.get_last_vehicle_id(), next + u64::from(admit));
                        if admit {
                            let entered = &session.get_vehicles()[&next];
                            assert_eq!(entered.cell_id, 6);
                            assert_eq!(entered.standstill_gap, gap);
                            assert_eq!(entered.speed_limit, desired);
                            accepted += 1;
                        }
                        checked += 1;
                    }
                }
            }
        }
    }
    assert_eq!(checked, 3520);
    println!("checked,accepted\n{checked},{accepted}");
}
