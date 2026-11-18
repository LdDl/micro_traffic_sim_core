use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::trips::trip::{Trip, TripType};

fn road() -> Session {
    let mut session = Session::default(None);
    session.add_cells((1..=20).map(|x| Cell::new(x)
        .with_point(new_point(x as f64, 0.0, None))
        .with_forward_node(if x < 20 { x+1 } else { -1 })
        .with_left_node(-1).with_right_node(-1)
        .with_speed_limit(1).build()).collect());
    session
}

fn car(id: u64, cell: i64, reserve: i32) -> Vehicle {
    Vehicle::new(id).with_cell(cell).with_destination(-1).with_speed(1).with_speed_limit(1)
        .with_standstill_gap(reserve).with_slowdown(0.0).with_slow_to_start(0.0).build()
}

fn trip(source: i64, destination: i64, reserve: i32) -> Trip {
    Trip::new(source,destination,TripType::Constant).with_id(source).with_time(1)
        .with_allowed_behaviour_type(BehaviourType::Aggressive).with_standstill_gap(reserve).build()
}

#[test]
fn source_uses_current_added_bodies() {
    let mut session=road();session.add_vehicles(vec![car(10,1,0)]);
    session.add_trip(trip(1,20,0));session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(),1);
}

#[test]
fn source_rejects_cell_occupied_after_movement() {
    let mut session=road();session.add_vehicles(vec![car(10,1,0)]);
    session.step().unwrap();assert_eq!(session.get_vehicles()[&10].cell_id,2);
    session.add_trip(trip(2,20,0));session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(),1);
}

#[test]
fn source_can_use_a_cell_vacated_this_tick() {
    let mut session=road();session.add_vehicles(vec![car(10,1,0)]);
    session.step().unwrap();assert_eq!(session.get_vehicles()[&10].cell_id,2);
    session.add_trip(trip(1,20,0));session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(),2);
}

#[test]
fn entrant_must_have_its_own_reserved_space() {
    let mut session=road();session.add_vehicles(vec![car(10,2,0)]);
    session.add_trip(trip(1,20,1));session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(),1);
}

#[test]
fn entrant_cannot_occupy_an_existing_forward_reserve() {
    let mut session=road();session.add_vehicles(vec![car(10,1,1)]);
    session.add_trip(trip(2,20,0));session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(),1);
}

#[test]
fn same_generation_sources_respect_new_reserves() {
    let mut session=road();session.add_trip(trip(1,20,1));session.add_trip(trip(2,20,0));
    session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(),1);
    assert_eq!(session.get_vehicles().values().next().unwrap().cell_id,1);
}

#[test]
fn zero_reserve_sources_can_be_adjacent() {
    let mut session=road();session.add_trip(trip(1,20,0));session.add_trip(trip(2,20,0));
    session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(),2);
}

#[test]
fn cached_exit_does_not_reserve_an_abandoned_forward_branch() {
    let mut session=road();let mut departing=car(10,1,1);
    session.add_cells(vec![Cell::new(1).with_point(new_point(1.0,0.0,None))
        .with_forward_node(2).with_left_node(-1).with_right_node(3).with_speed_limit(1).build()]);
    departing.destination=3;departing.cached_route=vec![1,3];departing.route_idx=0;
    session.add_vehicles(vec![departing]);session.add_trip(trip(2,20,0));session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(),2);
}

#[test]
fn entrant_cached_exit_ignores_the_unused_forward_queue() {
    let mut session=road();session.add_vehicles(vec![car(10,2,0)]);
    session.add_cells(vec![Cell::new(1).with_point(new_point(1.0,0.0,None))
        .with_forward_node(2).with_left_node(-1).with_right_node(3).with_speed_limit(1).build(),
        Cell::new(2).with_point(new_point(2.0,0.0,None))
        .with_forward_node(-1).with_left_node(-1).with_right_node(-1).with_speed_limit(1).build()]);
    session.add_trip(trip(1,3,1));session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(),2);
}
