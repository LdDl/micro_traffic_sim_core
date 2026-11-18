use super::*;
use crate::agents::Vehicle;
use crate::agents::{VehicleIntention, VehiclesStorage};
use crate::agents_types::AgentType;
use crate::geom::new_point;
use crate::grid::cell::{Cell, CellState};
use crate::grid::road_network::GridRoads;
use crate::grid::zones::ZoneType;
use crate::maneuver::LaneChangeType;
use crate::verbose::LocalLogger;

// A dwelling truck keeps its head and tail in their initial cells even when its intention
// contains a forward move.
#[test]
fn test_dwelling_tailed_vehicle_tail_does_not_advance() {
    // before and after: 1(tail) -> 2(tail) -> 3(head, dwelling) -> 4
    // The unused intention proposes head 4 and tail [2, 3].
    // Linear road 1 -> 2 -> 3 -> 4.
    let mut net = GridRoads::new();
    for (id, fwd, x) in [(1, 2, 0.0), (2, 3, 1.0), (3, 4, 2.0), (4, -1, 3.0)] {
        net.add_cell(
            Cell::new(id)
                .with_point(new_point(x, 0.0, None))
                .with_forward_node(fwd)
                .with_speed_limit(3)
                .build(),
        );
    }

    // The vehicle dwells with head 3 and tail [1, 2]. Its unused intention
    // proposes a forward move to 4 with projected tail [2, 3].
    let mut v = Vehicle::new(1)
        .with_cell(3)
        .with_tail_size(2, vec![1, 2])
        .with_destination(4)
        .with_relax_time(5)
        .build();
    v.relax_countdown_reset();
    assert!(
        v.get_relax_countdown() > 0,
        "precondition: vehicle is dwelling"
    );
    v.set_intention(VehicleIntention {
        intention_maneuver: LaneChangeType::NoChange,
        intention_cell_id: 4,
        intention_speed: 1,
        // the tail it WOULD have if it actually moved
        tail_intention_cells: vec![2, 3],
        ..Default::default()
    });

    let mut vehicles = VehiclesStorage::new();
    vehicles.insert(1, v);
    movement(&net, &mut vehicles, &LocalLogger::none()).unwrap();

    let v = vehicles
        .get(&1)
        .expect("vehicle stays (dwelling, not removed)");
    assert_eq!(v.cell_id, 3, "head stays put while dwelling");
    assert_eq!(
        v.tail_cells,
        vec![1, 2],
        "tail must NOT advance while the head dwells"
    );
    assert!(
        !v.tail_cells.contains(&v.cell_id),
        "head cell is not inside its own tail"
    );
}

// Arrival at the first transit stop updates the bus's next destination. Arrival at the
// final stop removes the other bus as completed.
#[test]
fn test_bus_completes_at_last_stop_not_first() {
    // bus A: 1 -> 2(first stop) -> 3 -> 4(final stop)
    // bus B:                      3 -> 4(final stop)
    // A remains and targets 4. B completes at 4.
    // Linear road 1 -> 2 -> 3 -> 4. Cells 2 and 4 are Transit stops.
    let mut net = GridRoads::new();
    for (id, fwd, x, zone) in [
        (1, 2, 0.0, ZoneType::Common),
        (2, 3, 1.0, ZoneType::Transit),
        (3, 4, 2.0, ZoneType::Common),
        (4, -1, 3.0, ZoneType::Transit),
    ] {
        net.add_cell(
            Cell::new(id)
                .with_point(new_point(x, 0.0, None))
                .with_forward_node(fwd)
                .with_speed_limit(3)
                .with_zone_type(zone)
                .build(),
        );
    }

    // Bus #1 arriving at the FIRST stop (cell 2): its immediate destination is the first
    // stop, but the trip only completes at the LAST stop (cell 4). It must survive the
    // stop and re-target to the next one.
    let mut bus_first = Vehicle::new(1)
        .with_type(AgentType::Bus)
        .with_cell(1)
        .with_speed_limit(3)
        .with_destination(2)
        .with_trip_destination(4)
        .with_transit_cells(vec![2, 4])
        .build();
    bus_first.set_intention(VehicleIntention {
        intention_maneuver: LaneChangeType::NoChange,
        intention_cell_id: 2,
        intention_speed: 1,
        ..Default::default()
    });

    // Bus #2 arriving at the LAST stop (cell 4), one transit already served: it must
    // despawn as completed here.
    let mut bus_last = Vehicle::new(2)
        .with_type(AgentType::Bus)
        .with_cell(3)
        .with_speed_limit(3)
        .with_destination(4)
        .with_trip_destination(4)
        .with_transit_cells(vec![2, 4])
        .build();
    // already served stop #1
    bus_last.transits_made_inc();
    bus_last.set_intention(VehicleIntention {
        intention_maneuver: LaneChangeType::NoChange,
        intention_cell_id: 4,
        intention_speed: 1,
        ..Default::default()
    });

    let mut vehicles = VehiclesStorage::new();
    vehicles.insert(1, bus_first);
    vehicles.insert(2, bus_last);
    let (completed, _lost) = movement(&net, &mut vehicles, &LocalLogger::none()).unwrap();

    // Only the bus at its LAST stop completes.
    assert_eq!(completed, 1, "exactly one bus (at its last stop) completes");

    let bus_first = vehicles
        .get(&1)
        .expect("bus must NOT despawn at the first stop");
    assert_eq!(bus_first.cell_id, 2, "bus #1 reached the first stop");
    assert_eq!(
        bus_first.destination, 4,
        "bus #1 re-targeted to the next (last) stop"
    );
    assert_eq!(
        bus_first.get_transits_made(),
        1,
        "bus #1 registered one transit"
    );

    assert!(
        vehicles.get(&2).is_none(),
        "bus #2 completed and was removed at the last stop"
    );
}

// Red-light and dwell holds leave both waiting clocks unchanged. A traffic hold increments
// both. Cell zero is also tested as a red-light target.
#[test]
fn test_wait_steps_not_accrued_on_red_or_dwell() {
    // 1(red hold)     -> 0 or 2(banned): both counters unchanged
    // 3(traffic hold) -> 4(free):        both counters + 1
    // 5(dwelling)     -> 6(free):        both counters unchanged
    for red_target in [0, 2] {
        let mut net = GridRoads::new();
        // The forward red-light target can be cell zero.
        net.add_cell(
            Cell::new(1)
                .with_speed_limit(3)
                .with_forward_node(red_target)
                .with_point(new_point(0.0, 0.0, None))
                .build(),
        );
        let mut banned = Cell::new(red_target)
            .with_speed_limit(3)
            .with_point(new_point(1.0, 0.0, None))
            .build();
        banned.set_state(CellState::Banned);
        net.add_cell(banned);
        // Traffic-block lane: 3 -> 4 (Free).
        net.add_cell(
            Cell::new(3)
                .with_speed_limit(3)
                .with_forward_node(4)
                .with_point(new_point(0.0, 1.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(4)
                .with_speed_limit(3)
                .with_point(new_point(1.0, 1.0, None))
                .build(),
        );
        // Dwell lane: 5 -> 6.
        net.add_cell(
            Cell::new(5)
                .with_speed_limit(3)
                .with_forward_node(6)
                .with_point(new_point(0.0, 2.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(6)
                .with_speed_limit(3)
                .with_point(new_point(1.0, 2.0, None))
                .build(),
        );

        // v1: held by the red light (forward Banned), blocked in place.
        let mut v1 = Vehicle::new(1).with_cell(1).build();
        v1.wait_steps = 10;
        v1.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            intention_cell_id: 1,
            intention_speed: 0,
            ..Default::default()
        });
        // v2: blocked by traffic (forward Free, not red), blocked in place -> control.
        let mut v2 = Vehicle::new(2).with_cell(3).build();
        v2.wait_steps = 10;
        v2.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            intention_cell_id: 3,
            intention_speed: 0,
            ..Default::default()
        });
        // v3: dwelling at a stop - intends to move but stays put this tick.
        let mut v3 = Vehicle::new(3).with_cell(5).with_relax_time(5).build();
        v3.relax_countdown_reset();
        v3.wait_steps = 10;
        v3.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 6,
            intention_speed: 1,
            ..Default::default()
        });

        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, v1);
        vehicles.insert(2, v2);
        vehicles.insert(3, v3);
        for vehicle in vehicles.values_mut() {
            vehicle.yield_cycle_wait_steps = 7;
            vehicle.hold_for_yield_cycle();
        }
        movement(&net, &mut vehicles, &LocalLogger::none()).unwrap();
        assert_eq!(vehicles[&1].yield_cycle_wait_steps, 7);
        assert_eq!(vehicles[&2].yield_cycle_wait_steps, 8);
        assert_eq!(vehicles[&3].yield_cycle_wait_steps, 7);

        assert_eq!(
            vehicles.get(&1).unwrap().wait_steps,
            10,
            "red light: patience must NOT accrue"
        );
        assert_eq!(
            vehicles.get(&2).unwrap().wait_steps,
            11,
            "traffic block: patience accrues"
        );
        assert_eq!(
            vehicles.get(&3).unwrap().wait_steps,
            10,
            "dwell: patience must NOT accrue"
        );
    }
}

// An executed lane change resets the yield-cycle clock while preserving the separate
// ordinary wait counter in this fixture.
#[test]
fn cycle_clock_clears_on_actual_lateral_progress() {
    let mut net = GridRoads::new();
    net.add_cell(Cell::new(1).with_left_node(2).with_speed_limit(1).build());
    net.add_cell(
        Cell::new(2)
            .with_forward_node(3)
            .with_speed_limit(1)
            .build(),
    );
    net.add_cell(Cell::new(3).with_speed_limit(1).build());
    let mut v = Vehicle::new(1).with_cell(1).with_speed(1).build();
    v.wait_steps = 10;
    v.yield_cycle_wait_steps = 4;
    v.hold_for_yield_cycle();
    v.set_intention(VehicleIntention {
        intention_maneuver: LaneChangeType::ChangeLeft,
        intention_cell_id: 2,
        intention_speed: 1,
        ..Default::default()
    });
    let mut vehicles = VehiclesStorage::new();
    vehicles.insert(1, v);
    movement(&net, &mut vehicles, &LocalLogger::none()).unwrap();
    assert_eq!(vehicles[&1].cell_id, 2);
    assert_eq!(vehicles[&1].yield_cycle_wait_steps, 0);
    assert_eq!(vehicles[&1].wait_steps, 10);
}
