use super::*;
use crate::behaviour::RelativeRandomization;

// Aggressive and Cooperative display as their lowercase external labels.
#[test]
fn test_behaviour_display() {
    assert_eq!(format!("{}", BehaviourType::Aggressive), "aggressive");
    assert_eq!(format!("{}", BehaviourType::Cooperative), "cooperative");
}

// The Aggressive preset exposes the expected slowdown, desired speed, aggression, and
// safety margin.
#[test]
fn test_aggressive_preset_parameters() {
    let params = BehaviourParameters::from_behaviour_type(BehaviourType::Aggressive);
    assert_eq!(params.slowdown_factor_p(), 0.1);
    assert_eq!(params.speed_limit(), 5);
    assert_eq!(params.aggressive_level(), 0.9);
    assert_eq!(params.min_safe_distance(), 0);
}

// Moving presets share p0 = 0.425 with p0 >= p. Block uses p0 = 1. This checks preset
// values, not an empirical jam-line fit.
#[test]
fn test_slow_to_start_presets_use_line_j_value() {
    let p0 = |b| BehaviourParameters::from_behaviour_type(b).slow_to_start_factor_p0();
    for b in [
        BehaviourType::Aggressive,
        BehaviourType::Cooperative,
        BehaviourType::LimitSpeedByTrip,
        BehaviourType::Undefined,
    ] {
        assert_eq!(p0(b), 0.425);
        assert!(p0(b) >= BehaviourParameters::from_behaviour_type(b).slowdown_factor_p());
    }
    assert_eq!(p0(BehaviourType::Block), 1.0);
}

// Preset gaps match their configured empty-cell reserves. The basic constructor and a
// negative override both give zero.
#[test]
fn test_standstill_gap_presets() {
    let gap = |b| BehaviourParameters::from_behaviour_type(b).standstill_gap();
    assert_eq!(gap(BehaviourType::Cooperative), 1);
    assert_eq!(gap(BehaviourType::LimitSpeedByTrip), 1);
    assert_eq!(gap(BehaviourType::Aggressive), 0);
    assert_eq!(gap(BehaviourType::Undefined), 1);
    assert_eq!(gap(BehaviourType::Block), 0);
    assert_eq!(
        BehaviourParameters::new(0.1, 4, 0.2, 0, 0).standstill_gap(),
        0
    );
    assert_eq!(
        BehaviourParameters::new(0.1, 4, 0.2, 0, 0)
            .with_standstill_gap(-3)
            .standstill_gap(),
        0
    );
}

// Moving presets select KKW randomization, the basic constructor selects VDR, and custom
// negative thresholds and decelerations are clamped.
#[test]
fn test_relative_randomization_presets() {
    for b in [
        BehaviourType::Aggressive,
        BehaviourType::Cooperative,
        BehaviourType::LimitSpeedByTrip,
        BehaviourType::Undefined,
    ] {
        assert_eq!(
            BehaviourParameters::from_behaviour_type(b).relative_randomization(),
            RelativeRandomization::KKW
        );
    }
    assert_eq!(
        BehaviourParameters::new(0.1, 4, 0.2, 0, 0).relative_randomization(),
        RelativeRandomization::VDR
    );
    let custom = RelativeRandomization {
        stop_time_threshold: -1,
        decel_slower: -2,
        decel_equal: 1,
        decel_faster: 2,
        lookahead: 10,
    };
    let params = BehaviourParameters::new(0.1, 4, 0.2, 0, 0).with_relative_randomization(custom);
    assert_eq!(params.relative_randomization().stop_time_threshold, 0);
    assert_eq!(params.relative_randomization().decel_slower, 0);
    assert_eq!(params.relative_randomization().decel_faster, 2);
}

// The three presets map aggression through P1 = 0.5 + 0.5 * aggression to 0.5, 0.75, and
// 0.95.
#[test]
fn test_change_p1_maps_aggressiveness() {
    // P1 = 0.5 + 0.5 * aggressive_level, in [0.5, 1.0].
    // aggr 0.0
    let coop = BehaviourParameters::from_behaviour_type(BehaviourType::Cooperative);
    assert_eq!(coop.change_p1(), 0.5, "cooperative -> floor 0.5");
    // aggr 0.5
    let undef = BehaviourParameters::from_behaviour_type(BehaviourType::Undefined);
    assert_eq!(undef.change_p1(), 0.75, "middling -> 0.75");
    // aggr 0.9
    let aggr = BehaviourParameters::from_behaviour_type(BehaviourType::Aggressive);
    assert!(
        (aggr.change_p1() - 0.95).abs() < 1e-9,
        "aggressive -> ~0.95"
    );
}

// With the supplied mixture weights, sampled behaviours belong to Aggressive or
// Cooperative. Their frequencies are not tested.
#[test]
fn test_random_vehicle_behaviour_type() {
    let ratio_a = 0.7;
    let ratio_b = 0.3;

    for _ in 0..100 {
        let random_behaviour: BehaviourType =
            BehaviourType::random_vehicle_behaviour_type(ratio_a, ratio_b);
        assert!(
            random_behaviour == BehaviourType::Aggressive
                || random_behaviour == BehaviourType::Cooperative,
            "Random behaviour must be Aggressive or Cooperative. Other have not been implemented yet, got: {:?}",
            random_behaviour
        );
    }
}
