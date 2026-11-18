//! Pairwise right-of-way rules and driver priority comparisons.

mod conflict_rule;

pub use self::conflict_rule::{
    aggressor_advantage, changing_lane, resolve_by_speed_and_cooperativity, resolve_merge_forward,
    resolve_merge_lane_change, resolve_simple_rules,
};
