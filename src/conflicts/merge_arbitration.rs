use super::conflicts::ConflictError;
use super::resolve_simple_rules;
use crate::agents::{Vehicle, VehicleID};
use crate::conflict_zones::{ConflictWinnerType, ConflictZone};
use crate::grid::cell::CellID;
use crate::intentions::{CellIntention, IntentionType};
use crate::utils::rand::random_bool;
use indexmap::IndexMap;
use std::collections::HashSet;

pub enum MergeDecision<'a> {
    Unregulated,
    Grant(&'a CellIntention),
    Yield,
}

// Only the constructor associates actual participants with a zone's two edges.
// The references remain tied to the immutable intention snapshot used to match.
struct MatchedMerge<'a> {
    first: &'a CellIntention,
    second: &'a CellIntention,
}

impl<'a> MatchedMerge<'a> {
    fn new(
        target: CellID,
        zone: &ConflictZone,
        a: &'a CellIntention,
        va: &Vehicle,
        b: &'a CellIntention,
        vb: &Vehicle,
    ) -> Option<Self> {
        let first = zone.get_first_edge();
        let second = zone.get_second_edge();
        if first.target != target
            || second.target != target
            || first.source == second.source
            || va.id == vb.id
            || a.get_vehicle_id() != va.id
            || b.get_vehicle_id() != vb.id
            || a.int_type == IntentionType::Tail
            || b.int_type == IntentionType::Tail
        {
            return None;
        }
        if intention_traverses_edge(va, first.source, target)
            && intention_traverses_edge(vb, second.source, target)
        {
            Some(Self {
                first: a,
                second: b,
            })
        } else {
            None
        }
    }

    fn priority(&self, first_wins: bool) -> (&'a CellIntention, &'a CellIntention) {
        if first_wins {
            (self.first, self.second)
        } else {
            (self.second, self.first)
        }
    }
}

// A grant must have no incoming yield obligation from any matched zone. Selecting
// only after collecting all obligations avoids losing a constraint in a pair fold.
pub fn arbitrate<'a>(
    target: CellID,
    intentions: &[&'a CellIntention],
    zones: &[&ConflictZone],
    vehicles: &IndexMap<VehicleID, Vehicle>,
) -> Result<MergeDecision<'a>, ConflictError> {
    if zones.is_empty() || intentions.iter().any(|i| i.int_type == IntentionType::Tail) {
        return Ok(MergeDecision::Unregulated);
    }
    if let [a, b] = intentions {
        return arbitrate_pair(target, a, b, zones, vehicles);
    }
    arbitrate_group(target, intentions, zones, vehicles)
}

fn arbitrate_pair<'a>(
    target: CellID,
    a: &'a CellIntention,
    b: &'a CellIntention,
    zones: &[&ConflictZone],
    vehicles: &IndexMap<VehicleID, Vehicle>,
) -> Result<MergeDecision<'a>, ConflictError> {
    let owner = |claim: &CellIntention| {
        let id = claim.get_vehicle_id();
        vehicles.get(&id).filter(|v| v.id == id).ok_or_else(|| {
            ConflictError::InvalidVehicle(format!("Vehicle {} not found or mismatched", id))
        })
    };
    let va = owner(a)?;
    let vb = owner(b)?;
    let mut a_yields = false;
    let mut b_yields = false;
    for zone in zones {
        let forward = MatchedMerge::new(target, zone, a, va, b, vb);
        let reverse = MatchedMerge::new(target, zone, b, vb, a, va);
        if forward.is_none() && reverse.is_none() {
            continue;
        }
        let first_wins = match zone.get_winner_type() {
            ConflictWinnerType::First => true,
            ConflictWinnerType::Second => false,
            _ => random_bool(0.5),
        };
        for matched in forward.iter().chain(reverse.iter()) {
            let (_, loser) = matched.priority(first_wins);
            a_yields |= loser.get_vehicle_id() == a.get_vehicle_id();
            b_yields |= loser.get_vehicle_id() == b.get_vehicle_id();
        }
    }
    Ok(match (a_yields, b_yields) {
        (false, false) => MergeDecision::Unregulated,
        (false, true) => MergeDecision::Grant(a),
        (true, false) => MergeDecision::Grant(b),
        (true, true) => MergeDecision::Yield,
    })
}

fn arbitrate_group<'a>(
    target: CellID,
    intentions: &[&'a CellIntention],
    zones: &[&ConflictZone],
    vehicles: &IndexMap<VehicleID, Vehicle>,
) -> Result<MergeDecision<'a>, ConflictError> {
    let participants: Vec<_> = intentions
        .iter()
        .map(|intention| {
            let id = intention.get_vehicle_id();
            vehicles.get(&id).filter(|v| v.id == id).ok_or_else(|| {
                ConflictError::InvalidVehicle(format!("Vehicle {} not found or mismatched", id))
            })
        })
        .collect::<Result<_, _>>()?;
    let mut yielding = HashSet::new();
    for zone in zones {
        let mut first_wins = None;
        for a in 0..intentions.len() {
            for b in 0..intentions.len() {
                if let Some(matched) = MatchedMerge::new(
                    target,
                    zone,
                    intentions[a],
                    participants[a],
                    intentions[b],
                    participants[b],
                ) {
                    // One draw per applicable zone, after both approaches are verified.
                    let priority =
                        *first_wins.get_or_insert_with(|| match zone.get_winner_type() {
                            ConflictWinnerType::First => true,
                            ConflictWinnerType::Second => false,
                            _ => random_bool(0.5),
                        });
                    let (_, loser) = matched.priority(priority);
                    yielding.insert(loser.get_vehicle_id());
                }
            }
        }
    }
    if yielding.is_empty() {
        return Ok(MergeDecision::Unregulated);
    }
    let mut eligible = intentions
        .iter()
        .copied()
        .filter(|i| !yielding.contains(&i.get_vehicle_id()));
    let Some(mut winner) = eligible.next() else {
        return Ok(MergeDecision::Yield);
    };
    for other in eligible {
        if winner.get_vehicle_id() != other.get_vehicle_id() {
            winner = resolve_simple_rules(winner, other, vehicles).0;
        }
    }
    Ok(MergeDecision::Grant(winner))
}

/// Checks the ordered path, including approach cells before the conflict edge.
pub fn intention_traverses_edge(vehicle: &Vehicle, source: CellID, target: CellID) -> bool {
    let mut previous = vehicle.cell_id;
    vehicle
        .intention
        .intermediate_cells
        .iter()
        .copied()
        .chain(std::iter::once(vehicle.intention.intention_cell_id))
        .any(|cell| {
            let traverses = previous == source && cell == target;
            previous = cell;
            traverses
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::VehicleIntention;
    use crate::conflict_zones::ConflictEdge;
    use crate::utils::rand::random_f64;

    fn sample(mode: u8) -> (Option<VehicleID>, Vec<u64>) {
        let mut vehicles = IndexMap::new();
        for (id, source) in [(1, 100), (2, 200), (3, 200)] {
            let mut vehicle = Vehicle::new(id).with_cell(source).with_speed(1).build();
            vehicle.set_intention(VehicleIntention {
                intention_maneuver: crate::maneuver::LaneChangeType::NoChange,
                intention_cell_id: 900,
                intention_speed: 1,
                ..Default::default()
            });
            vehicles.insert(id, vehicle);
        }
        let claims: Vec<_> = vehicles
            .keys()
            .map(|&id| CellIntention::new(id, IntentionType::Target))
            .collect();
        let refs: Vec<_> = claims.iter().collect();
        let zone = ConflictZone::new(
            1,
            ConflictEdge {
                source: if mode == 1 { 999 } else { 100 },
                target: 900,
            },
            ConflictEdge {
                source: 200,
                target: 900,
            },
        )
        .with_winner_type(ConflictWinnerType::Equal)
        .build();
        let winner = match mode {
            0 => None,
            3 => Some(if random_bool(0.5) { 1 } else { 2 }),
            _ => match arbitrate(900, &refs, &[&zone], &vehicles).unwrap() {
                MergeDecision::Grant(winner) => Some(winner.get_vehicle_id()),
                MergeDecision::Unregulated => None,
                MergeDecision::Yield => panic!("One two-approach zone cannot form a cycle"),
            },
        };
        (winner, (0..32).map(|_| random_f64().to_bits()).collect())
    }

    fn fresh_sample(mode: u8) -> (Option<VehicleID>, Vec<u64>) {
        std::thread::spawn(move || sample(mode)).join().unwrap()
    }

    #[test]
    fn unrelated_merge_does_not_consume_randomness() {
        assert_eq!(fresh_sample(0), fresh_sample(1));
    }

    #[test]
    fn equal_priority_uses_one_draw_for_every_participant_on_an_approach() {
        assert_eq!(fresh_sample(2), fresh_sample(3));
    }
    #[test]
    fn pair_fast_path_matches_group_constraints_and_random_stream() {
        fn decision_id(d: MergeDecision<'_>) -> (u8, VehicleID) {
            match d {
                MergeDecision::Unregulated => (0, 0),
                MergeDecision::Grant(winner) => (1, winner.get_vehicle_id()),
                MergeDecision::Yield => (2, 0),
            }
        }
        fn run(pair: bool) -> Vec<((u8, VehicleID), u64)> {
            let paths = [
                vec![100, 900],
                vec![200, 900],
                vec![100, 800, 900],
                vec![100, 900, 200, 900],
                vec![200, 900, 100, 900],
            ];
            let priorities = [
                None,
                Some(ConflictWinnerType::First),
                Some(ConflictWinnerType::Second),
                Some(ConflictWinnerType::Equal),
                Some(ConflictWinnerType::Undefined),
            ];
            let a = CellIntention::new(1, IntentionType::Target);
            let b = CellIntention::new(2, IntentionType::Target);
            let mut records = Vec::new();
            for pa in &paths {
                for pb in &paths {
                    let mut vehicles = IndexMap::new();
                    for (id, path) in [(1, pa), (2, pb)] {
                        let mut v = Vehicle::new(id).with_cell(path[0]).with_speed(1).build();
                        v.set_intention(VehicleIntention {
                            intention_maneuver: crate::maneuver::LaneChangeType::NoChange,
                            intention_cell_id: 900,
                            intermediate_cells: path[1..path.len() - 1].to_vec(),
                            intention_speed: 1,
                            ..Default::default()
                        });
                        vehicles.insert(id, v);
                    }
                    for p in priorities {
                        for q in priorities {
                            let zones: Vec<_> = [(1, 100, 200, p), (2, 200, 100, q)]
                                .into_iter()
                                .filter_map(|(id, first, second, priority)| {
                                    priority.map(|priority| {
                                        ConflictZone::new(
                                            id,
                                            ConflictEdge {
                                                source: first,
                                                target: 900,
                                            },
                                            ConflictEdge {
                                                source: second,
                                                target: 900,
                                            },
                                        )
                                        .with_winner_type(priority)
                                        .build()
                                    })
                                })
                                .collect();
                            let refs: Vec<_> = zones.iter().collect();
                            let d = if pair {
                                arbitrate_pair(900, &a, &b, &refs, &vehicles)
                            } else {
                                arbitrate_group(900, &[&a, &b], &refs, &vehicles)
                            }
                            .unwrap();
                            records.push((decision_id(d), random_f64().to_bits()));
                        }
                    }
                }
            }
            assert_eq!(records.len(), 625);
            records
        }
        let fast = std::thread::spawn(|| run(true)).join().unwrap();
        let general = std::thread::spawn(|| run(false)).join().unwrap();
        assert_eq!(fast, general);
    }
}
