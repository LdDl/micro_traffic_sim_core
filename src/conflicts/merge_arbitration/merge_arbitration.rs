use crate::conflicts::conflict_types::ConflictError;
use crate::conflicts::conflict_rule::resolve_simple_rules;
use crate::agents::{Vehicle, VehicleID};
use crate::conflict_zones::{ConflictWinnerType, ConflictZone};
use crate::grid::cell::CellID;
use crate::intentions::{CellIntention, IntentionType};
use crate::utils::rand::random_bool;
use indexmap::IndexMap;
use std::cmp::Reverse;
use std::collections::HashSet;

pub enum MergeDecision<'a> {
    Unregulated,
    Grant(&'a CellIntention),
    Yield(&'a CellIntention),
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
        (true, true) => MergeDecision::Yield(if cycle_rank(va) >= cycle_rank(vb) {
            a
        } else {
            b
        }),
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
    let mut obligations = Vec::new();
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
                    let (winner, loser) = matched.priority(priority);
                    yielding.insert(loser.get_vehicle_id());
                    obligations.push((winner.get_vehicle_id(), loser.get_vehicle_id()));
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
        return Ok(MergeDecision::Yield(cycle_candidate(
            intentions,
            &obligations,
            vehicles,
        )));
    };
    for other in eligible {
        if winner.get_vehicle_id() != other.get_vehicle_id() {
            winner = resolve_simple_rules(winner, other, vehicles).0;
        }
    }
    Ok(MergeDecision::Grant(winner))
}

fn cycle_rank(vehicle: &Vehicle) -> (bool, i32, Reverse<VehicleID>) {
    (
        vehicle.yield_cycle_ready(),
        vehicle.yield_cycle_wait_steps,
        Reverse(vehicle.id),
    )
}

// A source strongly connected component has no incoming priority from outside.
// Since every contender yields, a source component necessarily contains a cycle.
// Only these components may supply a candidate; ordinary external priorities remain.
fn cycle_candidate<'a>(
    intentions: &[&'a CellIntention],
    obligations: &[(VehicleID, VehicleID)],
    vehicles: &IndexMap<VehicleID, Vehicle>,
) -> &'a CellIntention {
    let mut vertices = IndexMap::new();
    for intention in intentions {
        let next = vertices.len();
        vertices.entry(intention.get_vehicle_id()).or_insert(next);
    }
    let n = vertices.len();
    let mut adjacency = vec![Vec::new(); n];
    for (winner, loser) in obligations {
        adjacency[vertices[winner]].push(vertices[loser]);
    }
    let mut reachable = vec![false; n * n];
    let mut stack = Vec::new();
    for source in 0..n {
        stack.push(source);
        while let Some(vertex) = stack.pop() {
            if reachable[source * n + vertex] {
                continue;
            }
            reachable[source * n + vertex] = true;
            stack.extend(adjacency[vertex].iter().copied());
        }
    }
    intentions
        .iter()
        .copied()
        .filter(|intention| {
            let i = vertices[&intention.get_vehicle_id()];
            (0..n).all(|j| !reachable[j * n + i] || reachable[i * n + j])
        })
        .max_by_key(|intention| cycle_rank(&vehicles[&intention.get_vehicle_id()]))
        .expect("A finite nonempty priority graph has a source component")
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
#[path = "tests.rs"]
mod tests;
