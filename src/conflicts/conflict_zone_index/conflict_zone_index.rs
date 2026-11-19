use crate::conflict_zones::{ConflictZone, ConflictZoneID};
use crate::grid::cell::CellID;
use crate::intentions::Intentions;
use std::collections::HashMap;

/// Target-cell incidence for crossing zones with distinct target cells.
///
/// Each target can belong to several zones. Update this index whenever a zone is
/// added or replaced. Merge zones have a separate incidence with the same lifecycle.
#[derive(Debug, Default)]
pub struct CrossingZoneIndex {
    zones_by_target: HashMap<CellID, Vec<ConflictZoneID>>,
    merge_zones_by_target: HashMap<CellID, Vec<ConflictZoneID>>,
    targets_by_zone: HashMap<ConflictZoneID, (CellID, CellID)>,
}

impl CrossingZoneIndex {
    /// Updates the incidence of one zone, removing its previous endpoints first.
    ///
    /// # Arguments
    /// * `zone` - The current definition of the added or replaced zone.
    pub fn replace(&mut self, zone: &ConflictZone) {
        let id = zone.get_id();
        let targets = (zone.get_first_edge().target, zone.get_second_edge().target);
        if self.targets_by_zone.get(&id) == Some(&targets) {
            return;
        }
        if let Some((first, second)) = self.targets_by_zone.remove(&id) {
            let index = if first == second {
                &mut self.merge_zones_by_target
            } else {
                &mut self.zones_by_target
            };
            for target in [first, second] {
                if let Some(ids) = index.get_mut(&target) {
                    ids.retain(|&candidate| candidate != id);
                    if ids.is_empty() {
                        index.remove(&target);
                    }
                }
            }
        }
        self.targets_by_zone.insert(id, targets);
        if targets.0 == targets.1 {
            let ids = self.merge_zones_by_target.entry(targets.0).or_default();
            ids.push(id);
            ids.sort_unstable();
        } else {
            for target in [targets.0, targets.1] {
                self.zones_by_target.entry(target).or_default().push(id);
            }
        }
    }

    /// Returns every merge zone at one target, in priority-draw order.
    ///
    /// # Arguments
    /// * `target` - Cell claimed by the current contenders.
    pub fn merge_candidates(&self, target: CellID) -> &[ConflictZoneID] {
        self.merge_zones_by_target
            .get(&target)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    /// Returns active crossing and merge zones, retaining every shared-target zone.
    ///
    /// # Arguments
    /// * `intentions` - Immutable current-tick target, transit and tail claims.
    pub fn all_candidates(&self, intentions: &Intentions) -> Vec<ConflictZoneID> {
        let mut ids = self.candidates(intentions);
        if self.merge_zones_by_target.is_empty() {
            return ids;
        }
        if self.merge_zones_by_target.len() < intentions.len() {
            for (target, zones) in &self.merge_zones_by_target {
                if intentions.get(target).is_some() {
                    ids.extend_from_slice(zones);
                }
            }
        } else {
            for (target, _) in intentions.iter() {
                if let Some(zones) = self.merge_zones_by_target.get(target) {
                    ids.extend_from_slice(zones);
                }
            }
        }
        ids.sort_unstable();
        ids.dedup();
        ids
    }

    /// Returns sorted distinct zones touching at least one intended target cell.
    ///
    /// One-sided candidates are retained for the same validation and legacy-zone
    /// suppression as the full scan. Participant and edge checks happen later.
    ///
    /// # Arguments
    /// * `intentions` - Initial grants including target, intermediate and tail cells.
    pub fn candidates(&self, intentions: &Intentions) -> Vec<ConflictZoneID> {
        let mut ids = Vec::new();
        if self.zones_by_target.len() < intentions.len() {
            for (target, zones) in &self.zones_by_target {
                if intentions.get(target).is_some() {
                    ids.extend_from_slice(zones);
                }
            }
        } else {
            for (target, _) in intentions.iter() {
                if let Some(zones) = self.zones_by_target.get(target) {
                    ids.extend_from_slice(zones);
                }
            }
        }
        ids.sort_unstable();
        ids.dedup();
        ids
    }
}
