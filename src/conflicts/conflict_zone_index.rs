use crate::conflict_zones::{ConflictZone, ConflictZoneID};
use crate::grid::cell::CellID;
use crate::intentions::Intentions;
use std::collections::HashMap;

/// Target-cell incidence for crossing zones with distinct target cells.
///
/// Each target can belong to several zones. Update this index whenever a zone is
/// added or replaced. Merge zones with a common target keep their existing path.
#[derive(Debug, Default)]
pub struct CrossingZoneIndex {
    zones_by_target: HashMap<CellID, Vec<ConflictZoneID>>,
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
            for target in [first, second] {
                if let Some(ids) = self.zones_by_target.get_mut(&target) {
                    ids.retain(|&candidate| candidate != id);
                    if ids.is_empty() {
                        self.zones_by_target.remove(&target);
                    }
                }
            }
        }
        if targets.0 != targets.1 {
            self.targets_by_zone.insert(id, targets);
            for target in [targets.0, targets.1] {
                self.zones_by_target.entry(target).or_default().push(id);
            }
        }
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
