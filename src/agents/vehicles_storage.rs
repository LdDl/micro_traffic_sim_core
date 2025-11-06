use crate::agents::{Vehicle, VehicleID};
use indexmap::IndexMap;
use std::ops::{Deref, DerefMut};

/// Just a storage for vehicles used across the simulation.
///
/// This wraps an internal IndexMap<VehicleID, Vehicle> but hides the
/// concrete map type from end-users (mostly developers, huh?),
/// allowing to evolve internals without breaking public APIs.
/// It implements Deref/DerefMut to the underlying map
/// so it can be passed to functions that expect
/// `&IndexMap<VehicleID, Vehicle>` or `&mut IndexMap<VehicleID, Vehicle>`.
#[derive(Debug, Default)]
pub struct VehiclesStorage(IndexMap<VehicleID, Vehicle>);

impl VehiclesStorage {
    /// Create empty vehicles storage
    pub fn new() -> Self { Self(IndexMap::new()) }

    /// Insert a vehicle by its id (vehicle.id is used as the key)
    pub fn insert_vehicle(&mut self, vehicle: Vehicle) {
        let id = vehicle.id;
        self.0.insert(id, vehicle);
    }

    /// Number of vehicles
    pub fn len(&self) -> usize { self.0.len() }

    /// Whether storage is empty
    pub fn is_empty(&self) -> bool { self.0.is_empty() }

    /// Immutable iterator over (&VehicleID, &Vehicle)
    pub fn iter(&self) -> indexmap::map::Iter<'_, VehicleID, Vehicle> { self.0.iter() }

    /// Mutable iterator over (&VehicleID, &mut Vehicle)
    pub fn iter_mut(&mut self) -> indexmap::map::IterMut<'_, VehicleID, Vehicle> { self.0.iter_mut() }

    /// Convenience: values iterator
    pub fn values(&self) -> indexmap::map::Values<'_, VehicleID, Vehicle> { self.0.values() }

    /// Convenience: mutable values iterator
    pub fn values_mut(&mut self) -> indexmap::map::ValuesMut<'_, VehicleID, Vehicle> { self.0.values_mut() }
}

// Allow transparent access to IndexMap API and deref-coercions in function calls
impl Deref for VehiclesStorage {
    type Target = IndexMap<VehicleID, Vehicle>;
    fn deref(&self) -> &Self::Target { &self.0 }
}
impl DerefMut for VehiclesStorage {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.0 }
}

// Make `for (id, v) in &vehicles_storage { .. }` work like for &IndexMap
impl<'a> IntoIterator for &'a VehiclesStorage {
    type Item = (&'a VehicleID, &'a Vehicle);
    type IntoIter = indexmap::map::Iter<'a, VehicleID, Vehicle>;
    fn into_iter(self) -> Self::IntoIter { self.0.iter() }
}

// Mutable iteration with `for (id, v) in &mut vehicles_storage { .. }`
impl<'a> IntoIterator for &'a mut VehiclesStorage {
    type Item = (&'a VehicleID, &'a mut Vehicle);
    type IntoIter = indexmap::map::IterMut<'a, VehicleID, Vehicle>;
    fn into_iter(self) -> Self::IntoIter { self.0.iter_mut() }
}
