//! Vehicle state, movement intentions, and vehicle storage. Shared dynamics parameters are re-exported from behaviour.

mod vehicle;
mod vehicle_intention;
mod vehicles_storage;

pub use self::{vehicle::*, vehicle_intention::*, vehicles_storage::VehiclesStorage};
pub use crate::behaviour::{RelativeRandomization, SynchronizationParameters};
