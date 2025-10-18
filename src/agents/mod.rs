//! Agents module.
//! This module contains the agent model (only vehicle currently) types used by the
//! simulation. It exposes helpers to create and configure vehicles and the
//! behavior enums used by intention and movement logic.
mod vehicle_intention;
mod vehicle;

pub use self::{vehicle_intention::*, vehicle::*};
