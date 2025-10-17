//! Agents module.
//! This module contains the agent model (only vehicle currently) and behaviour types used by the
//! simulation. It exposes helpers to create and configure vehicles and the
//! behavior enums used by intention and movement logic.
mod agents;
mod behaviour;
mod vehicle_intention;
mod vehicle;

pub use self::{agents::*, behaviour::*, vehicle_intention::*, vehicle::*};
