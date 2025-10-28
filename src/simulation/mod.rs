//! # Simulation module
//!
//! **Top-level orchestration module** – Coordinates all major components of the cellular automata traffic simulation.
//!
//! This module manages the simulation session, grid storage, vehicle and trip management, traffic lights, and the step-by-step execution of the simulation pipeline.
//! It is the main entry point for running, controlling, and inspecting the simulation state.
//!
//! ## Purpose
//!
//! - Manages the entire simulation lifecycle and state
//! - Handles vehicle generation, movement, intentions, conflicts, and traffic lights
//! - Provides API for adding trips, vehicles, cells, traffic lights, and conflict zones
//! - Collects and returns simulation state after each step
//!
//! ## Simulation pipeline
//!
//! The simulation module executes the following pipeline in [`Session::step()`](crate::simulation::session::Session::step):
//! ```text
//! 1. Generate vehicles (trips)
//! 2. Update positions
//! 3. Traffic light updates
//! 4. Prepare intentions      ← intentions module
//! 5. Collect conflicts       ← conflicts module
//! 6. Solve conflicts         ← conflicts module
//! 7. Execute movement        ← movement module
//! 8. Collect state dump
//! ```
//!
//! ## Components
//!
//! - [`grids_storage::GridsStorage`] – Stores the vehicle grid and traffic lights
//! - [`session::Session`] – Main simulation controller, manages vehicles, trips, and simulation steps
//! - [`states::AutomataState`] – Snapshot of the simulation state at each step
//! - [`states::VehicleState`] – State of each vehicle at a given timestamp
//! - [`states::TrafficLightGroupState`] – State of each traffic light group at a given timestamp
//! - [`session::SessionError`] – Unified error type for all simulation operations
//!
//! ## Usage
//!
//! Create a session, add trips, vehicles, cells, and traffic lights, then run the simulation step-by-step:
//! ```rust
//! use micro_traffic_sim_core::grid::road_network::GridRoads;
//! use micro_traffic_sim_core::agents::VehicleRef;
//! use micro_traffic_sim_core::trips::trip::Trip;
//! use micro_traffic_sim_core::traffic_lights::lights::{TrafficLight, TrafficLightID};
//! use micro_traffic_sim_core::simulation::session::Session;
//! use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
//! use micro_traffic_sim_core::verbose::VerboseLevel;
//! use std::collections::HashMap;
//!
//! let mut grid = GridRoads::new();
//! // Populate grid with cells
//! // ...
//! let vehicles: Vec<VehicleRef> = vec![/* ... vehicles ... */];
//! // Prepare vehicles or use generator
//! // ...
//! let trips: Vec<Trip> = vec![/* ... trips ... */];
//! // Prepare trips
//! // ...
//! let tls: HashMap<TrafficLightID, TrafficLight> = HashMap::new();
//! // Prepare traffic lights
//! // ...
//! let grids_storage = GridsStorage::new()
//!     .with_vehicles_net(grid)
//!     .with_tls(tls)
//!     .build();
//! let mut session = Session::new(grids_storage, None);
//! session.set_verbose_level(VerboseLevel::Main);
//! session.add_vehicles(vehicles);
//! for trip in trips.iter() {
//!     session.add_trip(trip.clone());
//! }
//! let steps = 10;
//! for step in 0..steps {
//!     match session.step() {
//!          Ok(automata_state) => {
//!             for v in automata_state.vehicles {
//!                 println!(
//!                     "{};{};{};{};{:.5};{}",
//!                     step,
//!                     v.id,
//!                     v.vehicle_type,
//!                     v.last_speed,
//!                     v.last_angle,
//!                     v.last_cell,
//!                 );
//!             }
//!          }
//!          Err(e) => {
//!            eprintln!("Error during simulation step {}: {:?}", step, e);
//!          }
//!     };
//! }
//! ```
//!
//! ## Error handling
//!
//! All errors from grid storage, intentions, conflicts, movement, and traffic lights are unified under [`session::SessionError`] for easier debugging.
//!
//! ## Integration
//!
//! The simulation module is the main entry point for running and controlling the traffic simulation. All other modules (intentions, conflicts, movement, traffic lights, trips) are coordinated here.
pub mod grids_storage;
pub mod session;
pub mod states;