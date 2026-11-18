//! Forward-path fallback when destination routing fails.

mod intention_no_route;

pub use self::intention_no_route::{NoRouteError, process_no_route_found};
