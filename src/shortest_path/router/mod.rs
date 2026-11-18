//! A* routing, cached-route reconnection, and forward path traversal.

mod router;

pub use self::router::{
    AStarError, AStarNode, path_no_goal, reconnect_to_cache, shortest_path,
    shortest_path_with_heuristic,
};
