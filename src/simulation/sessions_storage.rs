//! Optional sessions management for server-side scenarios.
//!
//! This module provides a lightweight, in-memory storage for multiple
//! simulation [`session::Session`](crate::simulation::session::Session) objects keyed by `Uuid`.
//! Each session has a time-to-live (TTL) that is extended on access; expired
//! sessions are dropped. No background threads are used; purging is performed
//! on demand and throttled by a configurable interval.
//!
//! Key features:
//! - Manage many sessions by ID, no cloning (mutate in place via closures)
//! - Time-based expiration with on-access extension
//! - Manual purge with throttling
//! - Structured logging via the crate’s `verbose` API
//!
//! Basic usage:
//! ```rust
//! use micro_traffic_sim_core::simulation::sessions_storage::SessionsStorage;
//! use micro_traffic_sim_core::simulation::session::Session;
//! use micro_traffic_sim_core::verbose::VerboseLevel;
//! use uuid::Uuid;
//! use std::time::Duration;
//!
//! let mut store = SessionsStorage::new()
//!     .with_session_exp_time(Duration::from_secs(300))  // default TTL per session
//!     .with_purge_every(Duration::from_secs(30))        // purge throttle
//!     .with_storage_verbose(VerboseLevel::Main);        // log create/expire events
//!
//! let session = Session::default(None);
//! let sid = session.get_id();
//! let created = store.register_session(sid, session, None);
//! assert!(created);
//!
//! // Access extends expiration and lets you mutate the session in place
//! let ok = store.with_session_mut(&sid, |sess| {
//!     // e.g., run one step
//!     let _ = sess.get_steps();
//! }).is_some();
//! assert!(ok);
//!
//! // Manual purge (optional). A throttled purge also happens on access.
//! store.purge_expired();
//! ```
//!
//! Thread-safety: wrap the storage in `Arc<Mutex<...>>` (or `RwLock`) when sharing it
//! across threads in a server; this type itself is not `Send`/`Sync`.
//!
use std::collections::HashMap;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use uuid::Uuid;

use crate::verbose::{VerboseLevel, LocalLogger, EVENT_SESSION_CREATE, EVENT_SESSION_EXPIRED};
use super::session::Session;

const DEFAULT_EXP_SECS: u64 = 300; // 5 minutes

/// In-memory storage for multiple [`Session`]s with TTL-based expiration and
/// on-access extension. Intended for optional server-side use.
pub struct SessionsStorage {
    store: HashMap<Uuid, Session>,
    default_exp_duration: Duration,
    logger: LocalLogger,
    // last cleanup timestamp to throttle purge calls if you want
    last_purge_at: i64,
    purge_every: Duration,
}

impl SessionsStorage {
    /// Creates a new storage with sane defaults:
    /// - default expiration: 300s
    /// - purge throttle: 30s
    /// - storage logging: `VerboseLevel::None`
    pub fn new() -> Self {
        Self {
            store: HashMap::new(),
            default_exp_duration: Duration::from_secs(DEFAULT_EXP_SECS),
            logger: LocalLogger::with_session(VerboseLevel::None, "sessions_storage".to_string()),
            last_purge_at: 0,
            purge_every: Duration::from_secs(30),
        }
    }

    /// Sets the default expiration (TTL) applied when registering a session
    /// without an explicit per-session duration.
    pub fn with_session_exp_time(mut self, d: Duration) -> Self {
        self.default_exp_duration = d;
        self
    }

    /// Sets the storage’s own logger level (not per-session). Used to emit
    /// `session_create` and `session_expired` events through the verbose API.
    pub fn with_storage_verbose(mut self, v: VerboseLevel) -> Self {
        self.logger.set_level(v);
        self
    }

    /// Sets the minimum interval between automatic purge runs triggered on
    /// access. Larger values reduce purge frequency, smaller values purge more
    /// aggressively.
    pub fn with_purge_every(mut self, d: Duration) -> Self {
        self.purge_every = d;
        self
    }

    /// Returns the number of sessions currently stored. Note that this may
    /// include items that will expire on next access if purge hasn’t run yet.
    pub fn sessions_num(&self) -> usize {
        self.store.len()
    }

    /// Registers a session under a specific `Uuid`.
    ///
    /// - If the ID is unused, inserts the session and returns `true`.
    /// - If an expired session exists under the same ID, replaces it and returns `true`.
    /// - If a live session already exists, does nothing and returns `false`.
    ///
    /// You can override the default expiration by passing `Some(duration)`;
    /// otherwise the storage-wide default is used.
    pub fn register_session(
        &mut self,
        session_id: Uuid,
        mut session: Session,
        exp: Option<Duration>,
    ) -> bool {
        let exp_dur = exp.unwrap_or(self.default_exp_duration);
        let exp_at = now_ns() + exp_dur.as_nanos() as i64;

        match self.store.get(&session_id) {
            None => {
                if self.logger.is_at_least(VerboseLevel::Main) {
                    self.logger.log_with_fields(
                        EVENT_SESSION_CREATE,
                        "Session has been created",
                        &[("session_id", &session_id), ("session_exp_at", &exp_at)],
                    );
                }
                session.set_expire_at(exp_at);
                self.store.insert(session_id, session);
                true
            }
            Some(existing) => {
                // If existing is expired, replace it
                if existing.get_expire_at() < now_ns() {
                    let mut new_sess = session;
                    new_sess.set_expire_at(exp_at);
                    self.store.insert(session_id, new_sess);
                    true
                } else {
                    false
                }
            }
        }
    }

    /// Retrieves a mutable handle to a live session and applies the provided
    /// closure to it. Extends the session’s expiration by the time elapsed
    /// since the last update. Returns `None` if the session doesn’t exist or
    /// has expired.
    ///
    /// Also performs a throttled purge on access.
    pub fn with_session_mut<R, F: FnOnce(&mut Session) -> R>(
        &mut self,
        session_id: &Uuid,
        f: F,
    ) -> Option<R> {
        self.extend_exp_and_cleanup();
        if let Some(sess) = self.store.get_mut(session_id) {
            if sess.get_expire_at() < now_ns() {
                return None;
            }
            // Extend expiration based on recent access
            let now = now_ns();
            let delta = now - sess.get_updated_at();
            sess.set_expire_at(sess.get_expire_at() + delta.max(0));
            sess.set_updated_at(now);
            Some(f(sess))
        } else {
            None
        }
    }

    /// Immediately removes all expired sessions and emits a `session_expired`
    /// log event for each one (if storage verbosity allows). This is also
    /// called automatically on access when the throttling interval is reached.
    pub fn purge_expired(&mut self) {
        let now = now_ns();
        self.store.retain(|sid, s| {
            let alive = s.get_expire_at() >= now;
            if !alive && self.logger.is_at_least(VerboseLevel::Main) {
                self.logger.log_with_fields(
                    EVENT_SESSION_EXPIRED,
                    "Session has been expired",
                    &[("session_id", sid), ("session_exp_at", &s.get_expire_at()), ("now", &now)],
                );
            }
            alive
        });
        self.last_purge_at = now;
    }

    /// Optional: throttle purges to every `purge_every` interval.
    fn extend_exp_and_cleanup(&mut self) {
        let now = now_ns();
        if now - self.last_purge_at >= self.purge_every.as_nanos() as i64 {
            self.purge_expired();
        }
    }
}

fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread::sleep;

    #[test]
    fn test_storage_timers() {
        // Create storage with default expiration 2s and purge cadence 5s
        let mut storage = SessionsStorage::new()
            .with_session_exp_time(Duration::from_secs(2))
            .with_purge_every(Duration::from_secs(5))
            .with_storage_verbose(VerboseLevel::None);

        // New session with custom expiration of 4s (overrides default 2s)
        let session = crate::simulation::session::Session::default(None);
        let object_id = session.get_id();
        let registered = storage.register_session(object_id, session, Some(Duration::from_secs(4)));
        assert!(registered, "session should be registered");

        // After ~3s the session should still exist
        sleep(Duration::from_secs(3));
        let exists = storage.with_session_mut(&object_id, |_s| {}).is_some();
        assert!(exists, "session should exist after first sleep (~3s)");

        // Another ~3s; access should extend the expiration based on time since last update
        sleep(Duration::from_secs(3));
        let exists = storage.with_session_mut(&object_id, |_s| {}).is_some();
        assert!(exists, "session should still exist after second sleep (~6s total) due to extension");

        // Sleep long enough to surpass the extended expiration and confirm it is gone
        sleep(Duration::from_secs(4));
        let exists = storage.with_session_mut(&object_id, |_s| {}).is_some();
        assert!(!exists, "session should be expired after final sleep");
    }
}