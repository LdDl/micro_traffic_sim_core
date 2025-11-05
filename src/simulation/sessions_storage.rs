use std::collections::HashMap;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use uuid::Uuid;

use crate::verbose::{VerboseLevel, EVENT_SESSION_CREATE, EVENT_SESSION_EXPIRED};
use super::session::Session;

const DEFAULT_EXP_SECS: u64 = 300; // 5 minutes

pub struct SessionsStorage {
    store: HashMap<Uuid, Session>,
    default_exp_duration: Duration,
    verbose: VerboseLevel,
    // last cleanup timestamp to throttle purge calls if you want
    last_purge_at: i64,
    purge_every: Duration,
}

impl SessionsStorage {
    pub fn new() -> Self {
        Self {
            store: HashMap::new(),
            default_exp_duration: Duration::from_secs(DEFAULT_EXP_SECS),
            verbose: VerboseLevel::None,
            last_purge_at: 0,
            purge_every: Duration::from_secs(30),
        }
    }

    pub fn with_session_exp_time(mut self, d: Duration) -> Self {
        self.default_exp_duration = d;
        self
    }

    pub fn with_storage_verbose(mut self, v: VerboseLevel) -> Self {
        self.verbose = v;
        self
    }

    pub fn with_purge_every(mut self, d: Duration) -> Self {
        self.purge_every = d;
        self
    }

    pub fn sessions_num(&self) -> usize {
        self.store.len()
    }

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
                if self.verbose.is_at_least(VerboseLevel::Main) {
                    self.verbose.log_with_fields(
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

    // Mutably operate on a session without cloning it
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

    pub fn purge_expired(&mut self) {
        let now = now_ns();
        self.store.retain(|sid, s| {
            let alive = s.get_expire_at() >= now;
            if !alive && self.verbose.is_at_least(VerboseLevel::Main) {
                self.verbose.log_with_fields(
                    EVENT_SESSION_EXPIRED,
                    "Session has been expired",
                    &[("session_id", sid), ("session_exp_at", &s.get_expire_at()), ("now", &now)],
                );
            }
            alive
        });
        self.last_purge_at = now;
    }

    // Optional: throttle purges to every purge_every
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