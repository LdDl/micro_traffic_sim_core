use super::*;
use std::thread::sleep;

// A custom session TTL overrides the storage default, access renews it, and a sufficiently
// long idle interval makes the session inaccessible.
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

    // Another ~3s. Access should extend the expiration based on time since last update
    sleep(Duration::from_secs(3));
    let exists = storage.with_session_mut(&object_id, |_s| {}).is_some();
    assert!(
        exists,
        "session should still exist after second sleep (~6s total) due to extension"
    );

    // Sleep long enough to surpass the extended expiration and confirm it is gone
    sleep(Duration::from_secs(4));
    let exists = storage.with_session_mut(&object_id, |_s| {}).is_some();
    assert!(!exists, "session should be expired after final sleep");
}

// Removal returns the stored session, updates the count, and makes subsequent removal and
// access return None.
#[test]
fn test_remove_session() {
    let mut storage = SessionsStorage::new();
    let session = crate::simulation::session::Session::default(None);
    let id = session.get_id();
    assert!(storage.register_session(id, session, None));
    assert_eq!(storage.sessions_num(), 1);

    // Explicit removal hands back the owned session and drops it from the store.
    let removed = storage.remove_session(&id);
    assert!(removed.is_some(), "remove should return the session");
    assert_eq!(removed.unwrap().get_id(), id);
    assert_eq!(storage.sessions_num(), 0);

    // Removing again is a no-op.
    assert!(storage.remove_session(&id).is_none());

    // After removal the session is no longer accessible.
    assert!(storage.with_session_mut(&id, |_s| {}).is_none());
}
