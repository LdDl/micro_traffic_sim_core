//! Vehicle intention preparation, longitudinal movement, and lane changes.

mod intention;

pub use self::intention::{
    IntentionError, find_alternate_intention, find_intention, prepare_intentions,
};
