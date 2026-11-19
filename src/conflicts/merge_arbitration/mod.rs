//! Merge-zone priority arbitration and intended-edge matching.

mod merge_arbitration;

pub use self::merge_arbitration::{MergeDecision, arbitrate, intention_traverses_edge};
