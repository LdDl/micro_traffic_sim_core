/// Longitudinal reserve projection applied after conflict resolution.
///
/// This session-wide structural choice uses gaps in cells and displacements in
/// cells per tick. It does not change driver preferences or noise probabilities.
/// Both variants shorten already granted paths using the initial physical bodies.
/// Reserve lookahead follows the forward continuation and ends at a planned exit
/// when the cached route matches the complete granted path. An unrelated queue
/// beyond that exit does not prevent approaching the turn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LongitudinalClearance {
    /// Preserve the existing conservative reserve contract.
    /// A follower with initial empty gap d and reserve g may move at most
    /// max(d - g, 0) cells, irrespective of the leader's granted motion.
    #[default]
    Conservative,
    /// Credit a frozen guaranteed displacement of an already moving leader.
    ///
    /// The leader must have a verified forward path and a one-cell body. Its
    /// conservative projected displacement supplies a lower bound. A stopped
    /// leader supplies zero even if it starts during this tick. Follower motion
    /// is bounded by its original grant and max(d + lower - g, 0).
    /// This project-specific extension has separate experimental validation;
    /// enabling it does not certify a physical traffic phase or network domain.
    MovingLeader,
}
