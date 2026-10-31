/// Longitudinal reserve projection applied after conflict resolution.
///
/// This session-wide structural choice uses gaps in cells and displacements in
/// cells per tick. It does not change driver preferences or noise probabilities.
/// Both variants enforce Vehicle::standstill_gap by shortening already granted
/// paths. A path stops before another vehicle's initial body, and neither policy
/// extends it. Leader motion can preserve an extra empty reserve, never authorize
/// entry into another vehicle's initially occupied cell.
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
    /// is bounded by its original grant and max(d + lower - g, 0). The grant itself
    /// is at most d, so the initial leader cell remains unreachable this tick.
    /// This project-specific extension has separate experimental validation;
    /// enabling it does not certify a physical traffic phase or network domain.
    MovingLeader,
}
