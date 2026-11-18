/// Randomization relative to the leader and the stop time threshold: the three phase layer of
/// the longitudinal rule.
///
/// Uses the stop-time threshold and speed-difference-dependent randomization of Gao, Jiang, Hu, Wang and Wu (2007), doi:10.1103/PhysRevE.76.026105, eqs (1) to (4), with the following modifications:
///
/// - Update order: the core accelerates, applies random slowdown, then clamps the candidate to the available path. Gao brakes to the gap before randomization. These orders can produce different speeds when the gap is binding.
/// - Interaction range: the nearest leader is probed along the forward chain within `lookahead` cells (15 cells, 67.5 m, in `KKW`). Without a leader in that range, the core uses `decel_equal`. Gao's original rule uses the preceding vehicle without this finite cutoff.
/// - Calibration: `KKW` selects a stop threshold of 5 ticks and decelerations 0, 1, 1 on the project's 4.5 m grid. In particular, `decel_faster = 1` omits the stronger random deceleration tested in the project's over-deceleration experiments. These values are project choices, not a reproduction of Gao's published parameter set.
/// - Network integration: the speed comparison uses the follower's current speed clipped to its local speed limit and the leader's start-of-tick snapshot. Acceleration and slowdown timers, claimed cells, conflict resolution and the separate standstill-gap rule also affect the executed movement.
/// - VDR boundary case: `p0` requires both zero speed and a standing count at least equal to the threshold. Setting the threshold to zero therefore selects `p0` only for stopped vehicles and `p` for moving ones, explicitly defining the VDR reduction.
///
/// The separate standstill-gap calibration gives an average jam spacing of about 7.5 m when two thirds of one-cell cars keep one empty cell. This does not change the 4.5 m cell size. The calibrated `p0` and driver-style `p` are supplied by behaviour parameters or trip overrides, not by this struct.
///
/// With threshold 0 and unit decelerations, the longitudinal randomization reduces to the core's VDR order. This does not assert equivalence to textbook NaSch's update order, Gao's complete model or the full network behaviour of an older core version.
///
/// Project benches reproduce ring hysteresis, low-speed flow near a bottleneck and wide moving jams emerging predominantly in that region under the operational phase classifier. These are measurements of this variant; they do not establish all properties of Gao's model or a persistent two-dimensional region of synchronized states.
///
/// Used while synchronization is disabled. An enabled SynchronizationParameters
/// configuration supplies its own stochastic correction and restart threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RelativeRandomization {
    /// Consecutive ticks of standing after which the slow to start probability `p0` applies; a
    /// shorter stop randomizes with the moving probability `p` (t_c of Gao 2007). 0 applies `p0`
    /// to every stopped vehicle (VDR).
    pub stop_time_threshold: i32,
    /// Deceleration in cells per tick of the random slowdown when the vehicle is slower than its
    /// leader (b_- of Gao 2007). 0 means no slowdown while catching up.
    pub decel_slower: i32,
    /// Deceleration when the vehicle is as fast as its leader or has no leader within
    /// `lookahead` cells (b_0). 1 is the NaSch dawdle.
    pub decel_equal: i32,
    /// Deceleration when the vehicle is faster than its leader (b_+, b_+ >= b_-).
    pub decel_faster: i32,
    /// Cells ahead searched for the leader whose speed selects the deceleration; 0 skips the
    /// search and uses `decel_equal`.
    pub lookahead: i32,
}

impl RelativeRandomization {
    /// The plain NaSch and VDR rule: `p0` for every stopped vehicle, deceleration 1, no leader
    /// search needed for the speed result. This reduces the longitudinal randomization to VDR;
    /// it does not imply equivalence of the entire network to older core versions.
    pub const VDR: RelativeRandomization = RelativeRandomization {
        stop_time_threshold: 0,
        decel_slower: 1,
        decel_equal: 1,
        decel_faster: 1,
        lookahead: 15,
    };
    /// The three phase values of the behaviour presets: no slowdown while catching up with a
    /// faster leader, slow to start after 5 ticks of standing, leader search over 15 cells (67 m).
    pub const KKW: RelativeRandomization = RelativeRandomization {
        stop_time_threshold: 5,
        decel_slower: 0,
        decel_equal: 1,
        decel_faster: 1,
        lookahead: 15,
    };

    /// Clamps every field to its admissible range (non negative, decelerations at most the
    /// maximum speed are not enforced, the executed speed is clamped by the rule itself).
    pub fn clamped(self) -> Self {
        RelativeRandomization {
            stop_time_threshold: self.stop_time_threshold.max(0),
            decel_slower: self.decel_slower.max(0),
            decel_equal: self.decel_equal.max(0),
            decel_faster: self.decel_faster.max(0),
            lookahead: self.lookahead.max(0),
        }
    }
}

impl Default for RelativeRandomization {
    fn default() -> Self {
        RelativeRandomization::VDR
    }
}
