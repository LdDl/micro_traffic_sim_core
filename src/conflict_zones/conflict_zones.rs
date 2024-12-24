use std::fmt;

/// Identifier for a conflict zone.
pub type ConflictZoneID = i32;

/// Types of conflict zones.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConflictZoneType {
    Undefined,
}

/// Types of conflict winners.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConflictWinnerType {
    Undefined,
    Equal,
    First,
    Second,
}

/// Represents an edge in a conflict zone.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConflictEdge {
    pub source: i32,
    pub target: i32,
}

/// Error types for conflict zones.
#[derive(Debug, Clone)]
pub enum ConflictZoneError {
    ZoneNotFound,
    BadZoneData,
}

impl fmt::Display for ConflictZoneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConflictZoneError::ZoneNotFound => write!(f, "zone is not found"),
            ConflictZoneError::BadZoneData => write!(f, "incorrect data for zone"),
        }
    }
}

impl std::error::Error for ConflictZoneError {}

/// Represents a conflict zone in the traffic simulation system.
///
/// A `ConflictZone` models an area where paths intersect, with properties like type, winner, and involved edges.
#[derive(Debug)]
pub struct ConflictZone {
    id: ConflictZoneID,
    zone_type: ConflictZoneType,
    winner_type: ConflictWinnerType,
    first_edge: ConflictEdge,
    second_edge: ConflictEdge,
}

impl ConflictZone {
    /// Constructs a new `ConflictZoneBuilder` for building a `ConflictZone` object.
    ///
    /// # Arguments
    /// * `id` - A unique identifier for the conflict zone.
    /// * `first_edge` - The first edge involved in the conflict.
    /// * `second_edge` - The second edge involved in the conflict.
    ///
    /// # Returns
    /// A `ConflictZoneBuilder` struct which is used to configure and build the `ConflictZone` object.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::conflict_zones::{ConflictEdge, ConflictZone, ConflictWinnerType};
    /// let first_edge = ConflictEdge { source: 1, target: 2 };
    /// let second_edge = ConflictEdge { source: 3, target: 4 };
    /// let builder = ConflictZone::new(1, first_edge, second_edge)
    ///     .with_winner_type(ConflictWinnerType::Second)
    ///     .build();
    /// ```
    pub fn new(id: ConflictZoneID, first_edge: ConflictEdge, second_edge: ConflictEdge) -> ConflictZoneBuilder {
        ConflictZoneBuilder {
            conflict_zone: ConflictZone {
                id,
                zone_type: ConflictZoneType::Undefined,  // Default: Undefined
                winner_type: ConflictWinnerType::Equal,  // Default: Equal
                first_edge,
                second_edge,
            },
        }
    }

    /// Returns the unique identifier (ID) of the conflict zone.
    pub fn get_id(&self) -> ConflictZoneID {
        self.id
    }

    /// Returns the type of the conflict zone.
    pub fn get_zone_type(&self) -> ConflictZoneType {
        self.zone_type
    }

    /// Returns the winner type of the conflict zone.
    pub fn get_winner_type(&self) -> ConflictWinnerType {
        self.winner_type
    }

    /// Returns the first edge of the conflict zone.
    pub fn get_first_edge(&self) -> &ConflictEdge {
        &self.first_edge
    }

    /// Returns the second edge of the conflict zone.
    pub fn get_second_edge(&self) -> &ConflictEdge {
        &self.second_edge
    }
}

/// A builder pattern implementation for constructing `ConflictZone` objects.
///
/// This builder allows for optional configuration of `ConflictZone` fields before building the final `ConflictZone` object.
pub struct ConflictZoneBuilder {
    conflict_zone: ConflictZone,
}

impl ConflictZoneBuilder {
    /// Sets the type of the conflict zone.
    ///
    /// # Arguments
    /// * `zone_type` - The `ConflictZoneType` to assign to the conflict zone.
    ///
    /// # Returns
    /// A `ConflictZoneBuilder` instance for further method chaining.
    pub fn with_zone_type(mut self, zone_type: ConflictZoneType) -> Self {
        self.conflict_zone.zone_type = zone_type;
        self
    }

    /// Sets the winner type for the conflict zone.
    ///
    /// # Arguments
    /// * `winner_type` - The `ConflictWinnerType` to assign to the conflict zone.
    ///
    /// # Returns
    /// A `ConflictZoneBuilder` instance for further method chaining.
    pub fn with_winner_type(mut self, winner_type: ConflictWinnerType) -> Self {
        self.conflict_zone.winner_type = winner_type;
        self
    }

    /// Builds the final `ConflictZone` object.
    ///
    /// # Returns
    /// The fully constructed `ConflictZone` object.
    pub fn build(self) -> ConflictZone {
        self.conflict_zone
    }
}