use crate::agents::Vehicle;
use crate::grid::cell::CellID;
use crate::intentions::{CellIntention, IntentionType};
use std::collections::HashMap;

/// Storage for cell intentions, mapping CellID to a vector of intentions and storing vehicle intentions.
#[derive(Debug, Default)]
pub struct Intentions<'a> {
    cells_intentions: HashMap<CellID, Vec<CellIntention<'a>>>,
}

impl<'a> Intentions<'a> {
    /// Creates a new empty intentions storage.
    ///
    /// Returns an empty `Intentions` struct with initialized HashMap for storing cell intentions.
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::intentions::{Intentions};
    /// let intentions = Intentions::new();
    /// ```
    pub fn new() -> Self {
        Self {
            cells_intentions: HashMap::new(),
        }
    }

    /// Returns number of intentions in the storage.
    ///
    /// # Returns
    /// Number of intentions in the storage
    ///
    /// # Examples
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, VehicleIntention, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let mut vehicle1 = Vehicle::new(1)
    ///    .with_type(AgentType::Car)
    ///    .build();
    /// let mut intention1 = VehicleIntention::default();
    /// intention1.intention_cell_id = 15;
    /// vehicle1.set_intention(intention1);
    /// let mut vehicle2 = Vehicle::new(2)
    ///    .with_type(AgentType::Car)
    ///    .build();
    /// let mut intention2 = VehicleIntention::default();
    /// intention2.intention_cell_id = 16;
    /// vehicle2.set_intention(intention2);
    /// let mut intentions = Intentions::new();
    /// intentions.add_intention(&mut vehicle1, IntentionType::Target);
    /// intentions.add_intention(&mut vehicle2, IntentionType::Target);
    /// println!("Number of intentions: {}", intentions.len());
    /// ```
    pub fn len(&self) -> usize {
        self.cells_intentions.len()
    }

    /// Returns true if the storage is empty.
    ///
    /// # Returns
    /// True if the storage is empty, false otherwise
    ///
    /// # Examples
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, VehicleIntention, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let mut intentions = Intentions::new();
    /// println!("Intentions storage is empty: {}", intentions.is_empty());
    /// ```
    pub fn is_empty(&self) -> bool {
        self.cells_intentions.is_empty()
    }

    /// Returns iterable reference to the intentions storage.
    ///
    /// # Returns
    /// Iterable reference to the intentions storage
    ///
    /// # Examples
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, VehicleIntention, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let mut vehicle1 = Vehicle::new(1)
    ///   .with_type(AgentType::Car)
    ///   .build();
    /// let mut intention1 = VehicleIntention::default();
    /// intention1.intention_cell_id = 15;
    /// vehicle1.set_intention(intention1);
    /// let mut vehicle2 = Vehicle::new(2)
    ///   .with_type(AgentType::Car)
    ///   .build();
    /// let mut intention2 = VehicleIntention::default();
    /// intention2.intention_cell_id = 16;
    /// vehicle2.set_intention(intention2);
    /// let mut intentions = Intentions::new();
    /// intentions.add_intention(&mut vehicle1, IntentionType::Target);
    /// intentions.add_intention(&mut vehicle2, IntentionType::Target);
    /// for (cell_id, ints) in intentions.iter() {
    ///     println!("Cell {} has {} intentions", cell_id, ints.len());
    /// }
    /// ```
    pub fn iter(&self) -> std::collections::hash_map::Iter<CellID, Vec<CellIntention<'a>>> {
        self.cells_intentions.iter()
    }

    /// Returns cell intentions for the given cell ID.
    ///
    /// # Parameters
    /// * `cell_id` - The ID of the cell to get intentions for
    ///
    /// # Returns
    /// Returns a reference to the vector of intentions for the given cell ID, or None if the cell ID is not found.
    ///
    /// # Examples
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, VehicleIntention, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let mut vehicle1 = Vehicle::new(1)
    ///   .with_type(AgentType::Car)
    ///   .build();
    /// let mut intention1 = VehicleIntention::default();
    /// intention1.intention_cell_id = 15;
    /// vehicle1.set_intention(intention1);
    /// let mut vehicle2 = Vehicle::new(2)
    ///   .with_type(AgentType::Car)
    ///   .build();
    /// let mut intention2 = VehicleIntention::default();
    /// intention2.intention_cell_id = 16;
    /// vehicle2.set_intention(intention2);
    /// let mut intentions = Intentions::new();
    /// intentions.add_intention(&mut vehicle1, IntentionType::Target);
    /// intentions.add_intention(&mut vehicle2, IntentionType::Target);
    /// let cell_intentions = intentions.get(&15);
    /// println!("Cell 15 has {} intentions", cell_intentions.unwrap().len());
    /// ```
    pub fn get(&self, cell_id: &CellID) -> Option<&Vec<CellIntention<'a>>> {
        self.cells_intentions.get(&cell_id)
    }

    /// Private method for pushing a new intention to the storage for the given cell ID.
    ///
    /// This method is a shortcut for adding intentions to the storage. It will create
    /// a new vector for the cell ID if it doesn't exist, or append to existing one.
    ///
    /// # Parameters
    /// * `cell_id` - The ID of the cell to add intention to
    /// * `vehicle` - Reference to the vehicle with intention
    /// * `intention_type` - Type of the intention to add
    /// ```
    fn push_intention(
        &mut self,
        cell_id: CellID,
        vehicle: &'a Vehicle,
        intention_type: IntentionType,
    ) {
        self.cells_intentions
            .entry(cell_id)
            .or_insert_with(Vec::new)
            .push(CellIntention::new(Some(vehicle), intention_type));
    }

    /// Adds a new intention to the storage for the given cell ID.
    ///
    /// This method is a shortcut for adding intentions to the storage. It will create
    /// a new vector for the cell ID if it doesn't exist, or append to existing one.
    ///
    /// # Parameters
    /// * `cell_id` - The ID of the cell to add intention to
    /// * `vehicle` - Reference to the vehicle with intention
    /// * `intention_type` - Type of the intention to add
    ///
    /// # Returns
    /// Returns mutable reference to self for method chaining
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, VehicleIntention, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let mut vehicle1 = Vehicle::new(1)
    ///     .with_type(AgentType::Car)
    ///     .with_cell(10)
    ///     .build();
    /// let mut intention1 = VehicleIntention::default();
    /// intention1.intention_cell_id = 15;
    /// vehicle1.set_intention(intention1);
    /// let mut vehicle2 = Vehicle::new(2)
    ///     .with_type(AgentType::Car)
    ///     .with_cell(20)
    ///     .build();
    /// let mut intention2 = VehicleIntention::default();
    /// intention2.intention_cell_id = 15;
    /// vehicle2.set_intention(intention2);
    /// let mut intentions = Intentions::new();
    /// intentions.add_intention(&mut vehicle1, IntentionType::Target);
    /// intentions.add_intention(&mut vehicle2, IntentionType::Target);
    /// ```
    pub fn add_intention(&mut self, vehicle: &'a mut Vehicle, intention_type: IntentionType) {
        let extracted_tail = Self::extract_tail_intention(vehicle);
        vehicle.intention.tail_intention_cells = extracted_tail.clone();
        for i in vehicle.intention.intermediate_cells.clone() {
            // Clone because of borrow checker
            // Just doing transit ("jump")
            self.push_intention(i, vehicle, IntentionType::Transit);
        }
        for int_occupied_cell_id in extracted_tail.iter() {
            if *int_occupied_cell_id < 1 {
                continue;
            }
            self.push_intention(*int_occupied_cell_id, vehicle, IntentionType::Tail);
        }
        self.push_intention(vehicle.intention.intention_cell_id, vehicle, intention_type);
    }

    fn extract_tail_intention(vehicle: &Vehicle) -> Vec<CellID> {
        let tail_size = vehicle.tail_cells.len();
        if tail_size == 0 {
            return vec![];
        }
        let mut intention_tail = vec![-1; tail_size];
        // Vehicle does not intend to move
        if vehicle.intention.intention_cell_id == vehicle.cell_id {
            for i in 0..tail_size - 1 {
                intention_tail[i] = vehicle.tail_cells[i + 1];
            }
            intention_tail[tail_size - 1] = vehicle.cell_id;
            return intention_tail;
        }

        let skip_size = vehicle.intention.intermediate_cells.len();
        match skip_size {
            // Moving with acceleration, but it is not greater than vehicle's size
            skip_size if skip_size > 0 && skip_size < tail_size => {
                // If there are fewer intermediate cells than the vehicle's size, offset vehicle tail
                intention_tail[0..(tail_size - skip_size - 1)]
                    .copy_from_slice(&vehicle.tail_cells[(skip_size + 1)..]);
                intention_tail[tail_size - skip_size - 1] = vehicle.cell_id;
                intention_tail[(tail_size - skip_size)..]
                    .copy_from_slice(&vehicle.intention.intermediate_cells);
            }
            // Moving with acceleration and it is greater or equal to vehicle's size
            skip_size if skip_size > 0 && skip_size >= tail_size => {
                // If there are more intermediate cells than the vehicle's size,
                // return the last N intermediate cells
                intention_tail.copy_from_slice(
                    &vehicle.intention.intermediate_cells[skip_size - tail_size..],
                );
            }
            // Otherwise
            _ => {
                for i in 0..tail_size - 1 {
                    intention_tail[i] = vehicle.tail_cells[i + 1];
                }
                intention_tail[tail_size - 1] = vehicle.cell_id;
            }
        }
        intention_tail
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::{AgentType, VehicleIntention};
    #[test]
    fn test_add_to_empty_intentions() {
        let mut intentions = Intentions::new();
        let mut vehicle = Vehicle::new(1).with_type(AgentType::Car).build();
        let intention = VehicleIntention {
            intention_cell_id: 1,
            ..Default::default()
        };
        vehicle.set_intention(intention);
        intentions.add_intention(&mut vehicle, IntentionType::Target);

        // Validate
        assert_eq!(intentions.cells_intentions.len(), 1);
        assert_eq!(intentions.cells_intentions.get(&1).unwrap().len(), 1);
        assert_eq!(
            intentions.cells_intentions.get(&1).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.cells_intentions.get(&1).unwrap()[0].int_type,
            IntentionType::Target
        );
    }

    #[test]
    fn test_add_multiple_to_same_cell() {
        let mut intentions = Intentions::new();

        // Add first vehicle
        let mut vehicle1 = Vehicle::new(1).with_type(AgentType::Car).build();
        let intention1 = VehicleIntention {
            intention_cell_id: 1,
            ..Default::default()
        };
        vehicle1.set_intention(intention1);
        intentions.add_intention(&mut vehicle1, IntentionType::Target);

        // Add second vehicle
        let mut vehicle2 = Vehicle::new(2).with_type(AgentType::Car).build();
        let intention2 = VehicleIntention {
            intention_cell_id: 1,
            ..Default::default()
        };
        vehicle2.set_intention(intention2);
        intentions.add_intention(&mut vehicle2, IntentionType::Transit);

        // Validate
        assert_eq!(intentions.cells_intentions.len(), 1);
        assert_eq!(intentions.cells_intentions.get(&1).unwrap().len(), 2);
        assert_eq!(
            intentions.cells_intentions.get(&1).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.cells_intentions.get(&1).unwrap()[0].int_type,
            IntentionType::Target
        );
        assert_eq!(
            intentions.cells_intentions.get(&1).unwrap()[1]
                .get_vehicle()
                .unwrap()
                .id,
            2
        );
        assert_eq!(
            intentions.cells_intentions.get(&1).unwrap()[1].int_type,
            IntentionType::Transit
        );
    }

    #[test]
    fn test_add_to_different_cells() {
        let mut intentions = Intentions::new();

        // Add to first cell
        let mut vehicle1 = Vehicle::new(1).with_type(AgentType::Car).build();
        let intention1 = VehicleIntention {
            intention_cell_id: 1,
            ..Default::default()
        };
        vehicle1.set_intention(intention1);
        intentions.add_intention(&mut vehicle1, IntentionType::Target);

        // Add to second cell
        let mut vehicle2 = Vehicle::new(2).with_type(AgentType::Car).build();
        let intention2 = VehicleIntention {
            intention_cell_id: 2,
            ..Default::default()
        };
        vehicle2.set_intention(intention2);
        intentions.add_intention(&mut vehicle2, IntentionType::Target);

        // Validate
        assert_eq!(intentions.cells_intentions.len(), 2);
        assert_eq!(intentions.cells_intentions.get(&1).unwrap().len(), 1);
        assert_eq!(intentions.cells_intentions.get(&2).unwrap().len(), 1);
        assert_eq!(
            intentions.cells_intentions.get(&1).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.cells_intentions.get(&1).unwrap()[0].int_type,
            IntentionType::Target
        );
        assert_eq!(
            intentions.cells_intentions.get(&2).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            2
        );
        assert_eq!(
            intentions.cells_intentions.get(&2).unwrap()[0].int_type,
            IntentionType::Target
        );
    }

    #[test]
    fn test_add_multiple_to_same_cell_tail() {
        let mut intentions = Intentions::new();

        // Setup first vehicle with tail
        let mut vehicle1 = Vehicle::new(1)
            .with_cell(10)
            .with_tail_size(2, vec![8, 9])
            .build();
        let intention1 = VehicleIntention {
            intention_cell_id: 11,
            ..Default::default()
        };
        vehicle1.set_intention(intention1);
        intentions.add_intention(&mut vehicle1, IntentionType::Target);

        // Add second vehicle
        let mut vehicle2 = Vehicle::new(2).with_type(AgentType::Car).build();
        let intention2 = VehicleIntention {
            intention_cell_id: 10,
            ..Default::default()
        };
        vehicle2.set_intention(intention2);
        intentions.add_intention(&mut vehicle2, IntentionType::Target);

        // Validate
        assert_eq!(intentions.cells_intentions.len(), 3); // Cells 11, 10, 9
        assert!(!intentions.cells_intentions.contains_key(&8)); // Cell 8 should not be included
        assert_eq!(intentions.cells_intentions.get(&9).unwrap().len(), 1); // Tail end
        assert_eq!(intentions.cells_intentions.get(&10).unwrap().len(), 2); // Tail start + vehicle 2
        assert_eq!(intentions.cells_intentions.get(&11).unwrap().len(), 1); // Vehicle 1 head

        // Check vehicle 1 tail and intentions
        assert_eq!(
            intentions.cells_intentions.get(&9).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.cells_intentions.get(&9).unwrap()[0].int_type,
            IntentionType::Tail
        );
        assert_eq!(
            intentions.cells_intentions.get(&10).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.cells_intentions.get(&10).unwrap()[0].int_type,
            IntentionType::Tail
        );

        // Check vehicle 2
        assert_eq!(
            intentions.cells_intentions.get(&10).unwrap()[1]
                .get_vehicle()
                .unwrap()
                .id,
            2
        );
        assert_eq!(
            intentions.cells_intentions.get(&10).unwrap()[1].int_type,
            IntentionType::Target
        );

        // Check vehicle 1 head
        assert_eq!(
            intentions.cells_intentions.get(&11).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.cells_intentions.get(&11).unwrap()[0].int_type,
            IntentionType::Target
        );
    }
}
