use crate::agents::{AgentType, Vehicle};
use crate::grid::cell::CellID;
use crate::intentions::{CellIntention, IntentionType};
use std::collections::HashMap;

/// Storage for cell intentions, mapping CellID to a vector of intentions
#[derive(Debug, Default)]
pub struct Intentions<'a> {
    intentions: HashMap<CellID, Vec<CellIntention<'a>>>,
}

impl<'a> Intentions<'a> {
    /// Creates a new empty intentions storage.
    ///
    /// Returns an empty `Intentions` struct with initialized HashMap for storing cell intentions.
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let vehicle = Vehicle::new(1)
    ///     .with_type(AgentType::Car)
    ///     .build();
    /// let cell_intention = CellIntention::new(Some(&vehicle), IntentionType::Target);
    /// let intentions = Intentions::new();
    /// ```
    pub fn new() -> Self {
        Self {
            intentions: HashMap::new(),
        }
    }

    /// Returns number of intentions in the storage.
    ///
    /// # Returns
    /// Number of intentions in the storage
    ///
    /// # Examples
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let mut vehicle1 = Vehicle::new(1)
    ///    .with_type(AgentType::Car)
    ///    .build();
    /// let mut vehicle2 = Vehicle::new(2)
    ///    .with_type(AgentType::Car)
    ///    .build();
    /// let mut intentions = Intentions::new();
    /// intentions.add_intention(15, &mut vehicle1, IntentionType::Target);
    /// intentions.add_intention(16, &mut vehicle2, IntentionType::Target);
    /// println!("Number of intentions: {}", intentions.len());
    /// ```
    pub fn len(&self) -> usize {
        self.intentions.len()
    }

    /// Returns iterable reference to the intentions storage.
    ///
    /// # Returns
    /// Iterable reference to the intentions storage
    ///
    /// # Examples
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let mut vehicle1 = Vehicle::new(1)
    ///   .with_type(AgentType::Car)
    ///   .build();
    /// let mut vehicle2 = Vehicle::new(2)
    ///   .with_type(AgentType::Car)
    ///   .build();
    /// let mut intentions = Intentions::new();
    /// intentions.add_intention(15, &mut vehicle1, IntentionType::Target);
    /// intentions.add_intention(16, &mut vehicle2, IntentionType::Target);
    /// for (cell_id, ints) in intentions.iter() {
    ///     println!("Cell {} has {} intentions", cell_id, ints.len());
    /// }
    /// ```
    pub fn iter(&self) -> std::collections::hash_map::Iter<CellID, Vec<CellIntention<'a>>> {
        self.intentions.iter()
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
    /// use micro_traffic_sim_core::agents::{Vehicle, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let mut vehicle1 = Vehicle::new(1)
    ///    .with_type(AgentType::Car)
    ///   .build();
    /// let mut vehicle2 = Vehicle::new(2)
    ///   .with_type(AgentType::Car)
    ///   .build();
    /// let mut intentions = Intentions::new();
    /// intentions.add_intention(15, &mut vehicle1, IntentionType::Target);
    /// intentions.add_intention(16, &mut vehicle2, IntentionType::Target);
    /// let cell_intentions = intentions.get(&15);
    /// println!("Cell 15 has {} intentions", cell_intentions.unwrap().len());
    /// ```
    pub fn get(&self, cell_id: &CellID) -> Option<&Vec<CellIntention<'a>>> {
        self.intentions.get(&cell_id)
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
    ) -> &mut Self {
        self.intentions
            .entry(cell_id)
            .or_insert_with(Vec::new)
            .push(CellIntention::new(Some(vehicle), intention_type));
        self
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
    /// use micro_traffic_sim_core::agents::{Vehicle, AgentType};
    /// use micro_traffic_sim_core::intentions::{Intentions, CellIntention, IntentionType};
    /// let mut vehicle1 = Vehicle::new(1)
    ///     .with_type(AgentType::Car)
    ///     .with_cell(10)
    ///     .build();
    /// let mut vehicle2 = Vehicle::new(2)
    ///     .with_type(AgentType::Car)
    ///     .with_cell(20)
    ///     .build();
    /// let mut intentions = Intentions::new();
    /// intentions.add_intention(15, &mut vehicle1, IntentionType::Target);
    /// intentions.add_intention(15, &mut vehicle2, IntentionType::Target);
    /// ```
    pub fn add_intention(
        &mut self,
        cell_id: CellID,
        vehicle: &'a mut Vehicle,
        intention_type: IntentionType,
    ) {
        self.intentions.entry(cell_id).or_insert_with(Vec::new);

        // Just doing transit
        if intention_type == IntentionType::Transit {
            vehicle.intermediate_cells.push(cell_id);
            // @todo: This should be used is Golang's "defer" alternative
            self.push_intention(cell_id, vehicle, intention_type);
            return;
        }
        vehicle.intention_cell_id = cell_id;
        let tail_size = vehicle.tail_cells.len();
        if tail_size > 0 {
            if vehicle.intention_cell_id == vehicle.cell_id {
                for i in 0..tail_size - 1 {
                    vehicle.tail_cells[i] = vehicle.tail_cells[i + 1];
                }
                vehicle.tail_cells[tail_size - 1] = vehicle.cell_id;
                for int_occupied_cell_id in vehicle.tail_cells.iter() {
                    if *int_occupied_cell_id < 1 {
                        continue;
                    }
                    self.push_intention(*int_occupied_cell_id, vehicle, IntentionType::Tail);
                }
                // @todo: This should be used is Golang's "defer" alternative
                self.push_intention(cell_id, vehicle, intention_type);
                return;
            }

            let skip_size = vehicle.intermediate_cells.len();
            match skip_size {
                // Moving with acceleration, but it is not greater than vehicle's size
                skip_size if skip_size > 0 && skip_size < tail_size => {
                    // If there are fewer intermediate cells than the vehicle's size, offset vehicle tail
                    vehicle.tail_intention_cells[0..(tail_size - skip_size - 1)]
                        .copy_from_slice(&vehicle.tail_cells[(skip_size + 1)..]);
                    vehicle.tail_intention_cells[tail_size - skip_size - 1] = vehicle.cell_id;
                    vehicle.tail_intention_cells[(tail_size - skip_size)..]
                        .copy_from_slice(&vehicle.intermediate_cells);
                }
                // Moving with acceleration and it is greater or equal to vehicle's size
                skip_size if skip_size > 0 && skip_size >= tail_size => {
                    // If there are more intermediate cells than the vehicle's size,
                    // return the last N intermediate cells
                    vehicle
                        .tail_cells
                        .copy_from_slice(&vehicle.intermediate_cells[skip_size - tail_size..]);
                }
                // Otherwise
                _ => {
                    for i in 0..tail_size - 1 {
                        vehicle.tail_cells[i] = vehicle.tail_cells[i + 1];
                    }
                    vehicle.tail_cells[tail_size - 1] = vehicle.cell_id;
                }
            }
            for int_occupied_cell_id in vehicle.tail_cells.iter() {
                if *int_occupied_cell_id < 1 {
                    continue;
                }
                self.push_intention(*int_occupied_cell_id, vehicle, IntentionType::Tail);
            }
        }
        // @todo: This should be used is Golang's "defer" alternative
        self.push_intention(cell_id, vehicle, intention_type);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_add_to_empty_intentions() {
        let mut intentions = Intentions::new();
        let mut vehicle = Vehicle::new(1).with_type(AgentType::Car).build();

        intentions.add_intention(1, &mut vehicle, IntentionType::Target);

        // Validate
        assert_eq!(intentions.intentions.len(), 1);
        assert_eq!(intentions.intentions.get(&1).unwrap().len(), 1);
        assert_eq!(
            intentions.intentions.get(&1).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.intentions.get(&1).unwrap()[0].int_type,
            IntentionType::Target
        );
    }

    #[test]
    fn test_add_multiple_to_same_cell() {
        let mut intentions = Intentions::new();

        // Add first vehicle
        let mut vehicle1 = Vehicle::new(1).with_type(AgentType::Car).build();
        intentions.add_intention(1, &mut vehicle1, IntentionType::Target);

        // Add second vehicle
        let mut vehicle2 = Vehicle::new(2).with_type(AgentType::Car).build();
        intentions.add_intention(1, &mut vehicle2, IntentionType::Transit);

        // Validate
        assert_eq!(intentions.intentions.len(), 1);
        assert_eq!(intentions.intentions.get(&1).unwrap().len(), 2);
        assert_eq!(
            intentions.intentions.get(&1).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.intentions.get(&1).unwrap()[0].int_type,
            IntentionType::Target
        );
        assert_eq!(
            intentions.intentions.get(&1).unwrap()[1]
                .get_vehicle()
                .unwrap()
                .id,
            2
        );
        assert_eq!(
            intentions.intentions.get(&1).unwrap()[1].int_type,
            IntentionType::Transit
        );
    }

    #[test]
    fn test_add_to_different_cells() {
        let mut intentions = Intentions::new();

        // Add to first cell
        let mut vehicle1 = Vehicle::new(1).with_type(AgentType::Car).build();
        intentions.add_intention(1, &mut vehicle1, IntentionType::Target);

        // Add to second cell
        let mut vehicle2 = Vehicle::new(2).with_type(AgentType::Car).build();
        intentions.add_intention(2, &mut vehicle2, IntentionType::Target);

        // Validate
        assert_eq!(intentions.intentions.len(), 2);
        assert_eq!(intentions.intentions.get(&1).unwrap().len(), 1);
        assert_eq!(intentions.intentions.get(&2).unwrap().len(), 1);
        assert_eq!(
            intentions.intentions.get(&1).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.intentions.get(&1).unwrap()[0].int_type,
            IntentionType::Target
        );
        assert_eq!(
            intentions.intentions.get(&2).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            2
        );
        assert_eq!(
            intentions.intentions.get(&2).unwrap()[0].int_type,
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
            .with_intention_tail(vec![9, 10])
            .build();
        intentions.add_intention(11, &mut vehicle1, IntentionType::Target);

        // Add second vehicle
        let mut vehicle2 = Vehicle::new(2).with_type(AgentType::Car).build();
        intentions.add_intention(10, &mut vehicle2, IntentionType::Target);

        // Validate
        assert_eq!(intentions.intentions.len(), 3); // Cells 11, 10, 9
        assert!(!intentions.intentions.contains_key(&8)); // Cell 8 should not be included
        assert_eq!(intentions.intentions.get(&9).unwrap().len(), 1); // Tail end
        assert_eq!(intentions.intentions.get(&10).unwrap().len(), 2); // Tail start + vehicle 2
        assert_eq!(intentions.intentions.get(&11).unwrap().len(), 1); // Vehicle 1 head

        // Check vehicle 1 tail and intentions
        assert_eq!(
            intentions.intentions.get(&9).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.intentions.get(&9).unwrap()[0].int_type,
            IntentionType::Tail
        );
        assert_eq!(
            intentions.intentions.get(&10).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.intentions.get(&10).unwrap()[0].int_type,
            IntentionType::Tail
        );

        // Check vehicle 2
        assert_eq!(
            intentions.intentions.get(&10).unwrap()[1]
                .get_vehicle()
                .unwrap()
                .id,
            2
        );
        assert_eq!(
            intentions.intentions.get(&10).unwrap()[1].int_type,
            IntentionType::Target
        );

        // Check vehicle 1 head
        assert_eq!(
            intentions.intentions.get(&11).unwrap()[0]
                .get_vehicle()
                .unwrap()
                .id,
            1
        );
        assert_eq!(
            intentions.intentions.get(&11).unwrap()[0].int_type,
            IntentionType::Target
        );
    }
}
