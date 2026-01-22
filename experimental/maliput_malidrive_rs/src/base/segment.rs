//! Segment implementation for maliput_malidrive.

use std::sync::{Arc, Weak};

use maliput::api::{Junction, Lane, Segment, SegmentId};

/// A concrete Segment implementation for maliput_malidrive.
pub struct MalidriveSegment {
    /// Segment identifier.
    id: SegmentId,
    /// The junction containing this segment.
    junction: Weak<dyn Junction>,
    /// The lanes in this segment (ordered from right to left).
    lanes: Vec<Arc<dyn Lane>>,
}

impl MalidriveSegment {
    /// Creates a new empty MalidriveSegment.
    pub fn new(id: SegmentId, junction: Weak<dyn Junction>) -> Self {
        Self {
            id,
            junction,
            lanes: Vec::new(),
        }
    }

    /// Adds a lane to this segment.
    ///
    /// Lanes should be added in order from right to left (increasing index).
    pub fn add_lane(&mut self, lane: Arc<dyn Lane>) {
        self.lanes.push(lane);
    }

    /// Sets the lanes for this segment.
    pub fn set_lanes(&mut self, lanes: Vec<Arc<dyn Lane>>) {
        self.lanes = lanes;
    }
}

impl Segment for MalidriveSegment {
    fn id(&self) -> &SegmentId {
        &self.id
    }

    fn junction(&self) -> Arc<dyn Junction> {
        self.junction.upgrade().expect("Junction has been dropped")
    }

    fn num_lanes(&self) -> usize {
        self.lanes.len()
    }

    fn lane(&self, index: usize) -> Arc<dyn Lane> {
        Arc::clone(&self.lanes[index])
    }
}

impl std::fmt::Debug for MalidriveSegment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MalidriveSegment")
            .field("id", &self.id)
            .field("num_lanes", &self.lanes.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_segment_creation() {
        let id = SegmentId::new("segment_1");
        // Note: A full test would require a mock Junction
        // Here we just test the ID creation
        assert_eq!(id.string(), "segment_1");
    }
}
