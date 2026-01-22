//! Junction implementation for maliput_malidrive.

use std::sync::{Arc, Weak};

use maliput::api::{Junction, JunctionId, MaliputError, MaliputResult, RoadGeometry, Segment};

/// A concrete Junction implementation for maliput_malidrive.
pub struct MalidriveJunction {
    /// Junction identifier.
    id: JunctionId,
    /// The road geometry containing this junction.
    road_geometry: Weak<dyn RoadGeometry>,
    /// The segments in this junction.
    segments: Vec<Arc<dyn Segment>>,
}

impl MalidriveJunction {
    /// Creates a new empty MalidriveJunction.
    pub fn new(id: JunctionId, road_geometry: Weak<dyn RoadGeometry>) -> Self {
        Self {
            id,
            road_geometry,
            segments: Vec::new(),
        }
    }

    /// Adds a segment to this junction.
    pub fn add_segment(&mut self, segment: Arc<dyn Segment>) {
        self.segments.push(segment);
    }

    /// Sets the segments for this junction.
    pub fn set_segments(&mut self, segments: Vec<Arc<dyn Segment>>) {
        self.segments = segments;
    }
}

impl Junction for MalidriveJunction {
    fn id(&self) -> &JunctionId {
        &self.id
    }

    fn road_geometry(&self) -> Arc<dyn RoadGeometry> {
        self.road_geometry
            .upgrade()
            .expect("RoadGeometry has been dropped")
    }

    fn num_segments(&self) -> usize {
        self.segments.len()
    }

    fn segment(&self, index: usize) -> MaliputResult<Arc<dyn Segment>> {
        self.segments
            .get(index)
            .cloned()
            .ok_or_else(|| MaliputError::IndexOutOfBounds {
                index,
                max: self.segments.len().saturating_sub(1),
            })
    }
}

impl std::fmt::Debug for MalidriveJunction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MalidriveJunction")
            .field("id", &self.id)
            .field("num_segments", &self.segments.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_junction_creation() {
        let id = JunctionId::new("junction_1".to_string());
        assert_eq!(id.string(), "junction_1");
    }
}
