//! XODR Lane Link types.
//!
//! Lane links define connectivity between lanes across lane section boundaries.

/// Lane link defining predecessor and successor lane IDs.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct LaneLink {
    /// ID of the predecessor lane (in previous lane section).
    pub predecessor_id: Option<i32>,
    /// ID of the successor lane (in next lane section).
    pub successor_id: Option<i32>,
}

impl LaneLink {
    /// Creates a new lane link with the given predecessor and successor IDs.
    pub fn new(predecessor_id: Option<i32>, successor_id: Option<i32>) -> Self {
        Self {
            predecessor_id,
            successor_id,
        }
    }

    /// Creates a lane link with only a predecessor.
    pub fn with_predecessor(id: i32) -> Self {
        Self {
            predecessor_id: Some(id),
            successor_id: None,
        }
    }

    /// Creates a lane link with only a successor.
    pub fn with_successor(id: i32) -> Self {
        Self {
            predecessor_id: None,
            successor_id: Some(id),
        }
    }

    /// Returns true if this lane link has a predecessor.
    pub fn has_predecessor(&self) -> bool {
        self.predecessor_id.is_some()
    }

    /// Returns true if this lane link has a successor.
    pub fn has_successor(&self) -> bool {
        self.successor_id.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lane_link_creation() {
        let link = LaneLink::new(Some(-1), Some(-1));
        assert_eq!(link.predecessor_id, Some(-1));
        assert_eq!(link.successor_id, Some(-1));
        assert!(link.has_predecessor());
        assert!(link.has_successor());
    }

    #[test]
    fn test_lane_link_with_predecessor() {
        let link = LaneLink::with_predecessor(-1);
        assert_eq!(link.predecessor_id, Some(-1));
        assert_eq!(link.successor_id, None);
        assert!(link.has_predecessor());
        assert!(!link.has_successor());
    }

    #[test]
    fn test_lane_link_with_successor() {
        let link = LaneLink::with_successor(-2);
        assert_eq!(link.predecessor_id, None);
        assert_eq!(link.successor_id, Some(-2));
        assert!(!link.has_predecessor());
        assert!(link.has_successor());
    }

    #[test]
    fn test_lane_link_default() {
        let link = LaneLink::default();
        assert_eq!(link.predecessor_id, None);
        assert_eq!(link.successor_id, None);
        assert!(!link.has_predecessor());
        assert!(!link.has_successor());
    }
}
