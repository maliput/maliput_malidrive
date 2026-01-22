//! XODR Connection types.
//!
//! Connections define how roads connect within a junction.

use crate::xodr::ContactPoint;

/// Lane link within a connection.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConnectionLaneLink {
    /// Lane ID in the incoming road.
    pub from: i32,
    /// Lane ID in the connecting road.
    pub to: i32,
}

impl ConnectionLaneLink {
    /// Creates a new lane link.
    pub fn new(from: i32, to: i32) -> Self {
        Self { from, to }
    }
}

/// Junction connection defining how roads connect.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Connection {
    /// Connection ID within the junction.
    pub id: String,
    /// ID of the incoming road.
    pub incoming_road: String,
    /// ID of the connecting road (the road within the junction).
    pub connecting_road: String,
    /// Contact point on the connecting road.
    pub contact_point: ContactPoint,
    /// Lane links defining how lanes connect.
    pub lane_links: Vec<ConnectionLaneLink>,
}

impl Connection {
    /// Creates a new connection.
    pub fn new(
        id: impl Into<String>,
        incoming_road: impl Into<String>,
        connecting_road: impl Into<String>,
        contact_point: ContactPoint,
    ) -> Self {
        Self {
            id: id.into(),
            incoming_road: incoming_road.into(),
            connecting_road: connecting_road.into(),
            contact_point,
            lane_links: Vec::new(),
        }
    }

    /// Adds a lane link to the connection.
    pub fn add_lane_link(&mut self, from: i32, to: i32) {
        self.lane_links.push(ConnectionLaneLink::new(from, to));
    }

    /// Returns the connected lane ID for a given incoming lane ID.
    pub fn connected_lane(&self, from_lane: i32) -> Option<i32> {
        self.lane_links
            .iter()
            .find(|link| link.from == from_lane)
            .map(|link| link.to)
    }

    /// Returns all from-lane IDs.
    pub fn from_lanes(&self) -> Vec<i32> {
        self.lane_links.iter().map(|link| link.from).collect()
    }

    /// Returns all to-lane IDs.
    pub fn to_lanes(&self) -> Vec<i32> {
        self.lane_links.iter().map(|link| link.to).collect()
    }

    /// Returns true if the connection has lane links.
    pub fn has_lane_links(&self) -> bool {
        !self.lane_links.is_empty()
    }

    /// Returns the number of lane links.
    pub fn num_lane_links(&self) -> usize {
        self.lane_links.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connection_lane_link() {
        let link = ConnectionLaneLink::new(-1, -1);
        assert_eq!(link.from, -1);
        assert_eq!(link.to, -1);
    }

    #[test]
    fn test_connection_creation() {
        let conn = Connection::new("0", "10", "100", ContactPoint::Start);
        assert_eq!(conn.id, "0");
        assert_eq!(conn.incoming_road, "10");
        assert_eq!(conn.connecting_road, "100");
        assert_eq!(conn.contact_point, ContactPoint::Start);
        assert!(conn.lane_links.is_empty());
    }

    #[test]
    fn test_connection_lane_links() {
        let mut conn = Connection::new("0", "10", "100", ContactPoint::Start);
        conn.add_lane_link(-1, -1);
        conn.add_lane_link(-2, -1);

        assert!(conn.has_lane_links());
        assert_eq!(conn.num_lane_links(), 2);

        assert_eq!(conn.connected_lane(-1), Some(-1));
        assert_eq!(conn.connected_lane(-2), Some(-1));
        assert_eq!(conn.connected_lane(-3), None);

        assert_eq!(conn.from_lanes(), vec![-1, -2]);
        assert_eq!(conn.to_lanes(), vec![-1, -1]);
    }
}
