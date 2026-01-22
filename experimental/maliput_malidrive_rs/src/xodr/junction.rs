//! XODR Junction types.
//!
//! Junctions represent intersections where multiple roads connect.

use crate::common::{MalidriveError, MalidriveResult};
use crate::xodr::Connection;

/// Junction type classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum JunctionType {
    /// Default junction type.
    #[default]
    Default,
    /// Virtual junction (no physical intersection).
    Virtual,
    /// Direct junction (direct road connections).
    Direct,
}

impl std::fmt::Display for JunctionType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            JunctionType::Default => write!(f, "default"),
            JunctionType::Virtual => write!(f, "virtual"),
            JunctionType::Direct => write!(f, "direct"),
        }
    }
}

impl std::str::FromStr for JunctionType {
    type Err = MalidriveError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "default" | "" => Ok(JunctionType::Default),
            "virtual" => Ok(JunctionType::Virtual),
            "direct" => Ok(JunctionType::Direct),
            _ => Err(MalidriveError::InvalidAttribute {
                attribute: "junction type".to_string(),
                value: s.to_string(),
            }),
        }
    }
}

/// XODR Junction definition.
///
/// A junction represents an intersection where multiple roads connect.
/// It contains connections that define how lanes from incoming roads
/// connect to lanes in connecting roads.
#[derive(Debug, Clone, PartialEq)]
pub struct Junction {
    /// Junction ID.
    pub id: String,
    /// Junction name (optional).
    pub name: Option<String>,
    /// Junction type.
    pub junction_type: JunctionType,
    /// Connections within this junction.
    pub connections: Vec<Connection>,
}

impl Junction {
    /// Creates a new junction with the given ID.
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            name: None,
            junction_type: JunctionType::default(),
            connections: Vec::new(),
        }
    }

    /// Creates a named junction.
    pub fn with_name(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            name: Some(name.into()),
            junction_type: JunctionType::default(),
            connections: Vec::new(),
        }
    }

    /// Adds a connection to the junction.
    pub fn add_connection(&mut self, connection: Connection) {
        self.connections.push(connection);
    }

    /// Returns the number of connections.
    pub fn num_connections(&self) -> usize {
        self.connections.len()
    }

    /// Returns a connection by its ID.
    pub fn connection_by_id(&self, id: &str) -> Option<&Connection> {
        self.connections.iter().find(|c| c.id == id)
    }

    /// Returns connections that have the given incoming road.
    pub fn connections_from_road(&self, road_id: &str) -> Vec<&Connection> {
        self.connections
            .iter()
            .filter(|c| c.incoming_road == road_id)
            .collect()
    }

    /// Returns connections that use the given connecting road.
    pub fn connections_with_road(&self, road_id: &str) -> Vec<&Connection> {
        self.connections
            .iter()
            .filter(|c| c.connecting_road == road_id)
            .collect()
    }

    /// Returns all unique incoming road IDs.
    pub fn incoming_roads(&self) -> Vec<&str> {
        let mut roads: Vec<&str> = self.connections.iter().map(|c| c.incoming_road.as_str()).collect();
        roads.sort();
        roads.dedup();
        roads
    }

    /// Returns all unique connecting road IDs.
    pub fn connecting_roads(&self) -> Vec<&str> {
        let mut roads: Vec<&str> = self.connections.iter().map(|c| c.connecting_road.as_str()).collect();
        roads.sort();
        roads.dedup();
        roads
    }

    /// Validates the junction.
    pub fn validate(&self) -> MalidriveResult<()> {
        if self.connections.is_empty() {
            return Err(MalidriveError::ValidationError(format!(
                "Junction {} has no connections",
                self.id
            )));
        }

        // Check for unique connection IDs
        let mut ids: Vec<&str> = self.connections.iter().map(|c| c.id.as_str()).collect();
        ids.sort();
        for i in 1..ids.len() {
            if ids[i] == ids[i - 1] {
                return Err(MalidriveError::ValidationError(format!(
                    "Junction {} has duplicate connection ID: {}",
                    self.id, ids[i]
                )));
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xodr::ContactPoint;

    #[test]
    fn test_junction_type_from_str() {
        assert_eq!(
            "default".parse::<JunctionType>().unwrap(),
            JunctionType::Default
        );
        assert_eq!(
            "virtual".parse::<JunctionType>().unwrap(),
            JunctionType::Virtual
        );
        assert_eq!(
            "direct".parse::<JunctionType>().unwrap(),
            JunctionType::Direct
        );
        assert!("invalid".parse::<JunctionType>().is_err());
    }

    #[test]
    fn test_junction_creation() {
        let junction = Junction::new("1");
        assert_eq!(junction.id, "1");
        assert!(junction.name.is_none());
        assert_eq!(junction.junction_type, JunctionType::Default);
        assert!(junction.connections.is_empty());
    }

    #[test]
    fn test_junction_with_name() {
        let junction = Junction::with_name("1", "Main Intersection");
        assert_eq!(junction.id, "1");
        assert_eq!(junction.name, Some("Main Intersection".to_string()));
    }

    #[test]
    fn test_junction_connections() {
        let mut junction = Junction::new("1");

        let conn1 = Connection::new("0", "10", "100", ContactPoint::Start);
        let conn2 = Connection::new("1", "10", "101", ContactPoint::Start);
        let conn3 = Connection::new("2", "20", "102", ContactPoint::Start);

        junction.add_connection(conn1);
        junction.add_connection(conn2);
        junction.add_connection(conn3);

        assert_eq!(junction.num_connections(), 3);

        // Test connection lookup
        assert!(junction.connection_by_id("0").is_some());
        assert!(junction.connection_by_id("99").is_none());

        // Test connections from road
        let from_10 = junction.connections_from_road("10");
        assert_eq!(from_10.len(), 2);

        // Test connections with road
        let with_100 = junction.connections_with_road("100");
        assert_eq!(with_100.len(), 1);
    }

    #[test]
    fn test_junction_roads() {
        let mut junction = Junction::new("1");

        junction.add_connection(Connection::new("0", "10", "100", ContactPoint::Start));
        junction.add_connection(Connection::new("1", "10", "101", ContactPoint::Start));
        junction.add_connection(Connection::new("2", "20", "100", ContactPoint::Start));

        let incoming = junction.incoming_roads();
        assert_eq!(incoming.len(), 2);
        assert!(incoming.contains(&"10"));
        assert!(incoming.contains(&"20"));

        let connecting = junction.connecting_roads();
        assert_eq!(connecting.len(), 2);
        assert!(connecting.contains(&"100"));
        assert!(connecting.contains(&"101"));
    }

    #[test]
    fn test_junction_validation() {
        let junction = Junction::new("1");
        assert!(junction.validate().is_err()); // No connections

        let mut junction = Junction::new("1");
        junction.add_connection(Connection::new("0", "10", "100", ContactPoint::Start));
        assert!(junction.validate().is_ok());

        // Duplicate connection IDs
        let mut junction = Junction::new("1");
        junction.add_connection(Connection::new("0", "10", "100", ContactPoint::Start));
        junction.add_connection(Connection::new("0", "20", "101", ContactPoint::Start));
        assert!(junction.validate().is_err());
    }
}
