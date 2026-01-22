//! XODR Road Link types.
//!
//! Road links define connectivity between roads at their start and end points.

use crate::common::MalidriveError;

/// Element type for road link predecessor/successor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElementType {
    /// Link to another road.
    Road,
    /// Link to a junction.
    Junction,
}

impl std::fmt::Display for ElementType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ElementType::Road => write!(f, "road"),
            ElementType::Junction => write!(f, "junction"),
        }
    }
}

impl std::str::FromStr for ElementType {
    type Err = MalidriveError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "road" => Ok(ElementType::Road),
            "junction" => Ok(ElementType::Junction),
            _ => Err(MalidriveError::InvalidAttribute {
                attribute: "elementType".to_string(),
                value: s.to_string(),
            }),
        }
    }
}

/// Contact point indicating which end of the linked road connects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactPoint {
    /// Start of the road (s = 0).
    Start,
    /// End of the road (s = length).
    End,
}

impl std::fmt::Display for ContactPoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ContactPoint::Start => write!(f, "start"),
            ContactPoint::End => write!(f, "end"),
        }
    }
}

impl std::str::FromStr for ContactPoint {
    type Err = MalidriveError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "start" => Ok(ContactPoint::Start),
            "end" => Ok(ContactPoint::End),
            _ => Err(MalidriveError::InvalidAttribute {
                attribute: "contactPoint".to_string(),
                value: s.to_string(),
            }),
        }
    }
}

/// Link element (predecessor or successor).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LinkElement {
    /// Type of the linked element.
    pub element_type: ElementType,
    /// ID of the linked element.
    pub element_id: String,
    /// Contact point on the linked element (only for road links).
    pub contact_point: Option<ContactPoint>,
}

impl LinkElement {
    /// Creates a new link element.
    pub fn new(
        element_type: ElementType,
        element_id: impl Into<String>,
        contact_point: Option<ContactPoint>,
    ) -> Self {
        Self {
            element_type,
            element_id: element_id.into(),
            contact_point,
        }
    }

    /// Creates a road link.
    pub fn road(element_id: impl Into<String>, contact_point: ContactPoint) -> Self {
        Self::new(ElementType::Road, element_id, Some(contact_point))
    }

    /// Creates a junction link.
    pub fn junction(element_id: impl Into<String>) -> Self {
        Self::new(ElementType::Junction, element_id, None)
    }
}

/// Road link containing predecessor and successor connections.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RoadLink {
    /// Predecessor element (at s = 0).
    pub predecessor: Option<LinkElement>,
    /// Successor element (at s = length).
    pub successor: Option<LinkElement>,
}

impl RoadLink {
    /// Creates a new empty road link.
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a road link with only a predecessor.
    pub fn with_predecessor(predecessor: LinkElement) -> Self {
        Self {
            predecessor: Some(predecessor),
            successor: None,
        }
    }

    /// Creates a road link with only a successor.
    pub fn with_successor(successor: LinkElement) -> Self {
        Self {
            predecessor: None,
            successor: Some(successor),
        }
    }

    /// Creates a road link with both predecessor and successor.
    pub fn with_both(predecessor: LinkElement, successor: LinkElement) -> Self {
        Self {
            predecessor: Some(predecessor),
            successor: Some(successor),
        }
    }

    /// Returns true if the road has a predecessor.
    pub fn has_predecessor(&self) -> bool {
        self.predecessor.is_some()
    }

    /// Returns true if the road has a successor.
    pub fn has_successor(&self) -> bool {
        self.successor.is_some()
    }

    /// Returns true if the road has no links.
    pub fn is_empty(&self) -> bool {
        self.predecessor.is_none() && self.successor.is_none()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_element_type() {
        assert_eq!("road".parse::<ElementType>().unwrap(), ElementType::Road);
        assert_eq!(
            "junction".parse::<ElementType>().unwrap(),
            ElementType::Junction
        );
        assert!("invalid".parse::<ElementType>().is_err());
    }

    #[test]
    fn test_contact_point() {
        assert_eq!(
            "start".parse::<ContactPoint>().unwrap(),
            ContactPoint::Start
        );
        assert_eq!("end".parse::<ContactPoint>().unwrap(), ContactPoint::End);
        assert!("invalid".parse::<ContactPoint>().is_err());
    }

    #[test]
    fn test_link_element_road() {
        let link = LinkElement::road("1", ContactPoint::Start);
        assert_eq!(link.element_type, ElementType::Road);
        assert_eq!(link.element_id, "1");
        assert_eq!(link.contact_point, Some(ContactPoint::Start));
    }

    #[test]
    fn test_link_element_junction() {
        let link = LinkElement::junction("5");
        assert_eq!(link.element_type, ElementType::Junction);
        assert_eq!(link.element_id, "5");
        assert_eq!(link.contact_point, None);
    }

    #[test]
    fn test_road_link() {
        let mut link = RoadLink::new();
        assert!(link.is_empty());
        assert!(!link.has_predecessor());
        assert!(!link.has_successor());

        link.predecessor = Some(LinkElement::road("1", ContactPoint::End));
        assert!(link.has_predecessor());
        assert!(!link.is_empty());

        link.successor = Some(LinkElement::road("2", ContactPoint::Start));
        assert!(link.has_successor());
    }

    #[test]
    fn test_road_link_constructors() {
        let link = RoadLink::with_predecessor(LinkElement::road("1", ContactPoint::End));
        assert!(link.has_predecessor());
        assert!(!link.has_successor());

        let link = RoadLink::with_successor(LinkElement::road("2", ContactPoint::Start));
        assert!(!link.has_predecessor());
        assert!(link.has_successor());

        let link = RoadLink::with_both(
            LinkElement::road("1", ContactPoint::End),
            LinkElement::road("2", ContactPoint::Start),
        );
        assert!(link.has_predecessor());
        assert!(link.has_successor());
    }
}
