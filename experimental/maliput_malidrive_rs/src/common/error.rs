//! Error types for maliput_malidrive.

use thiserror::Error;

/// The main error type for maliput_malidrive operations.
#[derive(Error, Debug)]
pub enum MalidriveError {
    /// Error during XODR parsing.
    #[error("XODR parse error: {0}")]
    XodrParseError(String),

    /// Error during road geometry construction.
    #[error("Road geometry construction error: {0}")]
    RoadGeometryConstructionError(String),

    /// Error during road curve computation.
    #[error("Road curve error: {0}")]
    RoadCurveError(String),

    /// Parameter out of range error.
    #[error("Parameter {parameter} = {value} is out of range [{min}, {max}]")]
    ParameterOutOfRange {
        parameter: String,
        value: f64,
        min: f64,
        max: f64,
    },

    /// Invalid geometry type.
    #[error("Invalid geometry type: {0}")]
    InvalidGeometryType(String),

    /// Missing required element in XODR.
    #[error("Missing required element: {0}")]
    MissingElement(String),

    /// Invalid attribute value in XODR.
    #[error("Invalid attribute value for '{attribute}': {value}")]
    InvalidAttribute { attribute: String, value: String },

    /// IO error.
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    /// XML parsing error.
    #[error("XML error: {0}")]
    XmlError(String),

    /// Validation error.
    #[error("Validation error: {0}")]
    ValidationError(String),

    /// Geometry computation error.
    #[error("Geometry error: {0}")]
    GeometryError(String),

    /// General parsing error.
    #[error("Parsing error: {0}")]
    ParsingError(String),

    /// Unsupported feature in XODR.
    #[error("Unsupported XODR feature: {0}")]
    UnsupportedFeature(String),

    /// Semantic error in XODR data.
    #[error("Semantic error: {0}")]
    SemanticError(String),

    /// Builder configuration error.
    #[error("Builder configuration error: {0}")]
    BuilderConfigError(String),
}

impl From<quick_xml::Error> for MalidriveError {
    fn from(err: quick_xml::Error) -> Self {
        MalidriveError::XmlError(err.to_string())
    }
}

impl From<quick_xml::events::attributes::AttrError> for MalidriveError {
    fn from(err: quick_xml::events::attributes::AttrError) -> Self {
        MalidriveError::XmlError(err.to_string())
    }
}

/// Result type alias for maliput_malidrive operations.
pub type MalidriveResult<T> = Result<T, MalidriveError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = MalidriveError::XodrParseError("test error".to_string());
        assert_eq!(format!("{}", err), "XODR parse error: test error");

        let err = MalidriveError::ParameterOutOfRange {
            parameter: "p".to_string(),
            value: 10.0,
            min: 0.0,
            max: 5.0,
        };
        assert_eq!(
            format!("{}", err),
            "Parameter p = 10 is out of range [0, 5]"
        );
    }

    #[test]
    fn test_error_conversion() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let err: MalidriveError = io_err.into();
        assert!(matches!(err, MalidriveError::IoError(_)));
    }
}
