//! Common macros for maliput_malidrive.

/// Validates that a condition is true, returning an error if not.
///
/// # Example
///
/// ```rust,ignore
/// malidrive_validate!(value > 0.0, "Value must be positive");
/// ```
#[macro_export]
macro_rules! malidrive_validate {
    ($condition:expr, $msg:expr) => {
        if !$condition {
            return Err($crate::common::MalidriveError::ValidationError(
                $msg.to_string(),
            ));
        }
    };
    ($condition:expr, $fmt:expr, $($arg:tt)*) => {
        if !$condition {
            return Err($crate::common::MalidriveError::ValidationError(
                format!($fmt, $($arg)*),
            ));
        }
    };
}

/// Validates that a value is within a range, returning an error if not.
///
/// # Example
///
/// ```rust,ignore
/// malidrive_is_in_range!(p, p0, p1, "p");
/// ```
#[macro_export]
macro_rules! malidrive_is_in_range {
    ($value:expr, $min:expr, $max:expr, $param_name:expr) => {
        if $value < $min || $value > $max {
            return Err($crate::common::MalidriveError::ParameterOutOfRange {
                parameter: $param_name.to_string(),
                value: $value as f64,
                min: $min as f64,
                max: $max as f64,
            });
        }
    };
}

/// Validates that a value is within a range with tolerance, returning an error if not.
///
/// # Example
///
/// ```rust,ignore
/// malidrive_is_in_range_with_tolerance!(p, p0, p1, tolerance, "p");
/// ```
#[macro_export]
macro_rules! malidrive_is_in_range_with_tolerance {
    ($value:expr, $min:expr, $max:expr, $tolerance:expr, $param_name:expr) => {
        if $value < ($min - $tolerance) || $value > ($max + $tolerance) {
            return Err($crate::common::MalidriveError::ParameterOutOfRange {
                parameter: $param_name.to_string(),
                value: $value as f64,
                min: ($min - $tolerance) as f64,
                max: ($max + $tolerance) as f64,
            });
        }
    };
}

/// Small epsilon value used for numerical comparisons.
pub const EPSILON: f64 = 1e-13;

/// Default linear tolerance.
pub const DEFAULT_LINEAR_TOLERANCE: f64 = 5e-2;

/// Default angular tolerance.
pub const DEFAULT_ANGULAR_TOLERANCE: f64 = 1e-3;

/// Default scale length.
pub const DEFAULT_SCALE_LENGTH: f64 = 1.0;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::MalidriveResult;

    fn validate_positive(value: f64) -> MalidriveResult<()> {
        malidrive_validate!(value > 0.0, "Value must be positive");
        Ok(())
    }

    fn validate_range(value: f64, min: f64, max: f64) -> MalidriveResult<()> {
        malidrive_is_in_range!(value, min, max, "value");
        Ok(())
    }

    #[test]
    fn test_malidrive_validate() {
        assert!(validate_positive(1.0).is_ok());
        assert!(validate_positive(-1.0).is_err());
    }

    #[test]
    fn test_malidrive_is_in_range() {
        assert!(validate_range(5.0, 0.0, 10.0).is_ok());
        assert!(validate_range(0.0, 0.0, 10.0).is_ok());
        assert!(validate_range(10.0, 0.0, 10.0).is_ok());
        assert!(validate_range(-1.0, 0.0, 10.0).is_err());
        assert!(validate_range(11.0, 0.0, 10.0).is_err());
    }

    #[test]
    fn test_constants() {
        assert!(EPSILON > 0.0);
        assert!(EPSILON < 1e-10);
        assert!(DEFAULT_LINEAR_TOLERANCE > 0.0);
        assert!(DEFAULT_ANGULAR_TOLERANCE > 0.0);
        assert!(DEFAULT_SCALE_LENGTH > 0.0);
    }
}
