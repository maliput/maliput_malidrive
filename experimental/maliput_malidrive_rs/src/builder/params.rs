//! Builder parameters for road network construction.

use std::collections::HashMap;

/// Builder configuration parameters.
///
/// These parameters control the construction of a road network from OpenDRIVE data.
#[derive(Debug, Clone)]
pub struct BuilderParams {
    /// Path to the OpenDRIVE file.
    pub opendrive_file: String,
    /// ID for the RoadGeometry.
    pub road_geometry_id: String,
    /// Linear tolerance for geometric computations (meters).
    pub linear_tolerance: f64,
    /// Angular tolerance for geometric computations (radians).
    pub angular_tolerance: f64,
    /// Scale length factor.
    pub scale_length: f64,
    /// Translation from inertial to backend frame {x, y, z}.
    pub inertial_to_backend_frame_translation: [f64; 3],
    /// Build policy: "sequential" or "parallel".
    pub build_policy: BuildPolicy,
    /// Standard strictness policy for OpenDRIVE compliance.
    pub standard_strictness_policy: StandardStrictnessPolicy,
    /// Whether to omit non-drivable lanes.
    pub omit_nondrivable_lanes: bool,
}

/// Build policy for construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BuildPolicy {
    /// Build sequentially.
    #[default]
    Sequential,
    /// Build in parallel.
    Parallel,
}

/// Standard strictness policy for OpenDRIVE compliance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StandardStrictnessPolicy {
    /// Strict: fail on any non-compliance.
    Strict,
    /// Allow schema errors but not semantic errors.
    AllowSchemaErrors,
    /// Allow semantic errors but not schema errors.
    AllowSemanticErrors,
    /// Permissive: allow all errors (default).
    #[default]
    Permissive,
}

impl Default for BuilderParams {
    fn default() -> Self {
        Self {
            opendrive_file: String::new(),
            road_geometry_id: "maliput".to_string(),
            linear_tolerance: 5e-2,
            angular_tolerance: 1e-3,
            scale_length: 1.0,
            inertial_to_backend_frame_translation: [0.0, 0.0, 0.0],
            build_policy: BuildPolicy::Sequential,
            standard_strictness_policy: StandardStrictnessPolicy::Permissive,
            omit_nondrivable_lanes: true,
        }
    }
}

impl BuilderParams {
    /// Creates a new BuilderParams with the given OpenDRIVE file path.
    pub fn new(opendrive_file: impl Into<String>) -> Self {
        Self {
            opendrive_file: opendrive_file.into(),
            ..Default::default()
        }
    }

    /// Creates BuilderParams from a map of string parameters.
    ///
    /// This is compatible with the maliput plugin system.
    pub fn from_map(params: &HashMap<String, String>) -> Self {
        let mut builder = Self::default();

        if let Some(file) = params.get("opendrive_file") {
            builder.opendrive_file = file.clone();
        }

        if let Some(id) = params.get("road_geometry_id") {
            builder.road_geometry_id = id.clone();
        }

        if let Some(tol) = params.get("linear_tolerance") {
            if let Ok(v) = tol.parse() {
                builder.linear_tolerance = v;
            }
        }

        if let Some(tol) = params.get("angular_tolerance") {
            if let Ok(v) = tol.parse() {
                builder.angular_tolerance = v;
            }
        }

        if let Some(scale) = params.get("scale_length") {
            if let Ok(v) = scale.parse() {
                builder.scale_length = v;
            }
        }

        if let Some(translation) = params.get("inertial_to_backend_frame_translation") {
            // Parse "{x,y,z}" format
            let trimmed = translation.trim_matches(|c| c == '{' || c == '}');
            let parts: Vec<&str> = trimmed.split(',').collect();
            if parts.len() == 3 {
                if let (Ok(x), Ok(y), Ok(z)) = (
                    parts[0].trim().parse(),
                    parts[1].trim().parse(),
                    parts[2].trim().parse(),
                ) {
                    builder.inertial_to_backend_frame_translation = [x, y, z];
                }
            }
        }

        if let Some(policy) = params.get("build_policy") {
            builder.build_policy = match policy.as_str() {
                "parallel" => BuildPolicy::Parallel,
                _ => BuildPolicy::Sequential,
            };
        }

        if let Some(strictness) = params.get("standard_strictness_policy") {
            builder.standard_strictness_policy = match strictness.as_str() {
                "strict" => StandardStrictnessPolicy::Strict,
                "allow_schema_errors" => StandardStrictnessPolicy::AllowSchemaErrors,
                "allow_semantic_errors" => StandardStrictnessPolicy::AllowSemanticErrors,
                _ => StandardStrictnessPolicy::Permissive,
            };
        }

        if let Some(omit) = params.get("omit_nondrivable_lanes") {
            builder.omit_nondrivable_lanes = omit == "true" || omit == "1";
        }

        builder
    }

    /// Sets the OpenDRIVE file path.
    pub fn with_opendrive_file(mut self, path: impl Into<String>) -> Self {
        self.opendrive_file = path.into();
        self
    }

    /// Sets the road geometry ID.
    pub fn with_road_geometry_id(mut self, id: impl Into<String>) -> Self {
        self.road_geometry_id = id.into();
        self
    }

    /// Sets the linear tolerance.
    pub fn with_linear_tolerance(mut self, tolerance: f64) -> Self {
        self.linear_tolerance = tolerance;
        self
    }

    /// Sets the angular tolerance.
    pub fn with_angular_tolerance(mut self, tolerance: f64) -> Self {
        self.angular_tolerance = tolerance;
        self
    }

    /// Sets the scale length.
    pub fn with_scale_length(mut self, scale: f64) -> Self {
        self.scale_length = scale;
        self
    }

    /// Sets the build policy.
    pub fn with_build_policy(mut self, policy: BuildPolicy) -> Self {
        self.build_policy = policy;
        self
    }

    /// Sets the standard strictness policy.
    pub fn with_strictness_policy(mut self, policy: StandardStrictnessPolicy) -> Self {
        self.standard_strictness_policy = policy;
        self
    }

    /// Sets whether to omit non-drivable lanes.
    pub fn with_omit_nondrivable_lanes(mut self, omit: bool) -> Self {
        self.omit_nondrivable_lanes = omit;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_params() {
        let params = BuilderParams::default();
        assert_eq!(params.road_geometry_id, "maliput");
        assert_eq!(params.linear_tolerance, 5e-2);
        assert_eq!(params.angular_tolerance, 1e-3);
        assert!(params.omit_nondrivable_lanes);
    }

    #[test]
    fn test_params_from_map() {
        let mut map = HashMap::new();
        map.insert("opendrive_file".to_string(), "test.xodr".to_string());
        map.insert("road_geometry_id".to_string(), "my_rg".to_string());
        map.insert("linear_tolerance".to_string(), "1e-3".to_string());

        let params = BuilderParams::from_map(&map);
        assert_eq!(params.opendrive_file, "test.xodr");
        assert_eq!(params.road_geometry_id, "my_rg");
        assert_eq!(params.linear_tolerance, 1e-3);
    }

    #[test]
    fn test_builder_pattern() {
        let params = BuilderParams::new("test.xodr")
            .with_road_geometry_id("custom_id")
            .with_linear_tolerance(1e-4)
            .with_build_policy(BuildPolicy::Parallel);

        assert_eq!(params.opendrive_file, "test.xodr");
        assert_eq!(params.road_geometry_id, "custom_id");
        assert_eq!(params.linear_tolerance, 1e-4);
        assert_eq!(params.build_policy, BuildPolicy::Parallel);
    }
}
