//! RoadGeometry builder for constructing maliput road geometry from OpenDRIVE.

use std::sync::Arc;

use maliput::api::RoadGeometryId;

use crate::base::MalidriveRoadGeometry;
use crate::builder::{BuilderParams, RoadCurveFactory};
use crate::common::{MalidriveError, MalidriveResult};
use crate::xodr::RoadHeader;

/// Builder for constructing MalidriveRoadGeometry from OpenDRIVE data.
pub struct RoadGeometryBuilder {
    /// Builder parameters.
    params: BuilderParams,
    /// Road curve factory.
    road_curve_factory: RoadCurveFactory,
}

impl RoadGeometryBuilder {
    /// Creates a new RoadGeometryBuilder with the given parameters.
    pub fn new(params: BuilderParams) -> Self {
        let road_curve_factory = RoadCurveFactory::new(
            params.linear_tolerance,
            params.scale_length,
        );

        Self {
            params,
            road_curve_factory,
        }
    }

    /// Builds the road geometry from parsed OpenDRIVE roads.
    pub fn build(&self, roads: &[RoadHeader]) -> MalidriveResult<Arc<MalidriveRoadGeometry>> {
        // Create the road geometry
        let mut road_geometry = MalidriveRoadGeometry::new(
            RoadGeometryId::new(&self.params.road_geometry_id),
            self.params.linear_tolerance,
            self.params.angular_tolerance,
            self.params.scale_length,
            self.params.inertial_to_backend_frame_translation,
        );

        // Process each road
        for road in roads {
            self.process_road(road, &mut road_geometry)?;
        }

        Ok(Arc::new(road_geometry))
    }

    /// Processes a single road and adds its components to the road geometry.
    fn process_road(
        &self,
        road: &RoadHeader,
        _road_geometry: &mut MalidriveRoadGeometry,
    ) -> MalidriveResult<()> {
        // Create the road curve for this road
        let _road_curve = self.road_curve_factory.make_road_curve(road)?;

        // Process lane sections
        if let Some(ref lanes) = road.lanes {
            for _lane_section in &lanes.lane_sections {
                // TODO: Create junctions, segments, and lanes from lane sections
                // This is a complex process that involves:
                // 1. Creating a Junction for each road (or using existing junctions)
                // 2. Creating Segments for each lane section
                // 3. Creating Lanes for each lane in the section
                // 4. Setting up BranchPoints for lane connectivity
            }
        }

        Ok(())
    }

    /// Returns the builder parameters.
    pub fn params(&self) -> &BuilderParams {
        &self.params
    }
}

/// Convenience function to build a road geometry from OpenDRIVE data.
pub fn build_road_geometry(
    params: BuilderParams,
    roads: &[RoadHeader],
) -> MalidriveResult<Arc<MalidriveRoadGeometry>> {
    let builder = RoadGeometryBuilder::new(params);
    builder.build(roads)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_creation() {
        let params = BuilderParams::new("test.xodr")
            .with_linear_tolerance(1e-3);

        let builder = RoadGeometryBuilder::new(params);
        assert_eq!(builder.params().opendrive_file, "test.xodr");
        assert_eq!(builder.params().linear_tolerance, 1e-3);
    }

    #[test]
    fn test_build_empty_road_geometry() {
        let params = BuilderParams::new("test.xodr");
        let builder = RoadGeometryBuilder::new(params);

        let rg = builder.build(&[]).unwrap();
        assert_eq!(rg.num_junctions(), 0);
    }
}
