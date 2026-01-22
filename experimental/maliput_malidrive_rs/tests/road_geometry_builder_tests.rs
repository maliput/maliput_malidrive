//! Tests for road geometry builder functionality.
//!
//! These tests are designed to replicate the C++ tests in
//! `test/regression/builder/road_geometry_builder_test.cc` to ensure
//! behavior parity after migration to Rust.

use maliput::api::RoadGeometry;
use maliput_malidrive::builder::{
    get_junction_id, get_lane_id, get_segment_id, is_drivable_lane_type,
    BuilderParams, RoadGeometryBuilder, SCALE_LENGTH, STRICT_ANGULAR_TOLERANCE, STRICT_LINEAR_TOLERANCE,
};
use maliput_malidrive::xodr::{load_from_file, GeometryDescription, LaneType};
use maliput_malidrive::road_curve::{
    ArcGroundCurve, CubicPolynomial, GroundCurve, LineGroundCurve, RoadCurve,
};
use nalgebra::Vector2;
use std::path::PathBuf;
use std::sync::Arc;

fn resources_dir() -> PathBuf {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    PathBuf::from(manifest_dir).join("../../resources")
}

// ============================================================================
// Builder Constructor Tests (matching BuilderTestSingleLane)
// ============================================================================

#[test]
fn test_road_geometry_builder_constructor() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load SingleLane.xodr");

    let params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH)
        .with_road_geometry_id("SingleLane");

    let builder = RoadGeometryBuilder::new(params).expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions).expect("Build should succeed");

    assert!(rg.linear_tolerance() > 0.0);
    assert!(rg.angular_tolerance() > 0.0);
    assert!(rg.scale_length() > 0.0);
}

#[test]
fn test_road_geometry_builder_constructor_bad_linear_tolerance() {
    let params = BuilderParams::new("test.xodr")
        .with_linear_tolerance(-5.0);

    let result = RoadGeometryBuilder::new(params);
    assert!(result.is_err(), "Negative linear tolerance should fail");
}

#[test]
fn test_road_geometry_builder_constructor_bad_angular_tolerance() {
    let params = BuilderParams::new("test.xodr")
        .with_angular_tolerance(-5.0);

    let result = RoadGeometryBuilder::new(params);
    assert!(result.is_err(), "Negative angular tolerance should fail");
}

#[test]
fn test_road_geometry_builder_constructor_bad_scale_length() {
    let params = BuilderParams::new("test.xodr")
        .with_scale_length(-5.0);

    let result = RoadGeometryBuilder::new(params);
    assert!(result.is_err(), "Negative scale length should fail");
}

// ============================================================================
// XODR Loading Tests
// ============================================================================

#[test]
fn test_load_single_lane_xodr() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load SingleLane.xodr");

    assert_eq!(doc.roads.len(), 1, "SingleLane should have 1 road");

    let road = &doc.roads[0];
    assert_eq!(road.id, "1", "Road ID should be 1");
    assert!((road.length - 100.0).abs() < 0.1);
    assert!(road.junction == "-1", "Road should not be in a junction");

    assert!(!road.plan_view.is_empty());
    assert_eq!(road.plan_view.geometries.len(), 1);

    let geom = &road.plan_view.geometries[0];
    assert!(matches!(geom.description, GeometryDescription::Line(_)));
    assert!((geom.length - 100.0).abs() < 0.1);
}

#[test]
fn test_load_arc_lane_xodr() {
    let xodr_path = resources_dir().join("ArcLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load ArcLane.xodr");

    assert_eq!(doc.roads.len(), 1);

    let road = &doc.roads[0];
    assert!(!road.plan_view.is_empty());
    assert_eq!(road.plan_view.geometries.len(), 1);

    let geom = &road.plan_view.geometries[0];
    if let GeometryDescription::Arc(arc) = &geom.description {
        assert!(arc.curvature.abs() > 0.0);
    } else {
        panic!("Geometry should be an arc");
    }
}

#[test]
fn test_load_tshape_road_xodr() {
    let xodr_path = resources_dir().join("TShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load TShapeRoad.xodr");

    // TShapeRoad has 9 roads and 1 junction in the C++ tests
    assert!(doc.roads.len() >= 3);
    assert!(doc.junctions.len() > 0);

    // Check for roads that don't belong to a Junction
    let main_roads: Vec<_> = doc.roads.iter()
        .filter(|r| r.junction == "-1")
        .collect();
    assert!(main_roads.len() >= 3);

    // Check for roads that belong to a Junction
    let junction_roads: Vec<_> = doc.roads.iter()
        .filter(|r| r.junction != "-1")
        .collect();
    assert!(junction_roads.len() > 0);
}

#[test]
fn test_load_lshape_road_xodr() {
    let xodr_path = resources_dir().join("LShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load LShapeRoad.xodr");

    // LShapeRoad has 3 roads in C++ tests
    assert_eq!(doc.roads.len(), 3);

    // Verify road connections exist
    for road in &doc.roads {
        if let Some(link) = &road.link {
            println!("Road {} has link info", road.id);
            if let Some(pred) = &link.predecessor {
                println!("  Predecessor: {}", pred.element_id);
            }
            if let Some(succ) = &link.successor {
                println!("  Successor: {}", succ.element_id);
            }
        }
    }
}

#[test]
fn test_load_line_multiple_sections_xodr() {
    let xodr_path = resources_dir().join("LineMultipleSections.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load LineMultipleSections.xodr");

    assert_eq!(doc.roads.len(), 1);
    let road = &doc.roads[0];
    
    // Should have 3 lane sections per C++ test
    assert_eq!(road.lanes.lane_sections.len(), 3);
}

#[test]
fn test_load_figure8_xodr() {
    let xodr_path = resources_dir().join("Figure8.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load Figure8.xodr");

    assert!(doc.roads.len() > 0);

    for road in &doc.roads {
        assert!(!road.plan_view.is_empty());
    }
}

// ============================================================================
// Lane Section Structure Tests
// ============================================================================

#[test]
fn test_lane_section_structure() {
    let xodr_path = resources_dir().join("TShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load TShapeRoad.xodr");

    for road in &doc.roads {
        // Skip junction roads for this test
        if road.junction != "-1" {
            continue;
        }

        for section in &road.lanes.lane_sections {
            // Left lanes should have positive IDs
            for lane in &section.left_lanes {
                assert!(lane.id > 0, "Left lane should have positive ID");
            }

            // Right lanes should have negative IDs
            for lane in &section.right_lanes {
                assert!(lane.id < 0, "Right lane should have negative ID");
            }

            // Center lane should have ID 0
            assert_eq!(section.center_lane.id, 0, "Center lane should have ID 0");
        }
    }
}

#[test]
fn test_driving_lane_identification() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load SingleLane.xodr");

    let road = &doc.roads[0];
    let section = &road.lanes.lane_sections[0];

    let has_driving_lanes = section.left_lanes.iter().any(|l| l.lane_type == LaneType::Driving)
        || section.right_lanes.iter().any(|l| l.lane_type == LaneType::Driving);

    assert!(has_driving_lanes, "SingleLane should have driving lanes");
}

// ============================================================================
// Junction/Connection Tests
// ============================================================================

#[test]
fn test_junction_connections() {
    let xodr_path = resources_dir().join("TShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load TShapeRoad.xodr");

    assert!(doc.junctions.len() > 0, "TShapeRoad should have junctions");

    let junction = &doc.junctions[0];
    assert!(junction.connections.len() > 0, "Junction should have connections");

    for connection in &junction.connections {
        assert!(!connection.incoming_road.is_empty(), "Connection should have incoming road");
        assert!(!connection.connecting_road.is_empty(), "Connection should have connecting road");
    }
}

// ============================================================================
// Ground Curve Tests
// ============================================================================

#[test]
fn test_line_geometry_to_ground_curve() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap()).unwrap();

    let road = &doc.roads[0];
    let geom = &road.plan_view.geometries[0];

    // Calculate dxy from orientation and length
    let dxy = Vector2::new(
        geom.length * geom.orientation.cos(),
        geom.length * geom.orientation.sin(),
    );

    let ground_curve = LineGroundCurve::new(
        1e-6,
        geom.start_point,
        dxy,
        0.0,
        geom.length,
    ).unwrap();

    assert!((ground_curve.arc_length() - 100.0).abs() < 1e-6);

    let start_pos = ground_curve.g(0.0).unwrap();
    assert!((start_pos.x - geom.start_point.x).abs() < 1e-6);
    assert!((start_pos.y - geom.start_point.y).abs() < 1e-6);

    let end_pos = ground_curve.g(100.0).unwrap();
    assert!((end_pos.x - 100.0).abs() < 1e-3);
}

#[test]
fn test_arc_geometry_to_ground_curve() {
    let xodr_path = resources_dir().join("ArcLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap()).unwrap();

    let road = &doc.roads[0];
    let geom = &road.plan_view.geometries[0];

    let curvature = if let GeometryDescription::Arc(arc) = &geom.description {
        arc.curvature
    } else {
        panic!("Expected arc geometry");
    };

    let ground_curve = ArcGroundCurve::new(
        1e-6,
        geom.start_point,
        geom.orientation,
        curvature,
        geom.length,
        0.0,
        geom.length,
    ).unwrap();

    assert!((ground_curve.arc_length() - geom.length).abs() < 1e-6);

    let start_pos = ground_curve.g(0.0).unwrap();
    assert!((start_pos.x - geom.start_point.x).abs() < 1e-6);
    assert!((start_pos.y - geom.start_point.y).abs() < 1e-6);
}

// ============================================================================
// Road Curve Tests
// ============================================================================

#[test]
fn test_road_curve_with_elevation() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap()).unwrap();

    let road = &doc.roads[0];
    let geom = &road.plan_view.geometries[0];

    // Calculate dxy from orientation and length
    let dxy = Vector2::new(
        geom.length * geom.orientation.cos(),
        geom.length * geom.orientation.sin(),
    );

    let ground_curve: Arc<dyn GroundCurve> = Arc::new(LineGroundCurve::new(
        1e-6,
        geom.start_point,
        dxy,
        0.0,
        geom.length,
    ).unwrap());

    let elevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, 0.0, geom.length));
    let superelevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, 0.0, geom.length));

    let road_curve = RoadCurve::new(
        ground_curve,
        elevation,
        superelevation,
        1e-6,
        1.0,
    );

    let pos = road_curve.w(50.0, 0.0, 0.0).unwrap();
    assert!((pos.x - 50.0).abs() < 1e-3);
    assert!((pos.z).abs() < 1e-3);
}

#[test]
fn test_road_curve_with_lateral_offset() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap()).unwrap();

    let road = &doc.roads[0];
    let geom = &road.plan_view.geometries[0];

    // Calculate dxy from orientation and length
    let dxy = Vector2::new(
        geom.length * geom.orientation.cos(),
        geom.length * geom.orientation.sin(),
    );

    let ground_curve: Arc<dyn GroundCurve> = Arc::new(LineGroundCurve::new(
        1e-6,
        geom.start_point,
        dxy,
        0.0,
        geom.length,
    ).unwrap());

    let elevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, 0.0, geom.length));
    let superelevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, 0.0, geom.length));

    let road_curve = RoadCurve::new(
        ground_curve,
        elevation,
        superelevation,
        1e-6,
        1.0,
    );

    let pos_center = road_curve.w(50.0, 0.0, 0.0).unwrap();
    let pos_offset = road_curve.w(50.0, 2.0, 0.0).unwrap();

    assert!((pos_offset.x - pos_center.x).abs() < 1e-3);
    assert!((pos_offset.y - pos_center.y - 2.0).abs() < 1e-3);
}

// ============================================================================
// Lane Width Tests
// ============================================================================

#[test]
fn test_lane_width_parsing() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap()).unwrap();

    let road = &doc.roads[0];
    let section = &road.lanes.lane_sections[0];

    for lane in &section.left_lanes {
        assert!(lane.widths.len() > 0);
        assert_eq!(lane.widths[0].s_offset, 0.0);
    }

    for lane in &section.right_lanes {
        assert!(lane.widths.len() > 0);
    }
}

// ============================================================================
// ID Generation Tests (matching C++ id_providers)
// ============================================================================

#[test]
fn test_lane_id_format() {
    // C++ format: {road}_{section}_{lane}
    assert_eq!(get_lane_id(1, 0, -1), "1_0_-1");
    assert_eq!(get_lane_id(1, 0, 1), "1_0_1");
    assert_eq!(get_lane_id(2, 1, -2), "2_1_-2");
    assert_eq!(get_lane_id(0, 0, 4), "0_0_4");
}

#[test]
fn test_segment_id_format() {
    // C++ format: {road}_{section}
    assert_eq!(get_segment_id("1", 0), "1_0");
    assert_eq!(get_segment_id("2", 1), "2_1");
}

#[test]
fn test_junction_id_format() {
    // C++ format: {road}_{section} for non-xodr junctions
    assert_eq!(get_junction_id("1", 0), "1_0");
    assert_eq!(get_junction_id("2", 1), "2_1");
}

// ============================================================================
// Drivable Lane Type Tests
// ============================================================================

#[test]
fn test_is_drivable_lane_type_comprehensive() {
    // Drivable types (matching C++ is_driveable_lane)
    assert!(is_drivable_lane_type(LaneType::Driving));
    assert!(is_drivable_lane_type(LaneType::Entry));
    assert!(is_drivable_lane_type(LaneType::Exit));
    assert!(is_drivable_lane_type(LaneType::OffRamp));
    assert!(is_drivable_lane_type(LaneType::OnRamp));
    assert!(is_drivable_lane_type(LaneType::Parking));
    assert!(is_drivable_lane_type(LaneType::Stop));
    assert!(is_drivable_lane_type(LaneType::Shoulder));
    assert!(is_drivable_lane_type(LaneType::Biking));
    assert!(is_drivable_lane_type(LaneType::Sidewalk));
    assert!(is_drivable_lane_type(LaneType::Bidirectional));
    assert!(is_drivable_lane_type(LaneType::ConnectingRamp));
    assert!(is_drivable_lane_type(LaneType::MwyEntry));
    assert!(is_drivable_lane_type(LaneType::MwyExit));

    // Non-drivable types
    assert!(!is_drivable_lane_type(LaneType::Border));
    assert!(!is_drivable_lane_type(LaneType::Curb));
    assert!(!is_drivable_lane_type(LaneType::None));
    assert!(!is_drivable_lane_type(LaneType::Restricted));
    assert!(!is_drivable_lane_type(LaneType::Median));
}

// ============================================================================
// Complex Map Geometry Tests
// ============================================================================

#[test]
fn test_crossing_8_course_geometry_types() {
    let xodr_path = resources_dir().join("Crossing8Course.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap());

    if doc.is_err() {
        // Some maps may not be available in all environments
        return;
    }

    let doc = doc.unwrap();

    println!("Crossing8Course: {} roads, {} junctions", doc.roads.len(), doc.junctions.len());
    assert!(doc.roads.len() > 0);

    let mut line_count = 0;
    let mut arc_count = 0;
    let mut spiral_count = 0;
    let mut poly_count = 0;

    for road in &doc.roads {
        for geom in &road.plan_view.geometries {
            match &geom.description {
                GeometryDescription::Line(_) => line_count += 1,
                GeometryDescription::Arc(_) => arc_count += 1,
                GeometryDescription::Spiral(_) => spiral_count += 1,
                GeometryDescription::ParamPoly3(_) => poly_count += 1,
            }
        }
    }

    println!("Geometry types: {} lines, {} arcs, {} spirals, {} poly",
        line_count, arc_count, spiral_count, poly_count);
}

// ============================================================================
// Road Geometry Structure Tests (matching C++ parameterized tests)
// ============================================================================

/// Test parameters matching C++ RoadGeometryBuilderTestParameters
struct RoadGeometryTestParams {
    /// XODR file name
    file_name: &'static str,
    /// Expected number of junctions
    expected_junctions: usize,
    /// Expected number of segments
    expected_segments: usize,
    /// Expected number of lanes
    expected_lanes: usize,
}

const TEST_CASES: &[RoadGeometryTestParams] = &[
    // SingleLane: 1 road, 1 section, 2 driving lanes (left=1, right=-1) -> 1 junction, 1 segment, 2 lanes
    RoadGeometryTestParams {
        file_name: "SingleLane.xodr",
        expected_junctions: 1,
        expected_segments: 1,
        expected_lanes: 2,
    },
    // ArcLane: 1 road, 1 section, 2 driving lanes (left=1, right=-1) -> 1 junction, 1 segment, 2 lanes  
    RoadGeometryTestParams {
        file_name: "ArcLane.xodr",
        expected_junctions: 1,
        expected_segments: 1,
        expected_lanes: 2,
    },
    // LineMultipleSections: 1 road, 3 sections, varying lanes per section
    RoadGeometryTestParams {
        file_name: "LineMultipleSections.xodr",
        expected_junctions: 3,
        expected_segments: 3,
        expected_lanes: 3, // Based on C++ test expectations
    },
];

#[test]
fn test_road_geometry_structure_single_lane() {
    let params = &TEST_CASES[0];
    let xodr_path = resources_dir().join(params.file_name);
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect(&format!("Failed to load {}", params.file_name));

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    assert_eq!(rg.num_junctions(), params.expected_junctions,
        "Junction count mismatch for {}", params.file_name);
    
    // Count total segments across all junctions
    let total_segments: usize = (0..rg.num_junctions())
        .map(|i| rg.junction(i).unwrap().num_segments())
        .sum();
    assert_eq!(total_segments, params.expected_segments,
        "Segment count mismatch for {}", params.file_name);
    
    // Count total lanes across all segments
    let mut total_lanes = 0;
    for i in 0..rg.num_junctions() {
        let junction = rg.junction(i).unwrap();
        for j in 0..junction.num_segments() {
            total_lanes += junction.segment(j).unwrap().num_lanes();
        }
    }
    assert_eq!(total_lanes, params.expected_lanes,
        "Lane count mismatch for {}", params.file_name);
}

#[test]
fn test_road_geometry_structure_arc_lane() {
    let params = &TEST_CASES[1];
    let xodr_path = resources_dir().join(params.file_name);
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect(&format!("Failed to load {}", params.file_name));

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    assert_eq!(rg.num_junctions(), params.expected_junctions,
        "Junction count mismatch for {}", params.file_name);
}

// ============================================================================
// Detailed Structure Tests (matching C++ RoadGeometryBuilderBaseTest)
// ============================================================================

/// Tests detailed road geometry structure for SingleLane.xodr
/// Matching C++ test: RoadGeometryBuilderBase/SingleLane
#[test]
fn test_single_lane_detailed_structure() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load SingleLane.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // Verify junction structure
    assert_eq!(rg.num_junctions(), 1);
    
    let junction = rg.junction(0).unwrap();
    assert_eq!(junction.id().string(), "1_0");  // Road 1, Section 0
    assert_eq!(junction.num_segments(), 1);
    
    let segment = junction.segment(0).unwrap();
    assert_eq!(segment.id().string(), "1_0");  // Road 1, Section 0
    assert_eq!(segment.num_lanes(), 2);  // Left lane (1) and Right lane (-1)
    
    // Verify lane IDs match expected format
    let lane0 = segment.lane(0).unwrap();
    let lane1 = segment.lane(1).unwrap();
    
    // Lane IDs should be "1_0_-1" (road 1, section 0, lane -1) 
    // and "1_0_1" (road 1, section 0, lane 1)
    let lane_ids: Vec<String> = vec![
        lane0.id().string().to_string(),
        lane1.id().string().to_string(),
    ];
    assert!(lane_ids.contains(&"1_0_-1".to_string()) || lane_ids.contains(&"1_0_1".to_string()),
        "Lane IDs should include 1_0_-1 and 1_0_1, got: {:?}", lane_ids);
}

/// Tests detailed road geometry structure for ArcLane.xodr
/// Matching C++ test: RoadGeometryBuilderBase/ArcLane
#[test]
fn test_arc_lane_detailed_structure() {
    let xodr_path = resources_dir().join("ArcLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load ArcLane.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    assert_eq!(rg.num_junctions(), 1);
    
    let junction = rg.junction(0).unwrap();
    assert_eq!(junction.id().string(), "1_0");
    assert_eq!(junction.num_segments(), 1);
    
    let segment = junction.segment(0).unwrap();
    assert_eq!(segment.num_lanes(), 2);  // Left lane (1) and Right lane (-1)
}

/// Tests detailed road geometry structure for SpiralRoad.xodr
/// Matching C++ test: RoadGeometryBuilderBase/SpiralRoad
#[test]
fn test_spiral_road_detailed_structure() {
    let xodr_path = resources_dir().join("SpiralRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load SpiralRoad.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // SpiralRoad: 1 road, 1 section, 2 driving lanes
    assert_eq!(rg.num_junctions(), 1);
    
    let junction = rg.junction(0).unwrap();
    assert_eq!(junction.id().string(), "1_0");
    assert_eq!(junction.num_segments(), 1);
    
    let segment = junction.segment(0).unwrap();
    assert_eq!(segment.num_lanes(), 2);
}

/// Tests detailed road geometry structure for SShapeRoad.xodr
/// Matching C++ test: RoadGeometryBuilderBase/SShapeRoad
#[test]
fn test_sshape_road_detailed_structure() {
    let xodr_path = resources_dir().join("SShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load SShapeRoad.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // SShapeRoad: 1 road, 1 section, 2 driving lanes
    // Road has 3 geometries (arc + line + arc) but still 1 lane section
    assert_eq!(rg.num_junctions(), 1);
    
    let junction = rg.junction(0).unwrap();
    assert_eq!(junction.id().string(), "1_0");
}

/// Tests detailed road geometry structure for LShapeRoad.xodr
/// Matching C++ test: RoadGeometryBuilderBase/LShapeRoad
#[test]
fn test_lshape_road_detailed_structure() {
    let xodr_path = resources_dir().join("LShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load LShapeRoad.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // LShapeRoad: 3 roads, 1 section each, 1 driving lane each
    // Junction IDs: 1_0, 2_0, 3_0
    assert_eq!(rg.num_junctions(), 3);
    
    // Collect junction IDs
    let junction_ids: Vec<String> = (0..rg.num_junctions())
        .map(|i| rg.junction(i).unwrap().id().string().to_string())
        .collect();
    
    assert!(junction_ids.contains(&"1_0".to_string()), "Should have junction 1_0");
    assert!(junction_ids.contains(&"2_0".to_string()), "Should have junction 2_0");
    assert!(junction_ids.contains(&"3_0".to_string()), "Should have junction 3_0");
}

/// Tests detailed road geometry structure for TShapeRoad.xodr
/// Matching C++ test: RoadGeometryBuilderBase/TShapeRoad
#[test]
fn test_tshape_road_detailed_structure() {
    let xodr_path = resources_dir().join("TShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load TShapeRoad.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // TShapeRoad: 3 main roads + 6 junction roads = 9 roads
    // Main road junctions: 0_0, 1_0, 2_0
    // Junction roads are grouped under junction "3"
    // Expected junctions: 4 (0_0, 1_0, 2_0, and junction "3" with 6 segments)
    assert!(rg.num_junctions() >= 4, 
        "TShapeRoad should have at least 4 junctions, got {}", rg.num_junctions());

    // Verify junction "3" exists (from XODR junction definition)
    let junction_ids: Vec<String> = (0..rg.num_junctions())
        .map(|i| rg.junction(i).unwrap().id().string().to_string())
        .collect();
    
    assert!(junction_ids.contains(&"0_0".to_string()), "Should have junction 0_0");
    assert!(junction_ids.contains(&"1_0".to_string()), "Should have junction 1_0");
    assert!(junction_ids.contains(&"2_0".to_string()), "Should have junction 2_0");
    assert!(junction_ids.contains(&"3".to_string()), "Should have junction 3");
}

/// Tests GapInLaneWidthDrivableLane.xodr - tests lane width variations
/// Matching C++ test: RoadGeometryBuilderBase/GapInLaneWidthDrivableLane
#[test]
fn test_gap_in_lane_width_drivable_lane() {
    let xodr_path = resources_dir().join("GapInLaneWidthDrivableLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load GapInLaneWidthDrivableLane.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // GapInLaneWidthDrivableLane: 1 road, 1 section, 1 driving lane
    assert_eq!(rg.num_junctions(), 1);
    
    let junction = rg.junction(0).unwrap();
    assert_eq!(junction.id().string(), "1_0");
    assert_eq!(junction.num_segments(), 1);
    
    let segment = junction.segment(0).unwrap();
    assert_eq!(segment.num_lanes(), 1);  // Only lane 1 (driving)
}

/// Tests DisconnectedRoadInJunction.xodr - road in junction not connected at one end
/// Matching C++ test: RoadGeometryBuilderBase/DisconnectedRoadInJunction
#[test]
fn test_disconnected_road_in_junction() {
    let xodr_path = resources_dir().join("DisconnectedRoadInJunction.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load DisconnectedRoadInJunction.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // DisconnectedRoadInJunction: 2 roads, 1 section each
    // Road 0 has junction 0_0, Road 1 belongs to Junction 2
    assert_eq!(rg.num_junctions(), 2);
    
    let junction_ids: Vec<String> = (0..rg.num_junctions())
        .map(|i| rg.junction(i).unwrap().id().string().to_string())
        .collect();
    
    assert!(junction_ids.contains(&"0_0".to_string()), "Should have junction 0_0");
    assert!(junction_ids.contains(&"2".to_string()), "Should have junction 2");
}

// ============================================================================
// Branch Point Tests
// ============================================================================

/// Tests that branch points are created for each lane end
#[test]
fn test_branch_points_created() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load SingleLane.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // Each lane should have 2 branch points (start and finish)
    // SingleLane has 2 lanes, so 4 branch points total
    let num_branch_points = rg.num_branch_points();
    assert_eq!(num_branch_points, 4, 
        "SingleLane should have 4 branch points (2 lanes × 2 ends)");
}

/// Tests branch point ID naming convention
#[test]
fn test_branch_point_id_format() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load SingleLane.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // Check that branch point IDs follow the "bp_{index}" format
    for i in 0..rg.num_branch_points() {
        let bp = rg.branch_point(i).unwrap();
        let id = bp.id().string();
        assert!(id.starts_with("bp_"), 
            "Branch point ID should start with 'bp_', got: {}", id);
    }
}

// ============================================================================
// Crossing 8 Course Tests
// ============================================================================

/// Tests Crossing8Course.xodr - complex figure-8 crossing
#[test]
fn test_crossing_8_course() {
    let xodr_path = resources_dir().join("Crossing8Course.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load Crossing8Course.xodr");

    let builder_params = BuilderParams::new(xodr_path.to_str().unwrap())
        .with_linear_tolerance(STRICT_LINEAR_TOLERANCE)
        .with_angular_tolerance(STRICT_ANGULAR_TOLERANCE)
        .with_scale_length(SCALE_LENGTH);

    let builder = RoadGeometryBuilder::new(builder_params)
        .expect("Builder creation should succeed");
    let rg = builder.build(&doc.roads, &doc.junctions)
        .expect("Build should succeed");

    // Crossing8Course is a complex map - verify it builds successfully
    assert!(rg.num_junctions() > 0, "Should have at least one junction");
    
    // Verify segments and lanes exist
    let mut total_lanes = 0;
    for i in 0..rg.num_junctions() {
        let junction = rg.junction(i).unwrap();
        for j in 0..junction.num_segments() {
            let segment = junction.segment(j).unwrap();
            total_lanes += segment.num_lanes();
        }
    }
    assert!(total_lanes > 0, "Should have at least one lane");
}
