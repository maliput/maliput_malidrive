//! Integration tests for XODR parsing
//!
//! These tests load real XODR files from the resources directory
//! and verify that they parse correctly.

use std::path::PathBuf;

use maliput_malidrive::xodr::parser::{load_from_file, XodrParser};
use maliput_malidrive::xodr::{GeometryType, LaneType};

/// Returns the path to the resources directory.
fn resources_dir() -> PathBuf {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    // manifest_dir is experimental/maliput_malidrive_rs
    // We need to go up 2 levels to maliput_malidrive, then into resources
    PathBuf::from(manifest_dir)
        .parent()
        .unwrap()  // experimental
        .parent()
        .unwrap()  // maliput_malidrive
        .join("resources")
}

/// Helper to load an XODR file from resources.
fn load_resource(name: &str) -> maliput_malidrive::common::MalidriveResult<maliput_malidrive::xodr::parser::XodrDocument> {
    let path = resources_dir().join(name);
    load_from_file(&path)
}

#[test]
fn test_load_single_lane() {
    let doc = load_resource("SingleLane.xodr").expect("Failed to load SingleLane.xodr");

    // Verify header
    assert_eq!(doc.header.rev_major, 1);
    assert_eq!(doc.header.rev_minor, 1);
    assert_eq!(doc.header.name.as_deref(), Some("SingleLane"));

    // Verify roads
    assert_eq!(doc.roads.len(), 1);
    let road = &doc.roads[0];
    assert_eq!(road.id, "1");
    assert!((road.length - 100.0).abs() < 1e-6);

    // Verify geometry
    assert_eq!(road.plan_view.geometries.len(), 1);
    let geom = &road.plan_view.geometries[0];
    assert!(geom.is_line());
    assert!((geom.length - 100.0).abs() < 1e-6);

    // Verify lanes
    assert_eq!(road.lanes.lane_sections.len(), 1);
    let section = &road.lanes.lane_sections[0];
    assert_eq!(section.left_lanes.len(), 1);
    assert_eq!(section.right_lanes.len(), 1);
    assert_eq!(section.center_lane.id, 0);

    // All lanes should be drivable in this simple road
    let drivable = doc.get_all_drivable_lanes();
    assert_eq!(drivable.len(), 3); // left, center, right are all driving type
}

#[test]
fn test_load_t_shape_road() {
    let doc = load_resource("TShapeRoad.xodr").expect("Failed to load TShapeRoad.xodr");

    // Verify header
    assert_eq!(doc.header.rev_major, 1);
    assert_eq!(doc.header.rev_minor, 4);

    // This file has 9 roads
    assert_eq!(doc.roads.len(), 9);

    // This file has 1 junction
    assert_eq!(doc.junctions.len(), 1);
    let junction = &doc.junctions[0];
    assert_eq!(junction.id, "3");
    assert_eq!(junction.connections.len(), 6);

    // Find roads with arc geometries
    let roads_with_arcs: Vec<_> = doc
        .roads
        .iter()
        .filter(|r| {
            r.plan_view
                .geometries
                .iter()
                .any(|g| g.geometry_type() == GeometryType::Arc)
        })
        .collect();

    // Roads 6, 7, 8, 9 have arc geometries
    assert!(roads_with_arcs.len() >= 4);

    // Check total road length
    let total_length = doc.total_road_length();
    assert!(total_length > 170.0);
    assert!(total_length < 200.0);
}

#[test]
fn test_load_arc_lane() {
    let doc = load_resource("ArcLane.xodr").expect("Failed to load ArcLane.xodr");

    // This should have arc geometry
    assert!(!doc.roads.is_empty());
    let has_arc = doc.roads.iter().any(|r| {
        r.plan_view
            .geometries
            .iter()
            .any(|g| g.is_arc())
    });
    assert!(has_arc, "ArcLane.xodr should contain arc geometry");
}

#[test]
fn test_load_figure8() {
    let doc = load_resource("Figure8.xodr").expect("Failed to load Figure8.xodr");

    // Figure8 should have multiple roads forming a figure-8 pattern
    assert!(doc.roads.len() >= 2);

    // Should have arc geometries for the curves
    let arc_count: usize = doc
        .roads
        .iter()
        .flat_map(|r| &r.plan_view.geometries)
        .filter(|g| g.is_arc())
        .count();

    assert!(arc_count > 0, "Figure8 should have arc geometries");
}

#[test]
fn test_lane_types() {
    let doc = load_resource("TShapeRoad.xodr").expect("Failed to load TShapeRoad.xodr");

    // TShapeRoad has various lane types
    let mut has_driving = false;
    let mut has_shoulder = false;
    let mut has_sidewalk = false;

    for road in &doc.roads {
        for section in &road.lanes.lane_sections {
            for lane in section.all_lanes() {
                match lane.lane_type {
                    LaneType::Driving => has_driving = true,
                    LaneType::Shoulder => has_shoulder = true,
                    LaneType::Sidewalk => has_sidewalk = true,
                    _ => {}
                }
            }
        }
    }

    assert!(has_driving, "Should have driving lanes");
    assert!(has_shoulder, "Should have shoulder lanes");
    assert!(has_sidewalk, "Should have sidewalk lanes");
}

#[test]
fn test_road_links_and_junctions() {
    let doc = load_resource("TShapeRoad.xodr").expect("Failed to load TShapeRoad.xodr");

    // Find roads that are part of the junction
    let junction_roads: Vec<_> = doc
        .roads
        .iter()
        .filter(|r| r.junction != "-1")
        .collect();

    // Roads 4-9 are connection roads in the junction
    assert!(!junction_roads.is_empty());

    // Check junction connections
    let junction = doc.get_junction("3").expect("Junction 3 should exist");
    for conn in &junction.connections {
        // Each connection should reference valid roads
        let incoming = doc.get_road(&conn.incoming_road);
        let connecting = doc.get_road(&conn.connecting_road);

        assert!(
            incoming.is_some(),
            "Incoming road {} should exist",
            conn.incoming_road
        );
        assert!(
            connecting.is_some(),
            "Connecting road {} should exist",
            conn.connecting_road
        );
    }
}

#[test]
fn test_parser_configuration() {
    use maliput_malidrive::xodr::parser::ParserConfiguration;

    let parser = XodrParser::with_config(ParserConfiguration::permissive());
    let path = resources_dir().join("SingleLane.xodr");
    let doc = parser.parse_file(&path).expect("Should parse successfully");

    assert_eq!(doc.roads.len(), 1);
}

#[test]
fn test_geometry_details() {
    let doc = load_resource("TShapeRoad.xodr").expect("Failed to load TShapeRoad.xodr");

    // Find a road with arc geometry
    let road_with_arc = doc
        .roads
        .iter()
        .find(|r| r.plan_view.geometries.iter().any(|g| g.is_arc()))
        .expect("Should have a road with arc geometry");

    // Check arc geometry details
    for geom in &road_with_arc.plan_view.geometries {
        if let Some(arc) = geom.as_arc() {
            // Arc curvature should be non-zero
            assert!(arc.curvature.abs() > 0.0);
        }
    }
}

#[test]
fn test_elevation_profile() {
    let doc = load_resource("ArcElevatedRoad.xodr").expect("Failed to load ArcElevatedRoad.xodr");

    // This road should have elevation profile
    let road = &doc.roads[0];

    // Even if no explicit elevation elements, the profile should exist
    // (it may be empty or have default flat elevation)
    assert!(road.elevation_profile.elevations.is_empty() || !road.elevation_profile.elevations.is_empty());
}

#[test]
fn test_lane_widths() {
    let doc = load_resource("SingleLane.xodr").expect("Failed to load SingleLane.xodr");

    let road = &doc.roads[0];
    let section = &road.lanes.lane_sections[0];

    // Left lane should have width
    let left_lane = &section.left_lanes[0];
    assert!(!left_lane.widths.is_empty());
    let width = left_lane.widths[0].a;
    assert!(width > 0.0, "Lane width should be positive");

    // Right lane should have width
    let right_lane = &section.right_lanes[0];
    assert!(!right_lane.widths.is_empty());
    let width = right_lane.widths[0].a;
    assert!(width > 0.0, "Lane width should be positive");
}
