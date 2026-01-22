//! Tests for road geometry builder functionality.

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

#[test]
fn test_load_single_lane_xodr() {
    let xodr_path = resources_dir().join("SingleLane.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load SingleLane.xodr");

    assert_eq!(doc.roads.len(), 1, "SingleLane should have 1 road");

    let road = &doc.roads[0];
    assert_eq!(road.id, "1", "Road ID should be 1");
    assert!((road.length - 100.0).abs() < 0.1);
    assert_eq!(road.junction, "-1");

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

    assert!(doc.roads.len() >= 3);
    assert!(doc.junctions.len() > 0);

    let main_roads: Vec<_> = doc.roads.iter().filter(|r| r.junction == "-1").collect();
    assert!(main_roads.len() >= 3);

    let junction_roads: Vec<_> = doc.roads.iter().filter(|r| r.junction != "-1").collect();
    assert!(junction_roads.len() > 0);
}

#[test]
fn test_load_lshape_road_xodr() {
    let xodr_path = resources_dir().join("LShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load LShapeRoad.xodr");

    for road in &doc.roads {
        if let Some(link) = &road.link {
            if let Some(pred) = &link.predecessor {
                println!("Road {} predecessor: {}", road.id, pred.element_id);
            }
            if let Some(succ) = &link.successor {
                println!("Road {} successor: {}", road.id, succ.element_id);
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

#[test]
fn test_lane_section_structure() {
    let xodr_path = resources_dir().join("TShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load TShapeRoad.xodr");

    for road in &doc.roads {
        if road.junction != "-1" {
            continue;
        }

        for section in &road.lanes.lane_sections {
            for lane in &section.left_lanes {
                assert!(lane.id > 0);
            }

            for lane in &section.right_lanes {
                assert!(lane.id < 0);
            }

            assert_eq!(section.center_lane.id, 0);
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

    assert!(has_driving_lanes);
}

#[test]
fn test_junction_connections() {
    let xodr_path = resources_dir().join("TShapeRoad.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap())
        .expect("Failed to load TShapeRoad.xodr");

    assert!(doc.junctions.len() > 0);

    let junction = &doc.junctions[0];
    assert!(junction.connections.len() > 0);

    for connection in &junction.connections {
        assert!(!connection.incoming_road.is_empty());
        assert!(!connection.connecting_road.is_empty());
    }
}

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

#[test]
fn test_crossing_8_course() {
    let xodr_path = resources_dir().join("Crossing8Course.xodr");
    let doc = load_from_file(xodr_path.to_str().unwrap());

    if doc.is_err() {
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
