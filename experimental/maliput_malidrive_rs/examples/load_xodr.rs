//! Example: Loading and querying an XODR file
//!
//! This example demonstrates how to load an OpenDRIVE (.xodr) file
//! and query its contents using the maliput_malidrive parser.
//!
//! Run with:
//! ```
//! cargo run --example load_xodr -- path/to/file.xodr
//! ```
//!
//! Or without arguments to load the default test file:
//! ```
//! cargo run --example load_xodr
//! ```

use std::env;
use std::path::Path;

use maliput_malidrive::xodr::parser::load_from_file;

fn main() {
    // Get the XODR file path from command line or use default
    let args: Vec<String> = env::args().collect();
    let xodr_path = if args.len() > 1 {
        args[1].clone()
    } else {
        // Default to SingleLane.xodr in the resources directory
        // manifest_dir is experimental/maliput_malidrive_rs
        // Go up 2 levels to maliput_malidrive, then into resources
        let manifest_dir = env!("CARGO_MANIFEST_DIR");
        let resources = Path::new(manifest_dir)
            .parent()
            .unwrap()  // experimental
            .parent()
            .unwrap()  // maliput_malidrive
            .join("resources");
        resources.join("SingleLane.xodr").to_string_lossy().to_string()
    };

    println!("Loading XODR file: {}", xodr_path);
    println!("---");

    // Check if file exists
    if !Path::new(&xodr_path).exists() {
        eprintln!("Error: File not found: {}", xodr_path);
        eprintln!();
        eprintln!("Usage: cargo run --example load_xodr -- <path_to_xodr_file>");
        eprintln!();
        eprintln!("Example XODR files can be found in:");
        eprintln!("  ../../../resources/SingleLane.xodr");
        eprintln!("  ../../../resources/TShapeRoad.xodr");
        eprintln!("  ../../../resources/Figure8.xodr");
        std::process::exit(1);
    }

    // Load and parse the XODR file
    let document = match load_from_file(&xodr_path) {
        Ok(doc) => doc,
        Err(e) => {
            eprintln!("Error parsing XODR file: {}", e);
            std::process::exit(1);
        }
    };

    // Print header information
    println!("=== XODR Document Header ===");
    println!(
        "OpenDRIVE Version: {}.{}",
        document.header.rev_major, document.header.rev_minor
    );
    if let Some(name) = &document.header.name {
        println!("Name: {}", name);
    }
    if let Some(version) = &document.header.version {
        println!("Version: {}", version);
    }
    if let Some(date) = &document.header.date {
        println!("Date: {}", date);
    }
    println!();

    // Print road summary
    println!("=== Roads ({}) ===", document.roads.len());
    for road in &document.roads {
        println!("Road ID: {}", road.id);
        if let Some(name) = &road.name {
            if !name.is_empty() {
                println!("  Name: {}", name);
            }
        }
        println!("  Length: {:.2} m", road.length);

        // Print plan view summary
        let num_geometries = road.plan_view.geometries.len();
        println!("  Plan View: {} geometry element(s)", num_geometries);
        for (i, geom) in road.plan_view.geometries.iter().enumerate() {
            println!(
                "    [{:2}] {} @ s={:.2}, length={:.2}, xy=({:.2}, {:.2}), hdg={:.4} rad",
                i,
                geom.geometry_type(),
                geom.s_0,
                geom.length,
                geom.start_point.x,
                geom.start_point.y,
                geom.orientation
            );
        }

        // Print elevation profile
        if !road.elevation_profile.elevations.is_empty() {
            println!(
                "  Elevation Profile: {} elevation(s)",
                road.elevation_profile.elevations.len()
            );
        }

        // Print superelevation profile
        if !road.lateral_profile.superelevations.is_empty() {
            println!(
                "  Lateral Profile: {} superelevation(s)",
                road.lateral_profile.superelevations.len()
            );
        }

        // Print lane sections
        println!(
            "  Lane Sections: {}",
            road.lanes.lane_sections.len()
        );
        for (i, section) in road.lanes.lane_sections.iter().enumerate() {
            println!(
                "    Section {} @ s={:.2}: {} left, {} right lanes",
                i,
                section.s,
                section.left_lanes.len(),
                section.right_lanes.len()
            );

            // Print individual lanes
            for lane in section.left_lanes.iter() {
                let width = lane
                    .widths
                    .first()
                    .map(|w| w.a)
                    .unwrap_or(0.0);
                println!(
                    "      Lane {} (left): type={}, width={:.2} m",
                    lane.id,
                    lane.lane_type,
                    width
                );
            }

            println!(
                "      Lane {} (center): type={}",
                section.center_lane.id, section.center_lane.lane_type
            );

            for lane in section.right_lanes.iter() {
                let width = lane
                    .widths
                    .first()
                    .map(|w| w.a)
                    .unwrap_or(0.0);
                println!(
                    "      Lane {} (right): type={}, width={:.2} m",
                    lane.id,
                    lane.lane_type,
                    width
                );
            }
        }
        println!();
    }

    // Print junction summary
    if !document.junctions.is_empty() {
        println!("=== Junctions ({}) ===", document.junctions.len());
        for junction in &document.junctions {
            println!("Junction ID: {}", junction.id);
            if let Some(name) = &junction.name {
                println!("  Name: {}", name);
            }
            println!("  Type: {:?}", junction.junction_type);
            println!("  Connections: {}", junction.connections.len());
            for conn in &junction.connections {
                println!(
                    "    {} -> {} via {} (contact: {:?})",
                    conn.incoming_road, conn.connecting_road, conn.id, conn.contact_point
                );
            }
        }
        println!();
    }

    // Print some statistics
    println!("=== Statistics ===");
    println!(
        "Total road length: {:.2} m",
        document.total_road_length()
    );

    let drivable_lanes = document.get_all_drivable_lanes();
    println!("Total drivable lanes: {}", drivable_lanes.len());

    // Print road-by-road drivable lanes
    for (road, section, lane) in &drivable_lanes {
        println!(
            "  Road {} section @ s={:.2}: lane {}",
            road.id, section.s, lane.id
        );
    }
}
