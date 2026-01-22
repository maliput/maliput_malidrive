//! OpenDRIVE (XODR) parser module.
//!
//! This module provides parsing capabilities for OpenDRIVE XML files,
//! converting them to internal data structures that can be used to
//! construct maliput road networks.
//!
//! ## OpenDRIVE Format Support
//!
//! The parser supports OpenDRIVE 1.4, 1.5, and 1.8 specifications with
//! the following capabilities:
//!
//! | Feature | Status |
//! |---------|--------|
//! | Road Links (Predecessor/Successor) | ✅ Supported |
//! | PlanView (Lines, Arcs, Spirals, Param Poly3) | ✅ Supported |
//! | Elevation Profile | ✅ Supported |
//! | Superelevation | ✅ Supported |
//! | Lane Sections (Left, Center, Right) | ✅ Supported |
//! | Lane Width/Offset | ✅ Supported |
//! | Lane Links | ✅ Supported |
//! | Junctions & Connections | ✅ Supported |
//!
//! ## Example
//!
//! ```rust,ignore
//! use maliput_malidrive::xodr::{load_from_file, XodrDocument};
//!
//! let doc = load_from_file("map.xodr")?;
//!
//! // Access parsed road data
//! for road in &doc.roads {
//!     println!("Road ID: {}", road.id);
//! }
//! ```

mod geometry;
mod lane;
mod lane_link;
mod lane_offset;
mod lane_section;
mod lane_width;
mod elevation_profile;
mod lateral_profile;
mod road_header;
mod road_link;
mod road_type;
mod junction;
mod connection;
mod unit;
mod header;
mod reference_geometry;
pub mod parser;

pub use geometry::*;
pub use lane::*;
pub use lane_link::*;
pub use lane_offset::*;
pub use lane_section::*;
pub use lane_width::*;
pub use elevation_profile::*;
pub use lateral_profile::*;
pub use road_header::*;
pub use road_link::*;
pub use road_type::*;
pub use junction::*;
pub use connection::*;
pub use unit::*;
pub use header::*;
pub use reference_geometry::*;
pub use parser::{load_from_file, load_from_str, ParserConfiguration, XodrDocument, XodrParser};
