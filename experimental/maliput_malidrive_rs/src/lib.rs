//! # maliput_malidrive - OpenDRIVE backend for maliput
//!
//! This crate provides a Rust implementation of the maliput_malidrive backend,
//! which constructs maliput road networks from OpenDRIVE (`.xodr`) map files.
//!
//! ## Overview
//!
//! maliput_malidrive is the most feature-complete backend for maliput, providing:
//! - Parsing of OpenDRIVE 1.4/1.5/1.8 format road networks
//! - Support for complex road geometries: lines, arcs, spirals, parametric cubic polynomials
//! - Handling of junctions, lane connections, and road topology
//! - Production-ready implementation for simulation and autonomous vehicle applications
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                    Your Application                         │
//! │         (uses maliput::api interfaces)                      │
//! └─────────────────────────────────────────────────────────────┘
//!                             │
//!                             ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │              maliput_malidrive (this crate)                 │
//! │  - OpenDRIVE parser (xodr/)                                 │
//! │  - Road geometry builder (builder/)                         │
//! │  - Lane/Segment/RoadGeometry implementations (base/)        │
//! │  - Road curve computations (road_curve/)                    │
//! └─────────────────────────────────────────────────────────────┘
//!                             │
//!                             ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │                  OpenDRIVE File (.xodr)                     │
//! │         (XML format describing road networks)               │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Modules
//!
//! - [`xodr`]: OpenDRIVE XML parser - converts `.xodr` files to internal data structures
//! - [`road_curve`]: Mathematical representations of road geometry (lines, arcs, spirals, polynomials)
//! - [`base`]: Concrete implementations of maliput API interfaces
//! - [`builder`]: Constructs maliput objects from parsed XODR data
//! - [`common`]: Common utilities and error types
//!
//! ## Usage
//!
//! ```rust,ignore
//! use maliput_malidrive::builder::RoadNetworkBuilder;
//! use maliput::api::RoadGeometry;
//!
//! // Build a road network from an OpenDRIVE file
//! let road_network = RoadNetworkBuilder::new()
//!     .with_opendrive_file("path/to/map.xodr")
//!     .with_linear_tolerance(1e-3)
//!     .with_angular_tolerance(1e-3)
//!     .build()?;
//!
//! // Use the maliput API to query the road network
//! let road_geometry = road_network.road_geometry();
//! let num_junctions = road_geometry.num_junctions();
//! ```

pub mod common;
pub mod road_curve;
pub mod xodr;
// TODO: base and builder modules are WIP - they need to be updated to match the maliput_rs API
// pub mod base;
// pub mod builder;

pub use common::*;
