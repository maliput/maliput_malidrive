//! Base implementations of maliput API interfaces.
//!
//! This module provides concrete implementations of the maliput API traits
//! using road curve geometry.

mod lane;
mod segment;
mod junction;
mod branch_point;
mod road_geometry;

pub use lane::*;
pub use segment::*;
pub use junction::*;
pub use branch_point::*;
pub use road_geometry::*;
