//! Road curve mathematical representations.
//!
//! This module provides mathematical representations of road geometry
//! including ground curves (2D reference lines) and 3D road curves
//! that combine ground curves with elevation and superelevation.
//!
//! ## Architecture
//!
//! ```text
//! RoadCurve
//!   ├── GroundCurve (2D reference line in XY plane)
//!   │     ├── LineGroundCurve (straight line)
//!   │     ├── ArcGroundCurve (circular arc)
//!   │     ├── SpiralGroundCurve (clothoid/Euler spiral)
//!   │     ├── ParamPoly3GroundCurve (parametric cubic polynomial)
//!   │     └── PiecewiseGroundCurve (composite curve)
//!   ├── Function (elevation profile)
//!   │     ├── CubicPolynomial
//!   │     ├── PiecewiseCubicPolynomial
//!   │     └── ConstantFunction
//!   ├── Function (superelevation)
//!   └── LaneOffset (lateral lane position)
//! ```

pub mod arc_ground_curve;
pub mod cubic_polynomial;
pub mod function;
pub mod ground_curve;
pub mod lane_offset;
pub mod line_ground_curve;
pub mod param_poly3_ground_curve;
pub mod piecewise_ground_curve;
pub mod road_curve;
pub mod road_curve_offset;
pub mod spiral_ground_curve;

// Export specific items to avoid ambiguity
pub use arc_ground_curve::ArcGroundCurve;
pub use cubic_polynomial::{CubicPolynomial, PiecewiseCubicPolynomial};
pub use function::{ConstantFunction, Function, LinearFunction};
pub use ground_curve::{validate_p, GroundCurve, GROUND_CURVE_EPSILON};
pub use lane_offset::{
    AdjacentLaneFunctions, LaneOffset, MalidriveLaneOffset, SimpleLaneOffset,
    AT_LEFT_FROM_CENTER_LANE, AT_RIGHT_FROM_CENTER_LANE,
};
pub use line_ground_curve::LineGroundCurve;
pub use param_poly3_ground_curve::{ParamPoly3GroundCurve, ParamPoly3Range};
pub use piecewise_ground_curve::PiecewiseGroundCurve;
pub use road_curve::RoadCurve;
pub use road_curve_offset::RoadCurveOffset;
pub use spiral_ground_curve::SpiralGroundCurve;
