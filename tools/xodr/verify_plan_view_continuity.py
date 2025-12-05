#!/usr/bin/env python3
"""
Verify C0 and C1 continuity of geometries in OpenDRIVE files.

C0 continuity: Position continuity - the end point of one geometry matches
               the start point of the next geometry.
C1 continuity: Tangent continuity - the heading (direction) at the end of one
               geometry matches the heading at the start of the next geometry.

Supported geometry types:
- line
- arc
- paramPoly3
- spiral (Euler spiral / clothoid)

Usage:
    python verify_plan_view_continuity.py <opendrive_file.xodr> [--tolerance <value>] [--angular-tolerance <value>]
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def angular_difference(angle1: float, angle2: float) -> float:
    """
    Compute the smallest angular difference between two angles.
    Returns a value in [0, pi].
    """
    diff = normalize_angle(angle1 - angle2)
    return abs(diff)


@dataclass
class GeometryBase(ABC):
    """Base class for all geometry types."""
    s: float  # s-coordinate along the road
    x: float  # x-coordinate of the start point
    y: float  # y-coordinate of the start point
    hdg: float  # heading at the start point
    length: float  # length of the geometry
    geometry_type: str  # Type name for display

    @abstractmethod
    def get_end_point(self) -> Tuple[float, float]:
        """Get the end point in global coordinates."""
        pass

    @abstractmethod
    def get_end_heading(self) -> float:
        """Get the heading at the end point."""
        pass

    def get_start_point(self) -> Tuple[float, float]:
        """Get the start point in global coordinates."""
        return (self.x, self.y)

    def get_start_heading(self) -> float:
        """Get the heading at the start point."""
        return normalize_angle(self.hdg)


@dataclass
class LineGeometry(GeometryBase):
    """Represents a line geometry element."""

    def __post_init__(self):
        self.geometry_type = "line"

    def get_end_point(self) -> Tuple[float, float]:
        """Get the end point in global coordinates."""
        x_end = self.x + self.length * math.cos(self.hdg)
        y_end = self.y + self.length * math.sin(self.hdg)
        return (x_end, y_end)

    def get_end_heading(self) -> float:
        """Get the heading at the end point (same as start for a line)."""
        return normalize_angle(self.hdg)


@dataclass
class ArcGeometry(GeometryBase):
    """Represents an arc geometry element."""
    curvature: float = 0.0  # 1/radius, positive = left turn, negative = right turn

    def __post_init__(self):
        self.geometry_type = "arc"

    def get_end_point(self) -> Tuple[float, float]:
        """
        Get the end point in global coordinates.
        For an arc: x(s) = x0 + integral(cos(hdg + curvature*t), t=0..s)
                    y(s) = y0 + integral(sin(hdg + curvature*t), t=0..s)
        """
        if abs(self.curvature) < 1e-15:
            # Essentially a straight line
            x_end = self.x + self.length * math.cos(self.hdg)
            y_end = self.y + self.length * math.sin(self.hdg)
        else:
            radius = 1.0 / self.curvature
            # Arc center is perpendicular to heading
            # For positive curvature (left turn), center is to the left
            cx = self.x - radius * math.sin(self.hdg)
            cy = self.y + radius * math.cos(self.hdg)

            # End angle on the arc
            theta_end = self.hdg + self.curvature * self.length

            # End point
            x_end = cx + radius * math.sin(theta_end)
            y_end = cy - radius * math.cos(theta_end)

        return (x_end, y_end)

    def get_end_heading(self) -> float:
        """Get the heading at the end point."""
        # Heading changes linearly with arc length
        end_heading = self.hdg + self.curvature * self.length
        return normalize_angle(end_heading)


@dataclass
class ParamPoly3Geometry(GeometryBase):
    """Represents a paramPoly3 geometry element."""
    # Polynomial coefficients for U (local x)
    aU: float = 0.0
    bU: float = 0.0
    cU: float = 0.0
    dU: float = 0.0
    # Polynomial coefficients for V (local y)
    aV: float = 0.0
    bV: float = 0.0
    cV: float = 0.0
    dV: float = 0.0
    pRange: str = "normalized"  # "normalized" or "arcLength"

    def __post_init__(self):
        self.geometry_type = "paramPoly3"

    def get_p_max(self) -> float:
        """Get the maximum value of parameter p."""
        if self.pRange == "normalized":
            return 1.0
        else:  # arcLength
            return self.length

    def evaluate_local(self, p: float) -> Tuple[float, float]:
        """
        Evaluate the paramPoly3 at parameter p in local coordinates.
        Returns (u, v) in local frame.
        """
        u = self.aU + self.bU * p + self.cU * p**2 + self.dU * p**3
        v = self.aV + self.bV * p + self.cV * p**2 + self.dV * p**3
        return u, v

    def evaluate_local_derivative(self, p: float) -> Tuple[float, float]:
        """
        Evaluate the derivative of paramPoly3 at parameter p in local coordinates.
        Returns (du/dp, dv/dp) in local frame.
        """
        du_dp = self.bU + 2 * self.cU * p + 3 * self.dU * p**2
        dv_dp = self.bV + 2 * self.cV * p + 3 * self.dV * p**2
        return du_dp, dv_dp

    def local_to_global(self, u: float, v: float) -> Tuple[float, float]:
        """
        Transform local coordinates (u, v) to global coordinates (x, y).
        """
        cos_hdg = math.cos(self.hdg)
        sin_hdg = math.sin(self.hdg)
        x_global = self.x + u * cos_hdg - v * sin_hdg
        y_global = self.y + u * sin_hdg + v * cos_hdg
        return x_global, y_global

    def evaluate_global(self, p: float) -> Tuple[float, float]:
        """
        Evaluate the paramPoly3 at parameter p in global coordinates.
        Returns (x, y) in global frame.
        """
        u, v = self.evaluate_local(p)
        return self.local_to_global(u, v)

    def evaluate_global_heading(self, p: float) -> float:
        """
        Evaluate the heading at parameter p in global coordinates.
        Returns the heading angle in radians.
        """
        du_dp, dv_dp = self.evaluate_local_derivative(p)
        # Local heading
        local_heading = math.atan2(dv_dp, du_dp)
        # Global heading
        global_heading = self.hdg + local_heading
        return normalize_angle(global_heading)

    def get_start_point(self) -> Tuple[float, float]:
        """Get the start point in global coordinates."""
        return self.evaluate_global(0.0)

    def get_end_point(self) -> Tuple[float, float]:
        """Get the end point in global coordinates."""
        return self.evaluate_global(self.get_p_max())

    def get_start_heading(self) -> float:
        """Get the heading at the start point."""
        return self.evaluate_global_heading(0.0)

    def get_end_heading(self) -> float:
        """Get the heading at the end point."""
        return self.evaluate_global_heading(self.get_p_max())


@dataclass
class SpiralGeometry(GeometryBase):
    """
    Represents a spiral (Euler spiral / clothoid) geometry element.
    Note: Full spiral support is not yet implemented.
    """
    curvStart: float = 0.0  # Curvature at start
    curvEnd: float = 0.0  # Curvature at end

    def __post_init__(self):
        self.geometry_type = "spiral"

    def get_end_point(self) -> Tuple[float, float]:
        """
        Get the end point in global coordinates.
        WARNING: This is an approximation. Full spiral support not yet implemented.
        """
        # For now, use a simple linear interpolation of curvature
        # This is an approximation using numerical integration
        return self._integrate_spiral()

    def get_end_heading(self) -> float:
        """
        Get the heading at the end point.
        For a spiral, the heading change is: integral of curvature(s) ds
        With linear curvature: k(s) = k0 + (k1-k0)*s/L
        Integral: hdg_end = hdg_start + k0*L + (k1-k0)*L/2 = hdg_start + (k0+k1)*L/2
        """
        avg_curvature = (self.curvStart + self.curvEnd) / 2.0
        end_heading = self.hdg + avg_curvature * self.length
        return normalize_angle(end_heading)

    def _integrate_spiral(self, num_steps: int = 1000) -> Tuple[float, float]:
        """
        Numerically integrate the spiral to find the end point.
        Uses simple Euler integration.
        """
        x, y = self.x, self.y
        hdg = self.hdg
        ds = self.length / num_steps
        curvature_rate = (self.curvEnd - self.curvStart) / self.length if self.length > 0 else 0

        for i in range(num_steps):
            s = i * ds
            k = self.curvStart + curvature_rate * s
            x += ds * math.cos(hdg)
            y += ds * math.sin(hdg)
            hdg += k * ds

        return (x, y)


def parse_geometries(xodr_file: str) -> Tuple[dict, List[str]]:
    """
    Parse an OpenDRIVE file and extract all geometries.
    Returns a tuple of:
        - Dictionary mapping road_id to list of geometry objects
        - List of warning messages (e.g., for unsupported geometry types)
    """
    tree = ET.parse(xodr_file)
    root = tree.getroot()

    roads_geometries = {}
    warnings = []
    spiral_warning_shown = False

    for road in root.findall('.//road'):
        road_id = road.get('id')
        geometries = []

        plan_view = road.find('planView')
        if plan_view is None:
            continue

        for geometry in plan_view.findall('geometry'):
            s = float(geometry.get('s'))
            x = float(geometry.get('x'))
            y = float(geometry.get('y'))
            hdg = float(geometry.get('hdg'))
            length = float(geometry.get('length'))

            # Check for line geometry
            line_elem = geometry.find('line')
            if line_elem is not None:
                geom = LineGeometry(
                    s=s, x=x, y=y, hdg=hdg, length=length,
                    geometry_type="line"
                )
                geometries.append(geom)
                continue

            # Check for arc geometry
            arc_elem = geometry.find('arc')
            if arc_elem is not None:
                curvature = float(arc_elem.get('curvature'))
                geom = ArcGeometry(
                    s=s, x=x, y=y, hdg=hdg, length=length,
                    geometry_type="arc",
                    curvature=curvature
                )
                geometries.append(geom)
                continue

            # Check for paramPoly3 geometry
            param_poly3 = geometry.find('paramPoly3')
            if param_poly3 is not None:
                geom = ParamPoly3Geometry(
                    s=s, x=x, y=y, hdg=hdg, length=length,
                    geometry_type="paramPoly3",
                    aU=float(param_poly3.get('aU')),
                    bU=float(param_poly3.get('bU')),
                    cU=float(param_poly3.get('cU')),
                    dU=float(param_poly3.get('dU')),
                    aV=float(param_poly3.get('aV')),
                    bV=float(param_poly3.get('bV')),
                    cV=float(param_poly3.get('cV')),
                    dV=float(param_poly3.get('dV')),
                    pRange=param_poly3.get('pRange', 'normalized')
                )
                geometries.append(geom)
                continue

            # Check for spiral geometry
            spiral_elem = geometry.find('spiral')
            if spiral_elem is not None:
                if not spiral_warning_shown:
                    warnings.append(
                        "WARNING: Spiral (Euler spiral / clothoid) geometries found. "
                        "Full spiral support is not yet implemented. "
                        "Using numerical approximation for continuity checks."
                    )
                    spiral_warning_shown = True

                geom = SpiralGeometry(
                    s=s, x=x, y=y, hdg=hdg, length=length,
                    geometry_type="spiral",
                    curvStart=float(spiral_elem.get('curvStart')),
                    curvEnd=float(spiral_elem.get('curvEnd'))
                )
                geometries.append(geom)
                continue

            # Unknown geometry type
            warnings.append(f"WARNING: Unknown geometry type at s={s} in road {road_id}")

        if geometries:
            # Sort by s-coordinate
            geometries.sort(key=lambda g: g.s)
            roads_geometries[road_id] = geometries

    return roads_geometries, warnings


@dataclass
class ContinuityResult:
    """Result of a continuity check between two geometries."""
    road_id: str
    geometry_index: int  # Index of the first geometry in the pair
    s_position: float  # s-coordinate at the transition between geometries
    c0_error: float  # Position error (distance)
    c1_error: float  # Angular error (radians)
    c0_passed: bool
    c1_passed: bool
    # Detailed info
    end_point: Tuple[float, float]
    start_point: Tuple[float, float]
    end_heading: float
    start_heading: float
    # Geometry types
    geom1_type: str
    geom2_type: str


def check_continuity(
    roads_geometries: dict,
    position_tolerance: float = 1e-3,
    angular_tolerance: float = 1e-3
) -> List[ContinuityResult]:
    """
    Check C0 and C1 continuity for all consecutive geometries.

    Args:
        roads_geometries: Dictionary mapping road_id to list of geometry objects.
        position_tolerance: Maximum allowed position error for C0 continuity (meters).
        angular_tolerance: Maximum allowed angular error for C1 continuity (radians).

    Returns:
        List of ContinuityResult objects.
    """
    results = []

    for road_id, geometries in roads_geometries.items():
        for i in range(len(geometries) - 1):
            geom1 = geometries[i]
            geom2 = geometries[i + 1]

            # Get end point and heading of first geometry
            end_point = geom1.get_end_point()
            end_heading = geom1.get_end_heading()

            # Get start point and heading of second geometry
            start_point = geom2.get_start_point()
            start_heading = geom2.get_start_heading()

            # Calculate errors
            c0_error = math.sqrt(
                (end_point[0] - start_point[0])**2 +
                (end_point[1] - start_point[1])**2
            )
            c1_error = angular_difference(end_heading, start_heading)

            result = ContinuityResult(
                road_id=road_id,
                geometry_index=i,
                s_position=geom2.s,
                c0_error=c0_error,
                c1_error=c1_error,
                c0_passed=c0_error <= position_tolerance,
                c1_passed=c1_error <= angular_tolerance,
                end_point=end_point,
                start_point=start_point,
                end_heading=end_heading,
                start_heading=start_heading,
                geom1_type=geom1.geometry_type,
                geom2_type=geom2.geometry_type
            )
            results.append(result)

    return results


def print_results(results: List[ContinuityResult], verbose: bool = True) -> Tuple[int, int]:
    """
    Print the continuity check results.

    Returns:
        Tuple of (c0_failures, c1_failures)
    """
    c0_failures = 0
    c1_failures = 0

    if not results:
        print("No geometry transitions found to check.")
        return 0, 0

    print("\n" + "=" * 80)
    print("CONTINUITY CHECK RESULTS")
    print("=" * 80)

    current_road = None
    for result in results:
        if result.road_id != current_road:
            current_road = result.road_id
            print(f"\n--- Road ID: {result.road_id} ---")

        status_c0 = "✓ PASS" if result.c0_passed else "✗ FAIL"
        status_c1 = "✓ PASS" if result.c1_passed else "✗ FAIL"

        if not result.c0_passed:
            c0_failures += 1
        if not result.c1_passed:
            c1_failures += 1

        geom_transition = f"{result.geom1_type} -> {result.geom2_type}"
        print(f"\n  Transition at s={result.s_position:.6f} (geom {result.geometry_index} -> {result.geometry_index + 1}) [{geom_transition}]:")
        print(f"    C0 (Position): {status_c0} - Error: {result.c0_error:.6e} m")
        print(f"    C1 (Heading):  {status_c1} - Error: {result.c1_error:.6e} rad ({math.degrees(result.c1_error):.4f}°)")

        if verbose and (not result.c0_passed or not result.c1_passed):
            print("    Details:")
            print(f"      End point:     ({result.end_point[0]:.6f}, {result.end_point[1]:.6f})")
            print(f"      Start point:   ({result.start_point[0]:.6f}, {result.start_point[1]:.6f})")
            print(f"      End heading:   {result.end_heading:.6f} rad ({math.degrees(result.end_heading):.4f}°)")
            print(f"      Start heading: {result.start_heading:.6f} rad ({math.degrees(result.start_heading):.4f}°)")

    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)
    total_transitions = len(results)
    print(f"Total transitions checked: {total_transitions}")
    print(f"C0 continuity: {total_transitions - c0_failures}/{total_transitions} passed ({c0_failures} failures)")
    print(f"C1 continuity: {total_transitions - c1_failures}/{total_transitions} passed ({c1_failures} failures)")

    if c0_failures == 0 and c1_failures == 0:
        print("\n✓ All continuity checks PASSED!")
    else:
        print(f"\n✗ {c0_failures + c1_failures} total failures detected.")

    return c0_failures, c1_failures


def main():
    parser = argparse.ArgumentParser(
        description="Verify C0 and C1 continuity of geometries in OpenDRIVE files."
    )
    parser.add_argument(
        "xodr_file",
        help="Path to the OpenDRIVE (.xodr) file to analyze."
    )
    parser.add_argument(
        "--tolerance", "-t",
        type=float,
        default=1e-3,
        help="Position tolerance for C0 continuity check in meters (default: 1e-3)"
    )
    parser.add_argument(
        "--angular-tolerance", "-a",
        type=float,
        default=1e-3,
        help="Angular tolerance for C1 continuity check in radians (default: 1e-3)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Show detailed information for failures"
    )
    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="Only show summary"
    )

    args = parser.parse_args()

    print(f"Analyzing: {args.xodr_file}")
    print(f"Position tolerance: {args.tolerance} m")
    print(f"Angular tolerance: {args.angular_tolerance} rad ({math.degrees(args.angular_tolerance):.4f}°)")

    try:
        roads_geometries, warnings = parse_geometries(args.xodr_file)
    except FileNotFoundError:
        print(f"Error: File '{args.xodr_file}' not found.")
        sys.exit(1)
    except ET.ParseError as e:
        print(f"Error: Failed to parse XML: {e}")
        sys.exit(1)

    # Print warnings
    for warning in warnings:
        print(f"\n{warning}")

    # Count geometry types
    geometry_counts = {}
    total_geometries = 0
    for geoms in roads_geometries.values():
        for geom in geoms:
            geometry_counts[geom.geometry_type] = geometry_counts.get(geom.geometry_type, 0) + 1
            total_geometries += 1

    print(f"\nFound {len(roads_geometries)} road(s) with {total_geometries} geometries:")
    for gtype, count in sorted(geometry_counts.items()):
        print(f"  - {gtype}: {count}")

    results = check_continuity(
        roads_geometries,
        position_tolerance=args.tolerance,
        angular_tolerance=args.angular_tolerance
    )

    if not args.quiet:
        c0_failures, c1_failures = print_results(results, verbose=args.verbose)
    else:
        c0_failures = sum(1 for r in results if not r.c0_passed)
        c1_failures = sum(1 for r in results if not r.c1_passed)
        total = len(results)
        print(f"\nC0: {total - c0_failures}/{total} passed | C1: {total - c1_failures}/{total} passed")

    # Return non-zero exit code if there are failures
    if c0_failures > 0 or c1_failures > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
