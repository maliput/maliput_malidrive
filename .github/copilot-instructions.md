# GitHub Copilot Onboarding Instructions for maliput_malidrive

This document provides coding agents with essential information to work efficiently on this repository without extensive exploration.

## Repository Overview

**maliput_malidrive** is a [maliput](https://github.com/maliput/maliput) backend implementation that provides road network construction from **OpenDRIVE** (`.xodr`) map files. It is the most feature-complete maliput backend for production use.

**Technologies:** C++17, CMake, Bazel, ROS 2 (ament), Google Test, Eigen3
**Documentation:** https://maliput.readthedocs.io/en/latest/html/deps/maliput_malidrive/html/index.html

### Key Capabilities

- Parses OpenDRIVE 1.4/1.5/1.8 format road networks
- Supports complex road geometries: lines, arcs, spirals, parametric cubic polynomials
- Handles junctions, lane connections, and road rules
- Provides traffic light, phase ring, and intersection book support via YAML configuration
- Production-ready for simulation and autonomous vehicle applications

## Architecture: maliput Backend

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Application                         │
│         (uses maliput::api interfaces)                      │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                     maliput (API)                           │
│  - Abstract API (RoadGeometry, Lane, Segment, Junction)     │
│  - Plugin system for loading backends                       │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              maliput_malidrive (this repo)                  │
│  - OpenDRIVE parser (xodr/)                                 │
│  - Road geometry builder (builder/)                         │
│  - Lane/Segment/RoadGeometry implementations (base/)        │
│  - Road curve computations (road_curve/)                    │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  OpenDRIVE File (.xodr)                     │
│         (XML format describing road networks)               │
└─────────────────────────────────────────────────────────────┘
```

### Usage Patterns

**Direct usage via builder:**
```cpp
#include <maliput/api/road_network.h>
#include <maliput_malidrive/builder/road_network_builder.h>

auto road_network = malidrive::builder::RoadNetworkBuilder({
    {"opendrive_file", "/path/to/map.xodr"},
    {"linear_tolerance", "1e-3"},
    {"angular_tolerance", "1e-3"},
})();

const maliput::api::RoadGeometry* rg = road_network->road_geometry();
```

**Plugin-based loading (preferred for applications):**
```cpp
#include <maliput/plugin/create_road_network.h>

auto road_network = maliput::plugin::CreateRoadNetwork(
    "maliput_malidrive",  // backend name
    {{"opendrive_file", "/path/to/map.xodr"}}
);
```

## Build System & Commands

### Prerequisites

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions
```

### CMake/Colcon Build (Recommended)

```bash
# Create workspace
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/maliput/maliput_malidrive.git

# Install dependencies
export ROS_DISTRO=foxy
rosdep update
rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src

# Build
colcon build --packages-up-to maliput_malidrive

# Build with documentation
colcon build --packages-select maliput_malidrive --cmake-args " -DBUILD_DOCS=On"
```

### Bazel Build

```bash
# Build all targets
bazel build //...

# Build specific targets
bazel build //:road_curve
bazel build //:xodr
bazel build //:builder
```

### Testing

```bash
# CMake/colcon
colcon test --packages-select maliput_malidrive
colcon test-result --verbose

# Bazel
bazel test //...
```

### Code Formatting

```bash
# Check format
ament_clang_format --config=./.clang-format

# Reformat code (if tools/reformat_code.sh exists)
./tools/reformat_code.sh
```

## Project Architecture

### Directory Structure

```
maliput_malidrive/
├── include/maliput_malidrive/           # Public headers
│   ├── builder/                         # RoadNetwork/RoadGeometry builders
│   │   ├── params.h                     # Builder configuration parameters
│   │   ├── road_network_builder.h       # Main builder interface
│   │   └── rule_tools.h                 # Rule-building utilities
│   ├── common/                          # Common utilities
│   │   └── macros.h                     # Error handling, copy/move macros
│   ├── loader/                          # High-level loader interface
│   │   └── loader.h
│   └── constants.h                      # Global constants
├── src/maliput_malidrive/               # Implementation files
│   ├── base/                            # maliput API implementations
│   │   ├── lane.cc/h                    # Lane implementation
│   │   ├── road_geometry.cc/h           # RoadGeometry implementation
│   │   └── segment.h                    # Segment implementation
│   ├── builder/                         # Builder implementations
│   │   ├── road_network_builder.cc      # Builds complete RoadNetwork
│   │   ├── road_geometry_builder.cc     # Builds RoadGeometry from XODR
│   │   ├── road_curve_factory.cc        # Creates road curves
│   │   └── ...
│   ├── road_curve/                      # Road geometry math
│   │   ├── ground_curve.h               # Base ground curve interface
│   │   ├── line_ground_curve.cc/h       # Straight line geometry
│   │   ├── arc_ground_curve.cc/h        # Arc geometry
│   │   ├── spiral_ground_curve.cc/h     # Clothoid/Euler spiral
│   │   ├── param_poly3_ground_curve.cc  # Parametric cubic polynomials
│   │   ├── road_curve.cc/h              # 3D road curve (ground + elevation)
│   │   └── ...
│   ├── xodr/                            # OpenDRIVE parser
│   │   ├── parser.cc/h                  # Main XODR parser
│   │   ├── db_manager.cc/h              # Parsed data manager
│   │   ├── geometry.cc/h                # Geometry parsing
│   │   ├── lane.cc/h                    # Lane parsing
│   │   ├── junction.cc/h                # Junction parsing
│   │   └── ...
│   ├── test_utilities/                  # Test helpers
│   └── common/                          # Internal utilities
├── src/applications/                    # CLI tools
│   ├── xodr_validate.cc                 # Validate XODR files
│   ├── xodr_query.cc                    # Query XODR contents
│   ├── xodr_extract.cc                  # Extract XODR portions
│   └── xodr_to_obj.cc                   # Convert to OBJ mesh
├── src/plugin/                          # maliput plugin registration
│   └── road_network.cc                  # Plugin loader implementation
├── resources/                           # Sample XODR maps and configs
│   ├── *.xodr                           # OpenDRIVE map files
│   ├── *.yaml                           # Rule/traffic light configs
│   └── ...
├── test/                                # Unit and integration tests
│   ├── regression/                      # Component tests
│   └── integration/                     # Full pipeline tests
├── cmake/                               # CMake configuration
├── bazel/                               # Bazel configuration
└── tutorials/                           # Usage tutorials
```

### Key Components

| Component | Description |
|-----------|-------------|
| `xodr/` | OpenDRIVE XML parser - converts `.xodr` files to internal data structures |
| `road_curve/` | Mathematical representations of road geometry (lines, arcs, spirals, polynomials) |
| `builder/` | Constructs maliput objects from parsed XODR data |
| `base/` | Concrete implementations of maliput API interfaces |
| `loader/` | High-level loading interface |
| `plugin/` | maliput plugin system integration |

### OpenDRIVE Parser Capabilities

The XODR parser supports (see `src/maliput_malidrive/xodr/README.md` for full details):

| Feature | Status |
|---------|--------|
| Road Links (Predecessor/Successor) | ✅ Supported |
| PlanView (Lines, Arcs, Spirals, Param Poly3) | ✅ Supported |
| Elevation Profile | ✅ Supported |
| Superelevation | ✅ Supported |
| Lane Sections (Left, Center, Right) | ✅ Supported |
| Lane Width/Offset | ✅ Supported |
| Lane Links | ✅ Supported |
| Lane Speed | ✅ Supported |
| Junctions & Connections | ✅ Supported |
| Georeferencing | ✅ Supported |
| Road Objects/Signals | ❌ Not supported |
| Crossfall/Road Shape | ❌ Not supported |

## CI/CD & Validation

### GitHub Workflows

Located in `.github/workflows/`:
- `build.yml` - Main CI: CMake + Bazel builds
- `build_macos.yaml` - macOS builds
- `sanitizers.yml` - Address/Thread sanitizer builds
- `scan_build.yml` - Static analysis
- `wheel_generation.yml` - Python wheel generation

### Validation Checklist

Before submitting changes:

1. **Format (REQUIRED):**
   ```bash
   ament_clang_format --config=./.clang-format
   ```

2. **Build & Test:**
   ```bash
   colcon build --packages-select maliput_malidrive
   colcon test --packages-select maliput_malidrive
   colcon test-result --verbose
   ```

## Code Style Guidelines

### C++ Style

Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with project-specific configurations.

**Key settings (from `.clang-format`):**
- **Line length:** 120 characters
- **Pointer alignment:** Left (`int* ptr` not `int *ptr`)
- **Include order:** Related header, C headers, C++ headers, library headers, project headers

### Error Handling

Use malidrive's error handling macros (which wrap maliput's macros):

```cpp
#include "maliput_malidrive/common/macros.h"

// Validate conditions with descriptive messages (PREFERRED)
MALIDRIVE_VALIDATE(condition, maliput::common::assertion_error, "Descriptive error message");
MALIDRIVE_VALIDATE(value > 0, maliput::common::assertion_error, "Value must be positive");

// Range validation
MALIDRIVE_IS_IN_RANGE(value, min_value, max_value);

// Throw with message
MALIDRIVE_THROW_MESSAGE("Something went wrong");

// Throw unless condition
MALIDRIVE_THROW_UNLESS(condition);

// Abort with message (for unrecoverable errors)
MALIDRIVE_ABORT_MSG("Fatal error");

// Demand (assertion that aborts)
MALIDRIVE_DEMAND(condition);
```

### Copy/Move Semantics

Use malidrive's macros to explicitly declare copy/move behavior:

```cpp
class MyClass {
 public:
  // Delete all copy/move operations
  MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(MyClass)

  // Or default all operations
  MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MyClass)
};
```

### Logging

Use maliput's logging system:

```cpp
#include <maliput/common/logger.h>

maliput::log()->trace("Trace message");
maliput::log()->debug("Debug message");
maliput::log()->info("Info message");
maliput::log()->warn("Warning message");
maliput::log()->error("Error message");

// Trace with file/line info
MALIDRIVE_TRACE("Trace message with location");
```

### Naming Conventions

- **Classes/Structs:** `PascalCase` (e.g., `RoadNetworkBuilder`, `GroundCurve`)
- **Functions/Methods:** `PascalCase` for public API (e.g., `ToInertialPosition()`)
- **Variables:** `snake_case` (e.g., `lane_position`, `road_geometry`)
- **Member variables:** `snake_case_` with trailing underscore
- **Constants:** `kPascalCase` (e.g., `kLinearTolerance`)
- **Namespaces:** `lowercase` (e.g., `malidrive::builder`)

### Header Guards

Use `#pragma once` (not traditional include guards).

### Documentation

Use Doxygen-style comments:

```cpp
/// Brief description of the function.
///
/// Detailed description if needed.
///
/// @param param_name Description of parameter.
/// @returns Description of return value.
/// @throws ExceptionType When this condition occurs.
```

## Testing Patterns

Tests use Google Test (gtest) and Google Mock (gmock):

```cpp
#include <gtest/gtest.h>
#include "maliput_malidrive/base/lane.h"

namespace malidrive {
namespace test {
namespace {

GTEST_TEST(LaneTest, DefaultBehavior) {
  // Test implementation
  // Use "dut" (Device Under Test) for the object being tested
  const auto dut = CreateTestLane();
  EXPECT_EQ(dut->id().string(), "expected_id");
}

}  // namespace
}  // namespace test
}  // namespace malidrive
```

Test files are located in `test/regression/` and `test/integration/` mirroring the source structure.

## Builder Configuration Parameters

Key parameters for `RoadNetworkBuilder` (see `include/maliput_malidrive/builder/params.h`):

| Parameter | Description | Default |
|-----------|-------------|---------|
| `opendrive_file` | Path to XODR file | Required |
| `road_geometry_id` | ID for the RoadGeometry | `"maliput"` |
| `linear_tolerance` | Linear tolerance (meters) | `5e-2` |
| `angular_tolerance` | Angular tolerance (radians) | `1e-3` |
| `scale_length` | Scale length | `1.0` |
| `inertial_to_backend_frame_translation` | Frame translation `{X,Y,Z}` | `{0,0,0}` |
| `build_policy` | `"sequential"` or `"parallel"` | `"sequential"` |
| `standard_strictness_policy` | `"strict"`, `"allow_schema_errors"`, `"allow_semantic_errors"`, `"permissive"` | `"permissive"` |
| `omit_nondrivable_lanes` | Skip non-drivable lanes | `"true"` |

## Resources

Sample XODR maps are provided in `resources/`:
- `SingleLane.xodr` - Simple single lane road
- `TShapeRoad.xodr` - T-junction example
- `Figure8.xodr` - Figure-8 track
- `Town01.xodr` - `Town07.xodr` - Complex town maps
- Many more for various road configurations

Environment variable `MALIPUT_MALIDRIVE_RESOURCE_ROOT` points to installed resources.

## Common Gotchas

1. **Include order matters:** Follow the order enforced by `.clang-format`
2. **Use `MALIDRIVE_VALIDATE` over `MALIDRIVE_THROW_UNLESS`:** Prefer descriptive error messages
3. **Namespace wrapping:** Always wrap code in `namespace malidrive { ... }`
4. **Virtual destructors:** Abstract classes should have virtual destructors
5. **const correctness:** Use `const` wherever possible
6. **XODR tolerance:** OpenDRIVE files may have geometric inconsistencies; use appropriate `standard_strictness_policy`

## Related Projects

- [maliput](https://github.com/maliput/maliput) - Core API (dependency)
- [maliput_multilane](https://github.com/maliput/maliput_multilane) - Procedural road backend
- [maliput_dragway](https://github.com/maliput/maliput_dragway) - Simple straight roads
- [maliput_osm](https://github.com/maliput/maliput_osm) - OpenStreetMap backend
- [maliput_integration](https://github.com/maliput/maliput_integration) - Integration examples

## Documentation Links

- **Main Documentation:** https://maliput.readthedocs.io/
- **API Reference:** https://maliput.readthedocs.io/en/latest/html/deps/maliput_malidrive/html/index.html
- **Tutorials:** https://maliput.readthedocs.io/en/latest/html/deps/maliput_malidrive/html/tutorials.html
- **XODR Parser Details:** See `src/maliput_malidrive/xodr/README.md`

## License

BSD 3-Clause License. See [LICENSE](../LICENSE) file.

---

**Trust these instructions.** Only perform additional exploration if information is missing or incorrect.
