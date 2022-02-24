![gcc](https://github.com/ToyotaResearchInstitute/maliput_malidrive/actions/workflows/build.yml/badge.svg)

# maliput_malidrive

A maliput backend.

An implementation of Maliput's API that allows users to
instantiate a RoadNetwork based on the OpenDRIVE format specification.

## OpenDRIVE format specification coverage

The following table keeps track on the capabilities of the `maliput_malidrive`'s XODR parser according to the 1.5 version of OpenDRIVE format specification.

Group | SubGroup | OpenDRIVE Record | Status
:-:|:-:|:-:|:-:
Road | Link | Predecessor/Successor | :green_circle:
Road | Link | Neighbor | :red_circle:
Road | Road Type | Type | :green_circle:
Road | Road Type | Country | :green_circle:
Road | Road Type | Speed | :green_circle:
Road | PlanView | Lines | :green_circle:
Road | PlanView | Arcs | :green_circle:
Road | PlanView | Spirals | :red_circle:
Road | PlanView | Cubic Polynomials | :red_circle:
Road | PlanView | Parametric Cubic Polynomials | :red_circle:
Road | ElevationProfile | Elevation | :green_circle:
Road | LateralProfile | Superelevation | :green_circle:
Road | LateralProfile | Crossfall | :red_circle:
Road | LateralProfile | Road Shape | :red_circle:
Road | Road Objects | Road Shape | :red_circle:
Road | Road Signals | Road Shape | :red_circle:
Road | Surface | OpenCRG | :red_circle:
Road | Railroad Elements | RailRoad Switches | :red_circle:
LaneSection | LaneOffset | Lane Offset | :green_circle:
LaneSection | Lane | Left | :green_circle:
LaneSection | Lane | Center | :green_circle:
LaneSection | Lane | Right | :green_circle:
LaneSection | Lane | LaneLink | :green_circle:
LaneSection | Lane | LaneWidth | :green_circle:
LaneSection | Lane | LaneBorder | :red_circle:
LaneSection | Lane | RoadMark | :red_circle:
LaneSection | Lane | LaneMaterial | :red_circle:
LaneSection | Lane | LaneVisibility | :red_circle:
LaneSection | Lane | LaneSpeed | :green_circle:
LaneSection | Lane | LaneAccess | :red_circle:
LaneSection | Lane | LaneRule | :red_circle:
Controller | ControlEntry | ControlEntry | :red_circle:
Junction | Connection | Predecessor | :green_circle:
Junction | Connection | Successor | :green_circle:
Junction | Connection | LaneLink | :green_circle:
Junction | Priority | Priority | :red_circle:
Junction | Controller | Controller | :red_circle:
Junction | Surface | Surface | :red_circle:
JunctionGroup | Junction Reference | Junction Reference | :red_circle:
Stations | Platform | Segment | :red_circle:
Ancillary Data | Lane | userData | :green_circle:
Include tag | Include file | include | :red_circle:
Alternative Layouts | Sets | set | :red_circle:
Data Quality Description | dataQuality | dataQuality | :red_circle:

## Build

1. Setup a maliput_malidrive (or a wider maliput) development workspace as described [here](https://github.com/ToyotaResearchInstitute/maliput_documentation/blob/main/docs/installation_quickstart.rst)

2. Bring up your development workspace:

```sh
cd path/to/my/workspace
source ./bringup
```

3. Build maliput_malidrive packages and their dependencies:

   ```sh
   colcon build --packages-up-to maliput_malidrive
   ```

   **Note**: To build documentation a `-BUILD_DOCS` cmake flag is required:
   ```sh
   colcon build --packages-up-to maliput_malidrive --cmake-args " -DBUILD_DOCS=On"
   ```

## Run sample applications

1. Bring up your development workspace:

```sh
cd path/to/my/workspace
source ./bringup
```

2. Source your build:

```sh
source ./install/setup.bash
```

And then run one of the sample applications. To fully inspect them, visit the
`applications` directory.

There are more applications for using `maliput_malidrive` backend.
See [maliput_integration](https://github.com/ToyotaResearchInstitute/maliput_integration) repository.

## Documentation

See [maliput_documentation](https://github.com/ToyotaResearchInstitute/maliput_documentation) repository for further documentation.
