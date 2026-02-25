# Traffic Signal Database Parser

This directory contains a C++ parser and supporting utilities that read and validate YAML-based traffic signal type definitions to create maliput traffic signal objects such as `TrafficLight`s and `DiscreteValueRule`s.

## Overview

A traffic signal type definition describes the physical structure and control logic of a traffic signal. Each definition specifies:

- **Bulb structure**: Colors, types (round/arrow), states, dimensions, and positions
- **Bulbs**: A list of bulbs with specified orientation
- **Rule logic**: Mapping from bulb state combinations to Right-Of-Way Rule values

The parser in this directory reads YAML files containing these traffic signal type definitions, validates them, and creates the corresponding maliput `TrafficLight` and `DiscreteValueRule` objects.

## How It Works

When parsing an OpenDRIVE (`*.xodr`) road network file, traffic signals are referenced with a `type` attribute:

```xml
<signal type="1000001" subtype="-1" country="OpenDRIVE" s="10.5" t="2.0" zOffset="4.5" ... />
```

A parser uses this database to:

1. **Look up the signal type** (e.g., `1000001`) in the YAML database
2. **Create a maliput TrafficLight** object with bulbs from the type definition
3. **Create Right-Of-Way rules** mapping bulb state conditions to rule values (Go, Stop, etc.)
4. **Link rules to affected lanes** using the signal's validity information from the XODR file

## YAML Structure

Each YAML file in this directory should define a list of signal types under the top-level key `traffic_signal_types`. Each signal type entry contains:

### Signal Type Header

```yaml
- type: "1000001"               # Signal type identifier (must match XODR signal type)
  subtype: "-1"                 # Optional: signal subtype for finer matching
  country: "OpenDRIVE"          # Optional: country code or standard
  country_revision: null        # Optional: country standard revision
  description: "..."            # Human-readable description
```

### Bulbs

Each signal type contains a list of `bulbs`:

```yaml
  bulbs:                     # List of bulbs
    - id: "BulbName"
      position_traffic_light: [x, y, z]          # Position relative to traffic light frame
      orientation_traffic_light: [w, x, y, z]    # Orientation relative to traffic light frame
      color: "Red"           # One of: Red, Yellow, Green
      type: "Round"          # One of: Round, Arrow
      states: ["Off", "On", "Blinking"]  # Possible states for this bulb
      bounding_box: (optional)  # Custom bounding box if needed
        p_min: [x, y, z]
        p_max: [x, y, z]
      arrow_orientation_rad: (optional, required if type is Arrow)  # Arrow angle in radians
```

### Rule States

Rule states map bulb state combinations to Right-Of-Way Rule values:

```yaml
  rule_states:
    - condition:
        - bulb: "BulbId"
          state: "On"
        - bulb: "OtherBulbId"
          state: "Off"
      value: "Go"              # Rule value: Go, Stop, StopIfSafe, StopThenGo, ProceedWithCaution, or SignalMalfunctioning
```

## Coordinate Frames

All positions and orientations use right-handed coordinate systems with axes:

- **+X**: Direction the bulb is facing
- **+Y**: Left when facing the +X direction
- **+Z**: Up (gravity opposite)

### Frame Hierarchy

1. **Inertial/Road Network Frame**: Global reference frame for the entire road network
2. **Traffic Light Frame**: Origin at traffic light's center of mass; obtained from XODR signal position/orientation
3. **Bulb Frame**: Origin at bulb's center of mass; positioned/oriented relative to traffic light frame

## Bulb States

Three possible states for bulbs:

- **Off**: Bulb is not illuminated
- **On**: Bulb is fully illuminated
- **Blinking**: Bulb is flashing/blinking

Not all bulbs support all states. Define `states` array to specify which states are valid for each bulb.

## Right-Of-Way Rule Values

When bulb state conditions are met, the parser creates rules with one of these values:

| Value                  | Meaning                                                           |
| ---------------------- | ----------------------------------------------------------------- |
| `Go`                   | Proceed through the intersection                                  |
| `Stop`                 | Must stop before entering intersection                            |
| `StopIfSafe`           | Stop if safe to do so; may proceed if unsafe to stop              |
| `StopThenGo`           | Stop, then proceed when safe (e.g., blinking red)                 |
| `ProceedWithCaution`   | Proceed but watch for conflicting traffic (e.g., flashing yellow) |
| `SignalMalfunctioning` | Traffic signal is not functioning correctly                       |

## Examples

See `resources/traffic_signal_db/traffic_signal_db_example.yaml` for detailed examples including:

- Standard three-bulb vertical traffic light (type 1000001)
- Arrow-based traffic light (type 1000011)
- Pedestrian signal (type 1000002)

## Usage in Parsers

A parser loading these files should:

1. Load the YAML file
2. For each signal in the XODR file with type/subtype matching a database entry:
   - Create a `maliput::api::rules::TrafficLight` with bulbs
   - Create `maliput::api::rules::DiscreteValueRule` entries for each applicable lane
   - Map bulb state conditions to Right-Of-Way rule values
3. Add the rules to the `maliput::api::rules::RoadRulebook`
4. Add the traffic light to the `maliput::api::rules::TrafficLightBook`

## Color Mapping to maliput

YAML colors map to `maliput::api::rules::BulbColor` enum:

- `"Red"` → `BulbColor::kRed`
- `"Yellow"` → `BulbColor::kYellow`
- `"Green"` → `BulbColor::kGreen`

## Type Mapping to maliput

YAML types map to `maliput::api::rules::BulbType` enum:

- `"Round"` → `BulbType::kRound`
- `"Arrow"` → `BulbType::kArrow`

## State Mapping to maliput

YAML states map to `maliput::api::rules::BulbState` enum:

- `"Off"` → `BulbState::kOff`
- `"On"` → `BulbState::kOn`
- `"Blinking"` → `BulbState::kBlinking`

## Notes

- Severity for all Right-Of-Way rules is fixed at 0 (strict enforcement)
- If `bounding_box` is omitted, maliput uses default dimensions (~12" lens: 0.356m tall × 0.356m wide × 0.177m deep)
- Arrow orientation is only valid (and required) when `type: "Arrow"`
