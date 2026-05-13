# Traffic Control Device database parser

This directory contains a C++ parser and supporting utilities that read and validate YAML-based traffic device definitions to create maliput traffic control devices such as `TrafficLight`s and `TrafficSign`s.

## Overview

A traffic control device type definition describes the physical structure, semantics, and control logic of a traffic control device. Each definition contains two sections:

- **`odr_representation`**: The OpenDRIVE signal attributes used to match the device type (`type`, `subtype`, `country`, `country_revision`)
- **`properties`**: The device's characteristics — `device_type`, `device_semantics`, bulb structure, rule states, bounding box, and more

The parser in this directory reads YAML files containing these traffic control device type definitions, validates them, and creates the corresponding maliput `TrafficLight` and `TrafficSign` objects.

## How It Works

When parsing an OpenDRIVE (`*.xodr`) road network file, traffic signals carry attributes such as `type`, `subtype`, `country`, and `country_revision`:

```xml
<signal type="1000001" subtype="-1" country="OpenDRIVE" s="10.5" t="2.0" zOffset="4.5" ... />
```

A parser uses this database to:

1. **Match the signal** against an `odr_representation` entry using `type`, `subtype`, `country`, and `country_revision`
2. **Determine the device category** from `device_type` — either `traffic_light` or `traffic_sign`
3. **Create a maliput `TrafficLight` or `TrafficSign`** — for traffic signs, `device_semantics` is mapped to a `TrafficSignType`
4. **Link to affected lanes** using the signal's validity information from the XODR file

## YAML Structure

Each YAML file in this directory should define a list of signal types under the top-level key `odr_signal_types`. Each entry uses a two-level structure:

### Signal Type Header

```yaml
- odr_representation:
    type: "1000001"           # Signal type identifier (must match XODR signal type)
    subtype: "-1"             # Optional: signal subtype ("-1" / "none" treated as absent)
    country: "OpenDRIVE"      # Optional: country code or standard
    country_revision: null     # Optional: country standard revision
  properties:
    device_type: traffic_light  # Required: "traffic_light" or "traffic_sign"
    device_semantics: stop      # Optional: semantic meaning for traffic signs
    description: "..."          # Optional: Human-readable description
    is_position_dynamic: false  # Optional: whether position may change at runtime
    default_bounding_box:       # Optional: device-level bounding box
      p_min: [x, y, z]
      p_max: [x, y, z]
```

#### `device_type`

Required string that identifies the signal category. Used to route signals to the right maliput book:

| `device_type` value  | Result                                              |
|----------------------|-----------------------------------------------------|
| `"traffic_light"`    | Creates a `maliput::api::rules::TrafficLight`       |
| `"traffic_sign"`     | Creates a `maliput::api::rules::TrafficSign`        |

#### `device_semantics`

Optional string for traffic signs that identifies the semantic meaning. Defaults to `"other"` when absent. The supported values and their mapping to `maliput::api::rules::TrafficSignType` are:

| `device_semantics` value | `TrafficSignType` enum      |
|--------------------------|------------------------------|
| `"stop"`                 | `kStop`                     |
| `"yield"`                | `kYield`                    |
| `"give_way"`             | `kYield`                    |
| `"speed_limit"`          | `kSpeedLimit`               |
| `"speed_limit_begin"`    | `kSpeedLimit`               |
| `"no_entry"`             | `kNoEntry`                  |
| `"one_way"`              | `kOneWay`                   |
| `"pedestrian_crossing"`  | `kPedestrianCrossing`       |
| `"crosswalk"`            | `kPedestrianCrossing`       |
| `"no_left_turn"`         | `kNoLeftTurn`               |
| `"no_right_turn"`        | `kNoRightTurn`              |
| `"no_u_turn"`            | `kNoUTurn`                  |
| `"school_zone"`          | `kSchoolZone`               |
| `"construction"`         | `kConstruction`             |
| `"railroad_crossing"`    | `kRailroadCrossing`         |
| `"no_overtaking"`        | `kNoOvertaking`             |
| *(any other value)*      | `kUnknown`                  |

If `device_semantics` is set to a value not listed above, the builder will still create a `TrafficSign` with `TrafficSignType::kUnknown`. This allows the database to contain signal definitions for region-specific or non-standard sign types without requiring code changes.

The `device_semantics` matching is **case-insensitive**: `"No_Entry"`, `"NO_ENTRY"`, and `"no_entry"` are all equivalent.

#### Signal Fingerprint Matching

Each signal definition in the database is uniquely identified by a **fingerprint** composed of the following fields:

| Field              | Required | Description                                |
| ------------------ | -------- | ------------------------------------------ |
| `type`             | Yes      | Signal type identifier (e.g., `"1000001"`) |
| `subtype`          | No       | Signal subtype for finer matching          |
| `country`          | No       | Country code or standard                   |
| `country_revision` | No       | Country standard revision                  |

Only `type` is mandatory; `subtype`, `country`, and `country_revision` may be omitted (treated as null). When the parser looks up an XODR signal in the database, all fields are compared — omitted fields match only when the corresponding database entry also leaves them unset. This means two entries that share the same `type` but differ in any optional field (present vs. absent, or different values) are considered distinct definitions.

### Bulbs

Each signal type contains a list of `bulbs`:

```yaml
  bulbs:                     # List of bulbs
    - id: "BulbName"
      position_traffic_light: [x, y, z]          # Position relative to traffic light frame
      orientation_traffic_light: [w, x, y, z]    # Orientation relative to traffic light frame
      color: "Red"           # One of: Red, Yellow, Green
      type: "Round"          # One of: Round, Arrow, ArrowLeft, ArrowRight, ArrowUp, ArrowUpperLeft, ArrowUpperRight, UTurnLeft, UTurnRight, Walk, DontWalk
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
        - conditions:
            - bulb_id: "BulbId"
              state: "On"
            - bulb_id: "OtherBulbId"
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

## Examples

See `resources/traffic_control_device_db/traffic_control_device_db_example.yaml` for detailed examples including:

- Standard three-bulb vertical traffic light (type 1000001)
- Arrow-based traffic light (type 1000011)
- Pedestrian signal (type 1000002)

## Usage in Parsers

A parser loading these files should:

1. Load the YAML file
2. For each signal in the XODR file with type/subtype/country/country_revision matching a database entry:
   - Check the `device_type` to know whether to create a maliput `maliput::api::rules::TrafficLight` or `maliput::api::rules::TrafficSign`
   - If it is a `maliput::api::rules::TrafficSign`, assign its type by mapping the `device_semantics` field
   - If it is a `maliput::api::rules::TrafficLight`, load its bulb structure
3. Check for lane validities in the XODR file and set the related lanes for each signal and traffic light.
4. Add `maliput::api::rules::TrafficSign`s to the `maliput::api::rules::TrafficSignBook` and `maliput::api::rules::TrafficLight`s to the `maliput::api::rules::TrafficLightBook`

## Color Mapping to maliput

YAML colors map to `maliput::api::rules::BulbColor` enum:

- `"Red"` → `BulbColor::kRed`
- `"Yellow"` → `BulbColor::kYellow`
- `"Green"` → `BulbColor::kGreen`

## Type Mapping to maliput

YAML types map to `maliput::api::rules::BulbType` enum:

- `"Round"` → `BulbType::kRound`
- `"Arrow"` → `BulbType::kArrow`
- `"ArrowLeft"` → `BulbType::kArrowLeft`
- `"ArrowRight"` → `BulbType::kArrowRight`
- `"ArrowUp"` → `BulbType::kArrowUp`
- `"ArrowUpperLeft"` → `BulbType::kArrowUpperLeft`
- `"ArrowUpperRight"` → `BulbType::kArrowUpperRight`
- `"UTurnLeft"` → `BulbType::kUTurnLeft`
- `"UTurnRight"` → `BulbType::kUTurnRight`
- `"Walk"` → `BulbType::kWalk`
- `"DontWalk"` → `BulbType::kDontWalk`

## State Mapping to maliput

YAML states map to `maliput::api::rules::BulbState` enum:

- `"Off"` → `BulbState::kOff`
- `"On"` → `BulbState::kOn`
- `"Blinking"` → `BulbState::kBlinking`

## Notes

- If `bounding_box` is omitted, maliput uses default dimensions (~12" lens: 0.356m tall × 0.356m wide × 0.177m deep)
- Arrow orientation is only valid (and required) when `type: "Arrow"`
