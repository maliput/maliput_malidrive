# Traffic Control Device database parser

This directory contains a C++ parser and supporting utilities that read and validate YAML-based traffic device definitions to create maliput traffic control devices such as `TrafficLight`s, `TrafficSign`s, `RoadMarking`s, and `RoadObject`s.

## Overview

A traffic control device type definition describes the physical structure, semantics, and control logic of a traffic control device. Each definition contains two sections:

- **`odr_representation`**: The OpenDRIVE signal / object attributes used to match the device type (`type`, `subtype`, `country`, `country_revision`, `name`).
- **`properties`**: The device's characteristics — `device_type`, `device_semantics`, bulb structure, rule states, bounding box, and more.

The database has **two root keys**:

- **`odr_signal_types`** — devices represented by OpenDRIVE `signal` elements (traffic lights, traffic signs).
- **`odr_object_types`** — devices represented by OpenDRIVE `object` elements (road markings, road objects).

At least one of the two root keys must be present. The parser returns a single flat `std::vector<TrafficControlDeviceDefinition>` that mixes signal and object entries; downstream code discriminates them via the `device_type` field.

The parser in this directory reads YAML files containing these traffic control device type definitions, validates them, and creates the corresponding maliput `TrafficLight`, `TrafficSign`, road marking, and road object representations.

## How It Works

When parsing an OpenDRIVE (`*.xodr`) road network file, traffic signals carry attributes such as `type`, `subtype`, `country`, and `country_revision`:

```xml
<signal type="1000001" subtype="-1" country="OpenDRIVE" s="10.5" t="2.0" zOffset="4.5" ... />
```

OpenDRIVE `object` elements carry their own attributes (`type`, `subtype`, `name`):

```xml
<object type="roadMark" name="StopLine" s="12.0" t="0.0" ... />
```

A parser uses this database to:

1. **Match the signal or object** against an `odr_representation` entry.
2. **Determine the device category** from `device_type` — `TrafficLight`, `TrafficSign`, `RoadMarking`, or `RoadObject`.
3. **Create the corresponding maliput object** — for traffic signs, `device_semantics` is mapped to a `TrafficSignType`.
4. **Link to affected lanes** using the signal's validity information from the XODR file.

## YAML Structure

### `odr_signal_types`

Each entry uses a two-level structure:

```yaml
odr_signal_types:
  - odr_representation:
      type: "1000001"           # Signal type identifier (must match XODR signal type)
      subtype: "-1"             # Optional: signal subtype ("-1" / "none" treated as absent)
      country: "OpenDRIVE"      # Optional: country code or standard
      country_revision: null    # Optional: country standard revision
    properties:
      device_type: TrafficLight  # Required: "TrafficLight" or "TrafficSign"
      device_semantics: Stop      # Optional: semantic meaning for traffic signs
      description: "..."          # Optional: Human-readable description
      is_position_dynamic: false  # Optional: whether position may change at runtime
      default_bounding_box:       # Optional: device-level bounding box
        length: length            # Required: defined in the local coordinate system u/v along the u-axis
        width: width              # Required: defined in the local coordinate system u/v along the v-axis
        height: height            # Required: defined in the local coordinate system u/v along the z-axis
      bulbs: [...]                # Optional: only meaningful for TrafficLight
      rule_states: [...]          # Optional: only meaningful for TrafficLight
```

For signals, `device_type` MUST be `TrafficLight` or `TrafficSign`. Using `RoadMarking` or `RoadObject` in a signal entry is rejected with a parser error.

### `odr_object_types`

Object entries follow a **stricter** schema than signal entries:

```yaml
odr_object_types:
  - odr_representation:
      type: roadMark            # Required: OpenDRIVE object type
      subtype: null             # Optional: object subtype
      name: "StopLine"          # Optional: OpenDRIVE name attribute
    properties:
      device_type: RoadMarking # Required: "RoadMarking" or "RoadObject"
      device_semantics: StopLine  # Optional
      description: "..."        # Optional
      is_position_dynamic: false   # Optional
      default_bounding_box:       # Optional: device-level bounding box
        length: length            # Required: defined in the local coordinate system u/v along the u-axis
        width: width              # Required: defined in the local coordinate system u/v along the v-axis
        height: height            # Required: defined in the local coordinate system u/v along the z-axis
```

The following fields are **not allowed** in `odr_object_types` entries and are rejected by the parser:

- `country` and `country_revision` in `odr_representation` (objects don't use them in OpenDRIVE).
- `bulbs` and `rule_states` in `properties` (objects don't carry bulbs or rule logic).

For objects, `device_type` MUST be `RoadMarking` or `RoadObject`. Using `TrafficLight` or `TrafficSign` in an object entry is rejected with a parser error.

#### `device_type`

Required string that identifies the device category. Used to route entries to the right maliput book:

| `device_type` value  | Valid root             | Result                                              |
|----------------------|------------------------|-----------------------------------------------------|
| `"TrafficLight"`    | `odr_signal_types`     | Creates a `maliput::api::rules::TrafficLight`       |
| `"TrafficSign"`     | `odr_signal_types`     | Creates a `maliput::api::rules::TrafficSign`        |
| `"RoadMarking"`     | `odr_object_types`     | Creates a `maliput::api::object::RoadMarking`       |
| `"RoadObject"`      | `odr_object_types`     | Creates a `maliput::api::object::RoadObject`        |

The `device_type` matching is **case-insensitive**: `"Road_Marking"`, `"ROAD_MARKING"`, and `"RoadMarking"` are all equivalent.

#### `device_semantics`

Optional string for traffic signs that identifies the semantic meaning. Defaults to `"Other"` when absent. The supported values and their mapping to `maliput::api::rules::TrafficSignType` are:

| `device_semantics` value | `TrafficSignType` enum      |
|--------------------------|------------------------------|
| `"Stop"`                 | `kStop`                     |
| `"Yield"`                | `kYield`                    |
| `"GiveWay"`             | `kYield`                    |
| `"SpeedLimit"`          | `kSpeedLimit`               |
| `"SpeedLimitBegin"`    | `kSpeedLimit`               |
| `"NoEntry"`             | `kNoEntry`                  |
| `"OneWay"`              | `kOneWay`                   |
| `"PedestrianCrossing"`  | `kPedestrianCrossing`       |
| `"Crosswalk"`            | `kPedestrianCrossing`       |
| `"NoLeftTurn"`         | `kNoLeftTurn`               |
| `"NoRightTurn"`        | `kNoRightTurn`              |
| `"NoUTurn"`            | `kNoUTurn`                  |
| `"SchoolZone"`          | `kSchoolZone`               |
| `"Construction"`         | `kConstruction`             |
| `"RailroadCrossing"`    | `kRailroadCrossing`         |
| `"NoOvertaking"`        | `kNoOvertaking`             |
| *(any Other value)*      | `kUnknown`                  |

If `device_semantics` is set to a value not listed above, the builder will still create a `TrafficSign` with `TrafficSignType::kUnknown`. This allows the database to contain signal definitions for region-specific or non-standard sign types without requiring code changes.

The `device_semantics` matching is **case-insensitive**: `"No_Entry"`, `"NO_ENTRY"`, and `"NoEntry"` are all equivalent.

#### Signal Fingerprint Matching

Each entry in the database is uniquely identified by a **fingerprint** composed of the following fields:

| Field              | Required in Signals | Required in Objects | Description                                |
| ------------------ | :-----: | :-----: | ------------------------------------------ |
| `type`             | Yes     | Yes     | Signal/object type identifier              |
| `subtype`          | No      | No      | Subtype for finer matching                 |
| `country`          | No      | —       | Country code or standard (signals only)    |
| `country_revision` | No      | —       | Country standard revision (signals only)   |
| `name`             | No      | No      | OpenDRIVE `name` attribute                 |

Only `type` is mandatory. Omitted optional fields are treated as null. When the parser looks up an XODR signal/object in the database, all fields are compared — omitted fields match only when the corresponding database entry also leaves them unset.

The wildcard character `"*"` may be used for any optional field to match any value, including null. Specificity ranking is used to break ties when multiple entries match: the entry with the most non-wildcard fields wins.

#### Per-root conflict validation

Equal-specificity conflicts are detected **per root**:

- Two `odr_signal_types` entries that can match the same input and have equal specificity → rejected.
- Two `odr_object_types` entries that can match the same input and have equal specificity → rejected.
- A signal and an object entry sharing the same fingerprint do **NOT** conflict — OpenDRIVE signals and objects live in disjoint namespaces, so they are validated independently.

Object specificity counts only the three object fields (`type`, `subtype`, `name`) — `country`/`country_revision` do not apply.

### Bulbs

Traffic light definitions may contain a `bulbs` list (only valid under `odr_signal_types` with `device_type: TrafficLight`):

```yaml
bulbs:
  - id: "RedBulb"
    position_traffic_light: [x, y, z]
    orientation_traffic_light: [w, x, y, z]
    color: Red
    type: Round
    states: [Off, On, Blinking]
    initial_state: Off
    bounding_box:
      p_min: [x, y, z]
      p_max: [x, y, z]
    arrow_orientation_rad: 0.0  # Required only when type: Arrow
```

- `states` is optional; if omitted, the parser defaults to `[Off, On]`.
- `initial_state` is optional; if omitted, it defaults to `Off`.
- `arrow_orientation_rad` is only valid for `type: Arrow`.

## Coordinate Frames

All positions and orientations use right-handed coordinate systems:

- **+X**: device forward
- **+Y**: left
- **+Z**: up

Frame hierarchy:

1. **Inertial / road network frame**
2. **Traffic light frame**
3. **Bulb frame**

## Enum Mappings

Enum parsing is case-insensitive. Preferred YAML values are lowercase / snake_case. Legacy PascalCase values are still accepted for backward compatibility.

### BulbColor

| Preferred value | Legacy alias | maliput enum |
|-----------------|--------------|--------------|
| `Red`           | `Red`        | `BulbColor::kRed` |
| `Yellow`        | `Yellow`     | `BulbColor::kYellow` |
| `Green`         | `Green`      | `BulbColor::kGreen` |

### BulbType

The preferred snake_case values include the newer aliases `arrow_left`, `arrow_right`, `arrow_up`, `arrow_upper_left`, `arrow_upper_right`, `UTurnLeft`, `UTurnRight`, and `DontWalk`.

| Preferred value       | Legacy alias        | maliput enum |
|-----------------------|---------------------|--------------|
| `Round`               | `Round`             | `BulbType::kRound` |
| `Arrow`               | `Arrow`             | `BulbType::kArrow` |
| `arrow_left`          | `ArrowLeft`         | `BulbType::kArrowLeft` |
| `arrow_right`         | `ArrowRight`        | `BulbType::kArrowRight` |
| `arrow_up`            | `ArrowUp`           | `BulbType::kArrowUp` |
| `arrow_upper_left`    | `ArrowUpperLeft`    | `BulbType::kArrowUpperLeft` |
| `arrow_upper_right`   | `ArrowUpperRight`   | `BulbType::kArrowUpperRight` |
| `UTurnLeft`         | `UTurnLeft`         | `BulbType::kUTurnLeft` |
| `UTurnRight`        | `UTurnRight`        | `BulbType::kUTurnRight` |
| `walk`                | `Walk`              | `BulbType::kWalk` |
| `DontWalk`           | `DontWalk`          | `BulbType::kDontWalk` |

### BulbState

| Preferred value | Legacy alias | maliput enum |
|-----------------|--------------|--------------|
| `Off`           | `Off`        | `BulbState::kOff` |
| `On`            | `On`         | `BulbState::kOn` |
| `Blinking`      | `Blinking`   | `BulbState::kBlinking` |
| `counting`      | `Counting`   | `BulbState::kCounting` |

## Examples

A combined database with both roots:

```yaml
odr_signal_types:
  - odr_representation:
      type: "206"
      subtype: "-1"
    properties:
      device_type: TrafficSign
      device_semantics: GiveWay
      description: "Give way sign"
      default_bounding_box:
        length: 0.001
        width: 1.0
        height: 1.0
      rule_states:
        - conditions: []
          value: "StopIfSafe"

odr_object_types:
  # Stop line matched by name (e.g., RoadRunner output)
  - odr_representation:
      type: roadMark
      name: "StopLine"
    properties:
      device_type: RoadMarking
      device_semantics: StopLine
      description: "Stop line road marking"

  # Generic Crosswalk
  - odr_representation:
      type: Crosswalk
    properties:
      device_type: RoadMarking
      device_semantics: Crosswalk
      description: "Generic OpenDRIVE Crosswalk object"

  # Generic pole
  - odr_representation:
      type: pole
    properties:
      device_type: RoadObject
      description: "Generic OpenDRIVE pole object"
```

See also:

- `resources/traffic_control_device_db/traffic_control_device_db_example.yaml`

## Notes

- If bulb's `bounding_box` is omitted, maliput uses its default bulb dimensions.
- OpenDRIVE `length`, `width`, and `height` may override `default_bounding_box` values.
- The OpenDRIVE `name` field is supported for matching because some authoring tools encode object meaning there.
