\page xodr_validate_app xodr_validate application

# Validate a XODR file

`xodr_validate` application can be used to verify the description of an XODR file.

If the XODR file doesn't meet OpenDRIVE specification the application will notify.


```bash
  xodr_validate --xodr_file=<xodr_file_path>
```

For example:
```bash
$ xodr_validate --xodr_file=TShapeRoad.xodr

[INFO] Parser: Allow schema errors: disabled
[INFO] Parser: Allow semantic errors: disabled
Successfully loaded the map.

```

Let's try another example where the outcome isn't ideal, and let's use `trace` as log level to get more information about how the parser progress in the xodr file.

```
$ xodr_validate --xodr_file=NonContiguousRoad.xodr --log_level=trace
[INFO] Parser: Allow schema errors: disabled
[INFO] Parser: Allow semantic errors: disabled
[TRACE] XODR Parser configuration:
[TRACE] |__ tolerance: 0.001000
[TRACE] |__ allow_schema_errors: Disabled
[TRACE] |__ allow_semantic_errors: Disabled
[TRACE] ["maliput_ws/src/maliput_malidrive/maliput_malidrive/src/maliput_malidrive/xodr/db_manager.cc":47:ParseDoc] XODR parsing process has started.
[TRACE] ["maliput_ws/src/maliput_malidrive/maliput_malidrive/src/maliput_malidrive/xodr/db_manager.cc":48:ParseDoc] Verifying XODR tag in the file.
[TRACE] ["maliput_ws/src/maliput_malidrive/maliput_malidrive/src/maliput_malidrive/xodr/db_manager.cc":53:ParseDoc] Parsing header node.
[TRACE] ["maliput_ws/src/maliput_malidrive/maliput_malidrive/src/maliput_malidrive/xodr/db_manager.cc":59:ParseDoc] Parsing road headers.
[TRACE] ["maliput_ws/src/maliput_malidrive/maliput_malidrive/src/maliput_malidrive/xodr/parser.cc":797:As<malidrive::xodr::RoadHeader>] Parsing road id: 0
[TRACE] ["maliput_ws/src/maliput_malidrive/maliput_malidrive/src/maliput_malidrive/xodr/parser.cc":815:As<malidrive::xodr::RoadHeader>] Parsing road link.
[TRACE] ["maliput_ws/src/maliput_malidrive/maliput_malidrive/src/maliput_malidrive/xodr/parser.cc":822:As<malidrive::xodr::RoadHeader>] Parsing road type.
[TRACE] ["maliput_ws/src/maliput_malidrive/maliput_malidrive/src/maliput_malidrive/xodr/parser.cc":833:As<malidrive::xodr::RoadHeader>] Parsing planView.
parser.cc:As<malidrive::xodr::PlanView>:776: Geometries doesn't meet contiguity constraint.
The map could not be loaded.

```

In this case the XODR validation failed because in the `plainView` node there is no guarantee of contiguity between both geometries, for example.

```xml
<planView>
  <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="4.0">
    <line/>
  </geometry>
  <geometry s="4.0" x="10.0" y="10.0" hdg="0.0" length="0.6">
    <line/>
  </geometry>
</planView>
```


## More available options

`xodr_validate` application has several arguments that can be used when validating the XODR file. All of them can be accessed by doing `--help`

`--allow_schema_errors`: *If true, the XODR parser will attempt to work around XODR schema violations. By default set to `false`.*

`--allow_semantic_errors`: *If true, the XODR parser will attempt to work around XODR semantic violations. By default set to `false`.*

`--log_level`: *Sets the log output threshold; possible values: maliput::common::logger::level. By default set to `unchanged`.*

`--tolerance`: *Tolerance used to validate continuity in piecewise defined geometries. By default set to `0.001`.*
