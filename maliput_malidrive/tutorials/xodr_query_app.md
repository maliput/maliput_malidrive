\page xodr_query_app xodr_query application

# Perform queries to a XODR file

`xodr_query` application can be used to perform different kind of queries to a particular XODR description.


```bash
  xodr_query <xodr_file> <query>
```

For example, let's use `GetGeometries` query.
```bash
  xodr_query <xodr_file> GetGeometries <road_id>
```

```bash
$ xodr_query highway.xodr GetGeometries 101

[INFO] Parser: Allow schema errors: disabled
[INFO] Parser: Allow semantic errors: disabled
planView description for RoadId 101:
	1 - Geometry type: line | s: 0 | {x, y} : {2.67623487219478, -89.6625048603155} | hdg: 2.51459637064416
	2 - Geometry type: arc - curvature: 0.0394931278551992 | s: 2.05 | {x, y} : {1.01603038272228, -88.4596464468541} | hdg: 2.51460260559547
	3 - Geometry type: line | s: 13.9030097082693 | {x, y} : {-9.8339561755489, -83.9624505062969} | hdg: 2.98271503347307

```

## Commands

Supported commands:

`xodr_query <xodr_file> FindJunction <junction_id>` : *Obtains the XODR Junction hose ID is junction_id.*

`xodr_query <xodr_file> FindLargestElevationGap` :  *Obtains the largest gap in elevation functions of the XODR.*

`xodr_query <xodr_file> FindLargestGap` : *Obtains the largest gap of the XODR.*

`xodr_query <xodr_file> FindLargestGeometry` : *Obtains the largest geometry of the XODR.*

`xodr_query <xodr_file> FindLargestLaneSection` : *Obtains the largest laneSection of the XODR.*

`xodr_query <xodr_file> FindLargestSuperelevationGap` : *Obtains the largest superelevation gap of the XODR.*

`xodr_query <xodr_file> FindRoad RoadId` : *Obtains the XODR Road whose ID is RoadId.*

`xodr_query <xodr_file> FindShortestElevationGap` : *Obtains the shortest gap in elevation functions of the XODR.*

`xodr_query <xodr_file> FindShortestGap` : *Obtains the shortest gap of the XODR.*

`xodr_query <xodr_file> FindShortestGeometry` : *Obtains the shortest geometry of the XODR.*

`xodr_query <xodr_file> FindShortestLaneSection` : *Obtains the shortest laneSection of the XODR.*

`xodr_query <xodr_file> FindShortestSuperelevationGap` : *Obtains the shortest superelevation gap of the XODR.*

`xodr_query <xodr_file> GetGeometries <road_id>` : *Retrieves a list of the geometries of the correspondent road.*

`xodr_query <xodr_file> GetGeometriesToSimplify <tolerance>` : *Retrieves a list of geometries that can be simplified into simpler geometry descriptions and the type of geometry that would do it.*

`xodr_query <xodr_file> GetHeader` : *Prints the XODR Header.*

## More available options

`xodr_query` application has some arguments that can be used when performing queries. All of them can be accessed by doing `--help`

`--allow_schema_errors`: *If true, the XODR parser will attempt to work around XODR schema violations. By default set to `false`.*

`--allow_semantic_errors`: *If true, the XODR parser will attempt to work around XODR semantic violations. By default set to `false`.*

`--log_level`: *Sets the log output threshold; possible values: maliput::common::logger::level. By default set to `unchanged`.*

`--tolerance`: *Tolerance used to validate continuity in piecewise defined geometries. By default set to `0.001`.*
