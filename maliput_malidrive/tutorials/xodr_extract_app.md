\page xodr_extract_app xodr_extract application

## Generates a XODR description out of selected Roads from a XODR file.

`xodr_extract` application can be used to easily create a brand new XODR file containing selected Roads from other XODR file.
This application is particularly useful when debugging large maps.


```bash
  xodr_extract <xodr_file> <road_1> <road_2> ... <road_n> --output_file_path=<output_xodr_file_path>
```

For example, let's create a XODR file using only Roads 0, 1 and 2 of `TShapeRoad.xodr` file

```bash
$ xodr_query TShapeRoad.xodr 0 1 2 --output_file_path=CroppedTShapeRoad.xodr

[INFO] xodr_extract application
	|__ xodr_file_path: TShapeRoad.xodr
	|__ output_file_path: CroppedTShapeRoad.xodr
	|__ update_linkage: True
	|__ road_ids(3): 0, 1, 2,
[INFO] XODR file created: CroppedTShapeRoad.xodr

```

**Note:** *By default, the junction and the Road linkage (successor and predecessor roads) information is modified to not cause any kind of inconsistency in the new XODR file. (Road linkage is removed and the junction ids of the Roads are changed to "-1"). To change this behavior and extract the Roads exactly as they are the flag `--update_linkage=false` could be passed.*

## More available options

`xodr_extract` application has some available arguments that can be used. All of them can be accessed by doing `--help`

`--update_linkage` : *Update road linkage information and junction id information. By default set to true.*

`--output_file_path` : *Output XODR file path By default set to "./xodr_extract_output.xodr".*

`--log_level`: *Sets the log output threshold; possible values: maliput::common::logger::level. By default set to `unchanged`.*
