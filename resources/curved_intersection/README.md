This directory contains configuration files for the curved intersection.
The RoadRunner scenario files are located in:

https://github.com/ToyotaResearchInstitute/Worlds3D/blob/master/trivial_roads/trivial_roads_rr/Scenes/curved_intersection_01.rrscene

The lane-s-route-\*.yaml files were obtained using git SHA
70f59edcb8319d6184fe777481007de3d2cdf779.

Example of how to derive a lane-s-route using one of the aforementioned YAML
files:

$ ./install/malidrive/bin/malidrive/applications/malidrive_derive_lane_s_routes \
--config_file src/malidrive/resources/curved_intersection/lane-s-route_north-straight.yaml
