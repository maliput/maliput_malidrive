^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package maliput_malidrive
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2025-05-20)
------------------
* Allow offset discontinuities when allowing semantic errors in XODRs. (`#306 https://github.com/maliput/maliput_malidrive/issues/306`)
* Contributors: Santiago Lopez

0.4.4 (2025-04-22)
------------------
* Bazel: Bump to use latest maliput and eigen release. (`#303 <https://github.com/maliput/maliput_malidrive/issues/303>`_)
* Contributors: Franco Cipollone

0.4.3 (2025-04-10)
------------------
* Provide method to obtain the road orientation from a OS RoadPosition. (`#297 <https://github.com/maliput/maliput_malidrive/issues/297>`_)
* Provides OpenScenarioRelativeLanePositionToMaliputRoadPosition conversion method. (`#296 <https://github.com/maliput/maliput_malidrive/issues/296>`_)
* Contributors: Santiago Lopez

0.4.2 (2025-03-25)
------------------
* Provides OpenScenarioRelativeRoadPositionToMaliputRoadPosition conversion method. (`#294 <https://github.com/maliput/maliput_malidrive/issues/294>`_)
* Contributors: Franco Cipollone

0.4.1 (2025-03-18)
------------------
* Useful conversions from Maliput to OpenScenario Position types. (`#292 <https://github.com/maliput/maliput_malidrive/issues/292>`_)
* Contributors: Santiago Lopez

0.4.0 (2025-03-07)
------------------
* Implements DoBackendCustomCommand method. (`#289 <https://github.com/maliput/maliput_malidrive/issues/289>`_)
* Provide some useful conversions for OpenScenario users (`#288 <https://github.com/maliput/maliput_malidrive/issues/288>`_)
* Updates to presubmit file. (`#286 <https://github.com/maliput/maliput_malidrive/issues/286>`_)
* Contributors: Franco Cipollone

0.3.1 (2024-11-01)
------------------
* Minor changes to presubmit.yml file. (`#284 <https://github.com/maliput/maliput_malidrive/issues/284>`_)
* Bazel: Bumps maliput to 1.3.1. (`#283 <https://github.com/maliput/maliput_malidrive/issues/283>`_)
* Update wheel_generation.yml (`#282 <https://github.com/maliput/maliput_malidrive/issues/282>`_)
* Update github action versions. (`#281 <https://github.com/maliput/maliput_malidrive/issues/281>`_)
  ---------
* Contributors: Agustin Alba Chicar, Franco Cipollone

0.3.0 (2024-04-29)
------------------
* Flags spiral support. (`#279 <https://github.com/ToyotaResearchInstitute/maliput_malidrive/issues/279>`_)
* Spiral Lane integration (`#277 <https://github.com/ToyotaResearchInstitute/maliput_malidrive/issues/277>`_)
* Adds bazel version information to presubmit.yml file. (`#275 <https://github.com/ToyotaResearchInstitute/maliput_malidrive/issues/275>`_)
* Add curbs to the builder. (`#274 <https://github.com/ToyotaResearchInstitute/maliput_malidrive/issues/274>`_)

  ---------
* Contributors: Agustin Alba Chicar, Franco Cipollone

0.2.1 (2024-02-02)
------------------
* [bazel] Guarantee shared library creation for the plugin (`#269 <https://github.com/maliput/maliput_malidrive/issues/269>`_)
* Enables regression testing in bazel. (`#246 <https://github.com/maliput/maliput_malidrive/issues/246>`_)

0.2.0 (2024-01-05)
------------------
* [xodr] SmallTownRoads.xodr (`#266 <https://github.com/maliput/maliput_malidrive/issues/266>`_)
* [applications] xodr_to_obj (`#259 <https://github.com/maliput/maliput_malidrive/issues/259>`_)
* [infra] commented instructions for development with local maliput/registries (`#263 <https://github.com/maliput/maliput_malidrive/issues/263>`_)
* [infra] use the REAL bazel central registry (`#264 <https://github.com/maliput/maliput_malidrive/issues/264>`_)
* Adds spiral geometry parsing capabilities (`#261 <https://github.com/maliput/maliput_malidrive/issues/261>`_)
* Updates ros-tooling version to avoid error with dependency. (`#256 <https://github.com/maliput/maliput_malidrive/issues/256>`_)
* Replaces drake API by maliput new API. (`#244 <https://github.com/maliput/maliput_malidrive/issues/244>`_)
* Uses try_vcs_checkout for the branching scheme in bazel build. (`#253 <https://github.com/maliput/maliput_malidrive/issues/253>`_)
* Adds vcstool and pip to bazel-zen docker image. (`#254 <https://github.com/maliput/maliput_malidrive/issues/254>`_)
* Adds try_vcs_checkout to wheel_generation workflow. (`#252 <https://github.com/maliput/maliput_malidrive/issues/252>`_)
* Adds -O2 flag to bazel build. (`#255 <https://github.com/maliput/maliput_malidrive/issues/255>`_)
* Adds periodic wheel generation.(once a week) (`#251 <https://github.com/maliput/maliput_malidrive/issues/251>`_)
* Fixes bazel version not being correctly set. (`#249 <https://github.com/maliput/maliput_malidrive/issues/249>`_)
* Adds workflow for manylinux wheel creation. (`#248 <https://github.com/maliput/maliput_malidrive/issues/248>`_)
* Use compare methods instead of test_utilities' . (`#243 <https://github.com/maliput/maliput_malidrive/issues/243>`_)
* Removes maliput_drake dependency (`#240 <https://github.com/maliput/maliput_malidrive/issues/240>`_)
* Decompose bazel superlibrary (`#241 <https://github.com/maliput/maliput_malidrive/issues/241>`_)
* Bazel integration and utility tests (`#238 <https://github.com/maliput/maliput_malidrive/issues/238>`_)
* Removes fmt dependency (`#239 <https://github.com/maliput/maliput_malidrive/issues/239>`_)
* Bazel module (`#237 <https://github.com/maliput/maliput_malidrive/issues/237>`_)
* Containers for local devcontainer and CI workflows (`#235 <https://github.com/maliput/maliput_malidrive/issues/235>`_)
* Matches with change in logger format. (`#232 <https://github.com/maliput/maliput_malidrive/issues/232>`_)
* Adds missing gflags dependency to package. (`#233 <https://github.com/maliput/maliput_malidrive/issues/233>`_)
* Contributors: Agustin Alba Chicar, Daniel Stonier, Franco Cipollone

0.1.4 (2022-12-13)
------------------
* Provides default parameters for the RoadNetworkLoader. (`#231 <https://github.com/maliput/maliput_malidrive/issues/231>`_)
* Uses default ManualPhaseProvider implementation. (`#230 <https://github.com/maliput/maliput_malidrive/issues/230>`_)
* Removes OpenRangeValidator and use maliput's instead. (`#229 <https://github.com/maliput/maliput_malidrive/issues/229>`_)
* Updates triage workflow. (`#228 <https://github.com/maliput/maliput_malidrive/issues/228>`_)
* Contributors: Franco Cipollone

0.1.3 (2022-09-14)
------------------
* Matches with changes in Maliput: Lane::ToLanePosition. (`#227 <https://github.com/maliput/maliput_malidrive/issues/227>`_)
* Adds triage workflow. (`#225 <https://github.com/maliput/maliput_malidrive/issues/225>`_)
* Improves README. (`#224 <https://github.com/maliput/maliput_malidrive/issues/224>`_)
* Contributors: Franco Cipollone

0.1.2 (2022-07-01)
------------------
* Fixes environment hooks. (`#223 <https://github.com/maliput/maliput_malidrive/issues/223>`_)
* Contributors: Franco Cipollone

0.1.1 (2022-06-16)
------------------
* Standardizes CFlags configuration. (`#222 <https://github.com/ToyotaResearchInstitute/maliput_malidrive/issues/222>`_)
* Suppresses old-rule-api-related deprecation warnings.
* Fixes include folder installation.
* Contributors: Franco Cipollone

0.1.0 (2022-06-13)
------------------
* Use <doc_depend> for ament_cmake_doxygen dependency (`#217 <https://github.com/maliput/maliput_malidrive/issues/217>`_)
* Uses ros-action-ci in build.yaml workflow. (`#218 <https://github.com/maliput/maliput_malidrive/issues/218>`_)
* Moves package to root (`#216 <https://github.com/maliput/maliput_malidrive/issues/216>`_)
* Updates license. (`#215 <https://github.com/maliput/maliput_malidrive/issues/215>`_)
* Uses ament_export_targets. (`#214 <https://github.com/maliput/maliput_malidrive/issues/214>`_)
* Removes dashing support. (`#213 <https://github.com/maliput/maliput_malidrive/issues/213>`_)
* Adds the RoadGeometry to the IntersectionBook construction points. (`#211 <https://github.com/maliput/maliput_malidrive/issues/211>`_)
* Modifies library dependancies from utilities to utility (`#210 <https://github.com/maliput/maliput_malidrive/issues/210>`_)
* Renames PhaseBasedRightOfWayDiscreteValueRuleStateProvider class (`#209 <https://github.com/maliput/maliput_malidrive/issues/209>`_)
* Merge pull request `#208 <https://github.com/maliput/maliput_malidrive/issues/208>`_ from maliput/voldivh/value_range_method_to_state
  Changed values() and ranges() method from Rules to states()
* [FEAT]: Changed values() and ranges() method from Rules to states()
* Adds LoopRoadPedestrianCrosswalk xodr and yaml. (`#207 <https://github.com/maliput/maliput_malidrive/issues/207>`_)
* Fixes RoWR zone in SingleRoadPedestrianCrosswalk.yaml file. (`#206 <https://github.com/maliput/maliput_malidrive/issues/206>`_)
* Adds missing SingleRoadPedestrianCrosswalk.yaml. (`#205 <https://github.com/maliput/maliput_malidrive/issues/205>`_)
* Adds XODR and YAML to describe crosswalk intersection (`#203 <https://github.com/maliput/maliput_malidrive/issues/203>`_)
* Creates PhaseProviderBuilder functor. (`#202 <https://github.com/maliput/maliput_malidrive/issues/202>`_)
* Adds BUILD_DOCS flag as opt-in flag. (`#204 <https://github.com/maliput/maliput_malidrive/issues/204>`_)
* Uses PhaseRingBook loader for new rule api. (`#200 <https://github.com/maliput/maliput_malidrive/issues/200>`_)
* Supports loading rules via RuleRegistry + new RoadRulebook format. (`#198 <https://github.com/maliput/maliput_malidrive/issues/198>`_)
* Fixes typo in README (`#197 <https://github.com/maliput/maliput_malidrive/issues/197>`_)
* Adds workflow_dispatch option to scan_build and clan runs. (`#196 <https://github.com/maliput/maliput_malidrive/issues/196>`_)
* Adds CI badges (`#195 <https://github.com/maliput/maliput_malidrive/issues/195>`_)
* Improves the README adding info about OpenDRIVE specs coverage. (`#194 <https://github.com/maliput/maliput_malidrive/issues/194>`_)
* Replaces push by workflow_dispatch event in gcc build. (`#192 <https://github.com/maliput/maliput_malidrive/issues/192>`_)
* Improves standard_strictness_policy flag's documentation. (`#189 <https://github.com/maliput/maliput_malidrive/issues/189>`_)
* Exposes RoadNetwork configuration builder keys. (`#188 <https://github.com/maliput/maliput_malidrive/issues/188>`_)
* Pairs with maliput::plugin::RoadNetworkLoader functor change. (`#186 <https://github.com/maliput/maliput_malidrive/issues/186>`_)
* Adds max_linear_tolerance parameter: linear tolerance as a range (`#182 <https://github.com/maliput/maliput_malidrive/issues/182>`_)
* Improves use of maliput plugin architecture. (`#181 <https://github.com/maliput/maliput_malidrive/issues/181>`_)
* Removes RoadNetworkConfiguration-based load method. (`#177 <https://github.com/maliput/maliput_malidrive/issues/177>`_)
* Use maliput drake (`#178 <https://github.com/maliput/maliput_malidrive/issues/178>`_)
* Fixes help message for the xodr apps. (`#180 <https://github.com/maliput/maliput_malidrive/issues/180>`_)
* Adds ToRoadPosition query test. (`#175 <https://github.com/maliput/maliput_malidrive/issues/175>`_)
* Fixes bug at ToRoadPosition query. (`#174 <https://github.com/maliput/maliput_malidrive/issues/174>`_)
* Solves CMP0076 cmake warning related to log_level_flag target. (`#167 <https://github.com/maliput/maliput_malidrive/issues/167>`_)
* Reduces integrator's maximum_step_size to half of scale_length. (`#172 <https://github.com/maliput/maliput_malidrive/issues/172>`_)
* Documents maliput_malidrive xodr apps. (`#170 <https://github.com/maliput/maliput_malidrive/issues/170>`_)
* Fixes doxygen format at docstring. (`#171 <https://github.com/maliput/maliput_malidrive/issues/171>`_)
* Reduce epsilon value in lane's s_range_validation. (`#169 <https://github.com/maliput/maliput_malidrive/issues/169>`_)
* Enable doxygen verification. (`#168 <https://github.com/maliput/maliput_malidrive/issues/168>`_)
* Removes RoadGeometryBuilderBase in favor of less (unnecessary) abstraction. (`#166 <https://github.com/maliput/maliput_malidrive/issues/166>`_)
* Improves ToRoadPosition query. (`#165 <https://github.com/maliput/maliput_malidrive/issues/165>`_)
* Unify RoadNetworkBuilderBase with its implementation. (`#164 <https://github.com/maliput/maliput_malidrive/issues/164>`_)
* Remove InertialToLaneMappingConfig. (`#163 <https://github.com/maliput/maliput_malidrive/issues/163>`_)
* Removes from the installation files that (`#162 <https://github.com/maliput/maliput_malidrive/issues/162>`_)
  should not be installed anymore.
  Installed header files for maliput_malidrive
  consumers, like malidrive was, are no longer needed.
* xodr_extract app: Generates XODR file from another XODR's roads. (`#154 <https://github.com/maliput/maliput_malidrive/issues/154>`_)
* Improves throw message: Print range of tolerances evaluated (`#157 <https://github.com/maliput/maliput_malidrive/issues/157>`_)
* Adjusts integrators accuracy. (`#152 <https://github.com/maliput/maliput_malidrive/issues/152>`_)
* xodr_query app: Adds GetGeometries command. (`#151 <https://github.com/maliput/maliput_malidrive/issues/151>`_)
* Relax G1 contiguity for non drivable lanes (`#149 <https://github.com/maliput/maliput_malidrive/issues/149>`_)
* Includes Town01-07 maps. (`#145 <https://github.com/maliput/maliput_malidrive/issues/145>`_)
* RoadNetwork configuration as string-to-string map (`#143 <https://github.com/maliput/maliput_malidrive/issues/143>`_)
* Replaces AMENT_CURRENT_PREFIX by COLCON_PREFIX_PATH (`#141 <https://github.com/maliput/maliput_malidrive/issues/141>`_)
* Supports lane-width description with negative value (`#140 <https://github.com/maliput/maliput_malidrive/issues/140>`_)
* Tolerance selection: Improve logging and fix bug. (`#138 <https://github.com/maliput/maliput_malidrive/issues/138>`_)
* Set up linker properly when using clang in CI. (`#127 <https://github.com/maliput/maliput_malidrive/issues/127>`_)
* Disables extensive tests on tsan builds. (`#134 <https://github.com/maliput/maliput_malidrive/issues/134>`_)
* Solves warnings while clang build. (`#129 <https://github.com/maliput/maliput_malidrive/issues/129>`_)
* Improves cmake coding for testing. (`#132 <https://github.com/maliput/maliput_malidrive/issues/132>`_)
* Removes repeated cmake flags settings (`#131 <https://github.com/maliput/maliput_malidrive/issues/131>`_)
* Fixes throw when p value from integrator is negative (`#124 <https://github.com/maliput/maliput_malidrive/issues/124>`_)
* Fix some warnings related to unnecessary std::move() calls. (`#126 <https://github.com/maliput/maliput_malidrive/issues/126>`_)
* Increases kMaxToleranceSelectionRounds to 24. (`#125 <https://github.com/maliput/maliput_malidrive/issues/125>`_)
  Updates kMaxToleranceSelectionRounds constant from 20 to 24 to guarantee
  a tolerance between 0.05 and 0.50 when automatic tolerance selection is enabled.
* Discards tiny geometries (`#121 <https://github.com/maliput/maliput_malidrive/issues/121>`_)
* Discards function descriptions smaller than constants::kStrictTolerance. (`#118 <https://github.com/maliput/maliput_malidrive/issues/118>`_)
* Removes ament_target_dependencies from libraries. (`#122 <https://github.com/maliput/maliput_malidrive/issues/122>`_)
* Allows functions with NaN values only when they are discardable. (`#116 <https://github.com/maliput/maliput_malidrive/issues/116>`_)
* Piecwise GroundCurve: Use epsilon instead of tolerance at range validation. (`#119 <https://github.com/maliput/maliput_malidrive/issues/119>`_)
* Fixes bugs when creating LaneOffset and LaneWidth. (`#115 <https://github.com/maliput/maliput_malidrive/issues/115>`_)
* Functions starting at the very end of the road are discarded. (`#114 <https://github.com/maliput/maliput_malidrive/issues/114>`_)
* Enable foxy (`#113 <https://github.com/maliput/maliput_malidrive/issues/113>`_)
* Relaxes epsilon when building piece-wise-defined functions. (`#112 <https://github.com/maliput/maliput_malidrive/issues/112>`_)
* Parser: Allows function descriptions sharing same start point. (`#111 <https://github.com/maliput/maliput_malidrive/issues/111>`_)
* Allows lane links inconsistency when semantic errors are allowed. (`#102 <https://github.com/maliput/maliput_malidrive/issues/102>`_)
* Improves some throw messages. (`#103 <https://github.com/maliput/maliput_malidrive/issues/103>`_)
* Avoids error when logging an XML node that has curly braces. (`#101 <https://github.com/maliput/maliput_malidrive/issues/101>`_)
* Fixes body test style in road_geometry_builder_test (`#99 <https://github.com/maliput/maliput_malidrive/issues/99>`_)
* non-drivable lanes are always built but hidden if needed(`#97 <https://github.com/maliput/maliput_malidrive/issues/97>`_)
* Use --include-eol-distros with rosdep update (`#98 <https://github.com/maliput/maliput_malidrive/issues/98>`_)
* OpenRangeValidator: relative epsilon. (`#92 <https://github.com/maliput/maliput_malidrive/issues/92>`_)
* Improve some RoadGeometryBuilder log messages (`#91 <https://github.com/maliput/maliput_malidrive/issues/91>`_)
* Fix include style part 3: reorder headers (`#84 <https://github.com/maliput/maliput_malidrive/issues/84>`_)
* Fixes format.
* Require OpenDRIVE file in RoadGeometryConfiguration
* pybind11 is not needed in CI (`#86 <https://github.com/maliput/maliput_malidrive/issues/86>`_)
* Use python3 for check_test_ran script (`#85 <https://github.com/maliput/maliput_malidrive/issues/85>`_)
* Include XODR file name in error message (`#87 <https://github.com/maliput/maliput_malidrive/issues/87>`_)
  Include the XODR file name, if possible, when no workable tolerance
  is found. Useful for debugging purposes.
* Omit non driveable lanes (`#79 <https://github.com/maliput/maliput_malidrive/issues/79>`_)
* Improve the loader by opting for the automatic tolerance selection by default (`#77 <https://github.com/maliput/maliput_malidrive/issues/77>`_)
* Fix include style part 2: <> for drake, add/remove newlines (`#81 <https://github.com/maliput/maliput_malidrive/issues/81>`_)
* Fix include style part 1: use <> for maliput/ includes (`#80 <https://github.com/maliput/maliput_malidrive/issues/80>`_)
* CI: Removes prereqs install for drake. (`#76 <https://github.com/maliput/maliput_malidrive/issues/76>`_)
* Fixes segment bounds computation. (`#73 <https://github.com/maliput/maliput_malidrive/issues/73>`_)
* Upgrade ros-tooling to v0.2.1 (`#75 <https://github.com/maliput/maliput_malidrive/issues/75>`_)
* Use maliput_integration instead of maliput-integration. (`#74 <https://github.com/maliput/maliput_malidrive/issues/74>`_)
* Uses maliput_documentation instead of maliput-documentation. (`#72 <https://github.com/maliput/maliput_malidrive/issues/72>`_)
* Pairs hbound value with odrm implementation. (`#71 <https://github.com/maliput/maliput_malidrive/issues/71>`_)
* Differentiate between schema and semantic errors when using strictness policy (`#68 <https://github.com/maliput/maliput_malidrive/issues/68>`_)
* Improves the error message to explain which are the roads involved in… (`#67 <https://github.com/maliput/maliput_malidrive/issues/67>`_)
* Adds verbose error exceptions  (`#66 <https://github.com/maliput/maliput_malidrive/issues/66>`_)
* Lets disconnected Roads within a Junction to exist (`#64 <https://github.com/maliput/maliput_malidrive/issues/64>`_)
* Allows selecting the flexibility of the xodr parser. (`#62 <https://github.com/maliput/maliput_malidrive/issues/62>`_)
* Creates ParserConfiguration struct. (`#61 <https://github.com/maliput/maliput_malidrive/issues/61>`_)
* Polish road_geometry_configuration.h (`#63 <https://github.com/maliput/maliput_malidrive/issues/63>`_)
* Adds parameter to RoadGeometryConfiguration to set xodr parsing strictness (`#59 <https://github.com/maliput/maliput_malidrive/issues/59>`_)
* Parser: Improves log message when missing 'connection' in a junction. (`#60 <https://github.com/maliput/maliput_malidrive/issues/60>`_)
* Switch ament_cmake_doxygen to main. (`#58 <https://github.com/maliput/maliput_malidrive/issues/58>`_)
* Optimizes scan-build run in CI. (`#52 <https://github.com/maliput/maliput_malidrive/issues/52>`_)
* Add changelog template (`#48 <https://github.com/maliput/maliput_malidrive/issues/48>`_)
* Point to maliput_infrastructure instead of dsim-repos-index (`#47 <https://github.com/maliput/maliput_malidrive/issues/47>`_)
* Trigger PR clang builds on do-clang-test label (`#46 <https://github.com/maliput/maliput_malidrive/issues/46>`_)
* Restores scan-build workflow on label (`#45 <https://github.com/maliput/maliput_malidrive/issues/45>`_)
* Implements Inertial to Backend Frame translation (`#44 <https://github.com/maliput/maliput_malidrive/issues/44>`_)
* Moves disabled workflows to a different folder. (`#42 <https://github.com/maliput/maliput_malidrive/issues/42>`_)
* Adds tsan sanitizer workflow in CI (`#39 <https://github.com/maliput/maliput_malidrive/issues/39>`_)
* Parallel build policy set in integration tests. (`#40 <https://github.com/maliput/maliput_malidrive/issues/40>`_)
* Parallelizes the road geometry building process. (`#37 <https://github.com/maliput/maliput_malidrive/issues/37>`_)
* Refer to a specific clang version and use lld linker. (`#36 <https://github.com/maliput/maliput_malidrive/issues/36>`_)
* Matches with plugin extern c methods refactor. (`#35 <https://github.com/maliput/maliput_malidrive/issues/35>`_)
* Update ros-tooling version in CI. (`#34 <https://github.com/maliput/maliput_malidrive/issues/34>`_)
* Fixes ubsan behavior in CI. (`#32 <https://github.com/maliput/maliput_malidrive/issues/32>`_)
* Fixes plugin test failure when running ubsan. (`#33 <https://github.com/maliput/maliput_malidrive/issues/33>`_)
* Fix typo in GetRoadGeometryConfigurationFor() (`#27 <https://github.com/maliput/maliput_malidrive/issues/27>`_)
* Fixes CI's wrong main branch. (`#29 <https://github.com/maliput/maliput_malidrive/issues/29>`_)
* Removes Jenkins configuration. (`#28 <https://github.com/maliput/maliput_malidrive/issues/28>`_)
* Append library dirs to plugin test. (`#26 <https://github.com/maliput/maliput_malidrive/issues/26>`_)
* Restores integration tests and provides a dictionary with per xodr map configurations. (`#23 <https://github.com/maliput/maliput_malidrive/issues/23>`_)
* Removes constraint of `p` being in range of lane_offset's domain. (`#25 <https://github.com/maliput/maliput_malidrive/issues/25>`_)
* Builds non-driveable lanes. (`#22 <https://github.com/maliput/maliput_malidrive/issues/22>`_)
* Adds tests for RoadNetworkLoader maliput_malidrive plugin. (`#21 <https://github.com/maliput/maliput_malidrive/issues/21>`_)
* Implements a maliput RoadNetworkLoader plugin. (`#19 <https://github.com/maliput/maliput_malidrive/issues/19>`_)
* Uses phase based discrete value rule provider (`#20 <https://github.com/maliput/maliput_malidrive/issues/20>`_)
* Merge pull request `#18 <https://github.com/maliput/maliput_malidrive/issues/18>`_ from maliput/agalbachicar/`#361 <https://github.com/maliput/maliput_malidrive/issues/361>`__rename_to_geo_position
  Renames GeoPosition to InertialPosition.
* Merge branch 'main' into agalbachicar/`#361 <https://github.com/maliput/maliput_malidrive/issues/361>`__rename_to_geo_position
* Removes already completed TODO comment.. (`#17 <https://github.com/maliput/maliput_malidrive/issues/17>`_)
* Renames GeoPosition to InertialPosition.
* Updates README file. (`#3 <https://github.com/maliput/maliput_malidrive/issues/3>`_)
* Merge pull request `#2 <https://github.com/maliput/maliput_malidrive/issues/2>`_ from maliput/francocipollone/migrate_maliput_malidrive
  Migrates maliput_malidrive
* Adds GitHub Actions CI and Jenkins configs.
* Fixes header files in include folder for malidrive (`#734 <https://github.com/maliput/maliput_malidrive/issues/734>`_)
* Fixes xodr_file path in yaml files in resources folder. (`#733 <https://github.com/maliput/maliput_malidrive/issues/733>`_)
* Adds integration tests in maliput_malidrive package (`#727 <https://github.com/maliput/maliput_malidrive/issues/727>`_)
* Move header files in maliput_malidrive (`#730 <https://github.com/maliput/maliput_malidrive/issues/730>`_)
* Duplicates maps into malidrive (`#729 <https://github.com/maliput/maliput_malidrive/issues/729>`_)
* Use maliput::test_utilities and try same branch name in actions (`#728 <https://github.com/maliput/maliput_malidrive/issues/728>`_)
* Remove tests using Town0X maps in db_manager_test.cc (`#726 <https://github.com/maliput/maliput_malidrive/issues/726>`_)
* Build shared libs in maliput_malidrive. (`#725 <https://github.com/maliput/maliput_malidrive/issues/725>`_)
* Adds interface library.
* Moves xodr apps to maliput_malidrive package.
* Moves loader to maliput_malidrive package (`#716 <https://github.com/maliput/maliput_malidrive/issues/716>`_)
* Removes proj4 (`#715 <https://github.com/maliput/maliput_malidrive/issues/715>`_)
* Remove extra malidrive prefix in files and classes. (`#714 <https://github.com/maliput/maliput_malidrive/issues/714>`_)
* Moves builder folder to maliput_malidrive package (`#711 <https://github.com/maliput/maliput_malidrive/issues/711>`_)
* Moves base folder to maliput_malidrive (`#710 <https://github.com/maliput/maliput_malidrive/issues/710>`_)
* Moves id_providers to maliput_malidrive package (`#709 <https://github.com/maliput/maliput_malidrive/issues/709>`_)
* Moves xodr folder to maliput_malidrive package  (`#707 <https://github.com/maliput/maliput_malidrive/issues/707>`_)
* Moves malidrive_road_curve.h/cc to road_curve.h/cc (`#705 <https://github.com/maliput/maliput_malidrive/issues/705>`_)
* Moves utility folder to maliput_malidrive package. (`#706 <https://github.com/maliput/maliput_malidrive/issues/706>`_)
* Adapts constants namespace (`#702 <https://github.com/maliput/maliput_malidrive/issues/702>`_)
* Moves malidrive2 files of road_curve to maliput_malidrive (`#697 <https://github.com/maliput/maliput_malidrive/issues/697>`_)
* Improve error logging when parsing. (`#693 <https://github.com/maliput/maliput_malidrive/issues/693>`_)
* Splitts malidrive/constants.h (`#684 <https://github.com/maliput/maliput_malidrive/issues/684>`_)
* Use ament_add_gtest_executable for xodr tests, remove libgtest (`#689 <https://github.com/maliput/maliput_malidrive/issues/689>`_)
* Enable maliput_malidrive in Github Actions (`#683 <https://github.com/maliput/maliput_malidrive/issues/683>`_)
* Moves macro_test.cc to maliput_malidrive package (`#673 <https://github.com/maliput/maliput_malidrive/issues/673>`_)
* Improves logging (`#675 <https://github.com/maliput/maliput_malidrive/issues/675>`_)
* Moves test_utilities folder to maliput_malidrive package (`#674 <https://github.com/maliput/maliput_malidrive/issues/674>`_)
* Move macros.h folder to maliput_malidrive (`#670 <https://github.com/maliput/maliput_malidrive/issues/670>`_)
* Renaming maliput-malidrive to maliput_malidrive (`#672 <https://github.com/maliput/maliput_malidrive/issues/672>`_)
* first commit
* Contributors: Agustin Alba Chicar, Chien-Liang Fok, Franco Cipollone, Geoffrey Biggs, Liang Fok, Steve Peters, Voldivh
