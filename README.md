[![gcc](https://github.com/maliput/maliput_malidrive/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/maliput_malidrive/actions/workflows/build.yml)

# maliput_malidrive

## Description

`maliput_malidrive` package is a [Maliput](https://github.com/maliput/maliput) backend implementation.
Its underlying format specification is based on [OpenDRIVE format](https://www.asam.net/standards/detail/opendrive/). Allowing the users to load a road network out of a XODR file description.

Please go to [XODR Parser capabilities](src/maliput_malidrive/xodr/README.md) for further information about OpenDRIVE's parser.

**Note**: For full information about Maliput please visit [Maliput Documentation](https://maliput.readthedocs.io/en/latest/index.html).

### Resources

`maliput_malidrive` provides several map resources at [maliput_malidrive/resources](resources).
These resources are:
 - XODR files for describing different road networks using OpenDRIVE format specification.
 - YAML files for describing `maliput`'s road network information of type: Range Value Rules, Discrete Value Rules, Traffic Lights, Phase Rings, Intersections, etc.

Resources are installed natively, so the users are able to use them for their own interest.
In order to get the installation path check the environment variable: `MALIPUT_MALIDRIVE_RESOURCE_ROOT`.

## API Documentation

Refer to [Maliput Malidrive's Online API Documentation](https://maliput.readthedocs.io/en/latest/html/deps/maliput_malidrive/html/index.html).

## Examples

[Getting Started](https://maliput.readthedocs.io/en/latest/getting_started.html) page is a good place for starting to see Maliput's capabilities and how to use a Maliput backend for getting a road network.

 - [maliput_malidrive's applications](https://github.com/maliput/maliput_malidrive/blob/francocipollone/improve_readme/src/applications): This package provides some applications to validate a XODR file. See [maliput_malidrive's tutorials](https://maliput.readthedocs.io/en/latest/html/deps/maliput_malidrive/html/tutorials.html).
 - [maliput_integration](https://github.com/maliput/maliput_integration): Concentrates applications created for maliput. See [maliput_integration's tutorials](https://maliput.readthedocs.io/en/latest/html/deps/maliput_integration/html/integration_tutorials.html). These applications allow to select `maliput_malidrive` as the backend.

## Installation

### Supported platforms

Ubuntu Focal Fossa 20.04 LTS.

### Binary Installation on Ubuntu

See [Installation Docs](https://maliput.readthedocs.io/en/latest/installation.html#binary-installation-on-ubuntu).

### Source Installation on Ubuntu

#### Prerequisites

```
sudo apt install python3-rosdep python3-colcon-common-extensions
```

#### Build

1. Create colcon workspace if you don't have one yet.
    ```sh
    mkdir colcon_ws/src -p
    ```

2. Clone this repository in the `src` folder
    ```sh
    cd colcon_ws/src
    git clone https://github.com/maliput/maliput_malidrive.git
    ```

3. Install package dependencies via `rosdep`
    ```
    export ROS_DISTRO=foxy
    ```
    ```sh
    rosdep update
    rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src
    ```

4. Build the package
    ```sh
    colcon build --packages-up-to maliput_malidrive
    ```

    **Note**: To build documentation a `-BUILD_DOCS` cmake flag is required:
    ```sh
    colcon build --packages-select maliput_malidrive --cmake-args " -DBUILD_DOCS=On"
    ```
    More info at [Building Documentation](https://maliput.readthedocs.io/en/latest/developer_guidelines.html#building-the-documentation).

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)


### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/maliput_malidrive/blob/main/LICENSE)
