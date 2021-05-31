# maliput_malidrive

A maliput backend.

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
