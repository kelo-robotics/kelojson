# KELO JSON

[![build status](https://git.locomotec.com:444/kelo/mapping/kelojson/badges/master/build.svg)](https://git.locomotec.com:444/kelo/mapping/kelojson/commits/master)

Package to load `.kelojson` map data.

## Dependencies

- [geometry_common](https://git.locomotec.com:444/kelo/common/geometry_common)
- [yaml_common](https://git.locomotec.com:444/kelo/common/yaml_common)

## Documentation

We use [Doxygen](https://www.doxygen.nl/index.html) for code documentation.

**Note**: Building the documentation is disabled by default.

- To build the code documentation, Doxygen needs to be installed. On debian based
  systems, this can be achieved with
  ```bash
  sudo apt install doxygen
  ```

- Documentation can be built using the flag `-DBUILD_DOC=ON`
  ```bash
  catkin build kelo_navigation -DBUILD_DOC=ON
  ```

- The documentation will be generated at
  `<YOUR_CATKIN_WS>/build/kelojson/docs/html/index.html`

## Test

Run unit tests with

```bash
catkin build --this --catkin-make-args run_tests -- && rosrun kelojson kelojson_test
```

**Note**: Requires `GTest` package (`sudo apt install libgtest-dev`)
