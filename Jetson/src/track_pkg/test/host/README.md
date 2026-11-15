# Tracking host tests

The tests compile the production OC-SORT and value adapter without ROS or device
access. Dependencies are Eigen 3.x and GoogleTest, provided by the host or through
`CMAKE_PREFIX_PATH`; no dependency sources are vendored here.

```sh
cmake -S Jetson/src/track_pkg/test/host -B work/track-tests -DCMAKE_BUILD_TYPE=Release
cmake --build work/track-tests
ctest --test-dir work/track-tests --output-on-failure
```

The same test sources are registered by the ROS package when `BUILD_TESTING` is
enabled. Tests cover OC-SORT association/occlusion behavior and exact integer IDs
through the production adapter, including the 2^24, 2^31 and 2^53 boundaries.
