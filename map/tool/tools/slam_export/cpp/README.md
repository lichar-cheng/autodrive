# C++ SLAM Export Tool

This module provides in-memory and file export APIs for `.slam` archives.

## Configure And Build

```bash
cd tools/slam_export/cpp
cmake -S . -B build
cmake --build build
```

## Run Tests

```bash
cd tools/slam_export/cpp
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
