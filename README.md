## IsaacSim XR Teleop Device Plugins — Hand Tracking (Manus example)

This repository provides a Linux-only plugin framework that bridges the Isaac Sim Hand Tracking C API to vendor SDKs. It includes:
- A shared library implementing the Isaac Sim Hand Tracking C API
- An example Manus-backed plugin that streams glove data through that API

### Components
- **Core C API library**: `IsaacSimHandTrackerCAPI` (shared library)
- **Manus example plugin**: `IsaacSimManusHandTracking` (shared library)
- **CLI**: `ManusHandTrackerPrinter` (prints tracked data)

All targets use modern CMake and C++20.

## Prerequisites
- **Linux** (x86_64)
- **CMake** ≥ 3.22 (Ubuntu 22.04 default is supported)
- **GCC/Clang** with C++20 support
- **Manus SDK** for Linux x86_64 (headers and shared libraries)

Note: This project does not redistribute Manus SDK binaries. Provide your own Manus SDK to build and run the Manus example plugin.

## Getting the Manus SDK

The Manus SDK must be downloaded separately due to licensing.

1. Obtain a Manus account and credentials
2. Download the MANUS Core SDK from `https://my.manus-meta.com/resources/downloads` (tested with 2.5.1)
3. Extract and place `ManusSDK/` under `vendor/manus/`, or set `MANUS_SDK_ROOT` to your install

Expected layout when vendored:
```
vendor/manus/
  ManusHandTrackingPlugin.cpp
  ManusHandTrackingPlugin.h
  ManusSDK/
    include/           # Manus headers (ManusSDK.h, etc.)
    lib/ or lib64/     # Manus shared libs (libManusSDK.so or libManusSDK_Integrated.so)
```

## Build

Presets are provided:

- Default (RelWithDebInfo):
```bash
cmake --preset linux-manus-default
cmake --build --preset manus-default -j
```
- Debug:
```bash
cmake --preset linux-manus-debug
cmake --build --preset manus-debug -j
```
- Release:
```bash
cmake --preset linux-manus-release
cmake --build --preset manus-release -j
```

Artifacts are placed under your build directory (e.g., `build-manus-default`):
- `bin/ManusHandTrackerPrinter`
- `lib/libIsaacSimHandTrackerCAPI.so` (+ SONAME/version symlinks)
- `lib/libIsaacSimManusHandTracking.so`


## Using with Isaac Sim
- The Manus plugin implements the Isaac Sim Hand Tracking C API and is intended for the `isaacsim.xr.input_devices` extension.
- Load the built shared libraries from your Isaac Sim integration.
- Ensure the Manus SDK shared libraries are available via RPATH or `LD_LIBRARY_PATH`.

## Advanced
- Custom Manus SDK location:
```bash
MANUS_SDK_ROOT=/opt/ManusSDK cmake --preset linux-manus-default
cmake --build --preset manus-default -j
```
- Disable Manus plugin (build C API only):
```bash
cmake -S . -B build-core -DBUILD_MANUS_PLUGIN=OFF
cmake --build build-core -j
```

## Troubleshooting
- Missing `libIsaacSimHandTrackerCAPI.so` at runtime: run from `build-*/bin` or `install/bin`; avoid stale binaries under nested folders.
- Manus SDK not found at runtime: set `LD_LIBRARY_PATH` to your Manus SDK `lib/` (or bundle via the opt-in flag for builds).
- CMake cannot locate Manus headers/libs: verify `MANUS_SDK_ROOT`, or place `ManusSDK/` under `vendor/manus/` as shown above.

## Contributing
Contributions are welcome! Please see `CONTRIBUTING.md` for guidelines, coding style, and CI details.

## License
Source files are under their stated licenses. The Manus SDK is proprietary to Manus and is subject to its own license; it is not redistributed by this project.
