# robot_platform_utils

`robot_platform_utils` contains shared C++ and Python utilities used by CuArm-related repositories.

## What This Repo Includes

- C++ utility library and headers in `cpp/include` and `cpp/src`
- YAML-based configuration loading/writing helpers (for example `config_loader`)
- Networking-related utilities that integrate with `curi_udp` and `curi_tcp`
- Optional C++ test targets in `cpp/test`
- Optional Python extension build via `pybind11`
- Python helper scripts and tests in `python/scripts` and `python/test`

## Build Targets

The CMake file in `cpp/CMakeLists.txt` defines:

- `robot_platform_utils` (default)
- `robot::platform_utils` (alias target)
- Optional Python module build with `-DBUILD_PYTHON_LIB=ON`
- Optional C++ tests with `-DBUILD_CPP_TESTS=ON`

## How Other Repos Use This Repo via `add_subdirectory`

From a parent project CMake, include this repo's CMake directory and link against
`robot::platform_utils` (or `robot_platform_utils`):

```cmake
add_subdirectory(${CMAKE_SOURCE_DIR}/../robot_platform_utils/cpp ${CMAKE_BINARY_DIR}/robot_platform_utils)
target_link_libraries(your_target PRIVATE robot::platform_utils)
```

## Required Layout for Dependencies

`robot_platform_utils/cpp/CMakeLists.txt` automatically calls `add_subdirectory` for sibling
repositories `curi_udp/c` and `curi_tcp/c` (if targets `curi::udp` and `curi::tcp`
are not already defined), so a typical layout is:

Dependency repositories:

- `curi_udp`: [https://github.com/CURI-Simulation-and-Software-Group/curi_udp](https://github.com/CURI-Simulation-and-Software-Group/curi_udp)
- `curi_tcp`: [https://github.com/CURI-Simulation-and-Software-Group/curi_tcp](https://github.com/CURI-Simulation-and-Software-Group/curi_tcp)

```text
<workspace>/
  robot_platform_utils/
  curi_udp/
  curi_tcp/
```

## Build

### Build C++ tests

```bash
cd cpp
mkdir -p build && cd build
cmake -DBUILD_CPP_TESTS=ON ..
make
```

### Build Python module

1. Install `pybind11`

```bash
pip install pybind11
```

2. Build from C++ source

```bash
cd cpp
mkdir -p build && cd build
cmake -DBUILD_PYTHON_LIB=ON -DPython3_EXECUTABLE=$(which python3) ..
make
```

The built Python module (`config_loader`) is saved under `python/lib`, in an
architecture-specific subfolder such as `python/lib/x86`, `python/lib/arm64`.