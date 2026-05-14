cmake_minimum_required(VERSION 3.14)

# 1. Standardize project definition block
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(robot_platform_utils LANGUAGES C CXX)
endif()

# Enforce uniform C++ guidelines
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################### Architecture Mapping ###########################################
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    message(STATUS "Building for Linux-ARM architecture")
    set(LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/../python/lib/arm64")
else() 
    message(STATUS "Building for Linux-x86 architecture")
    set(LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/../python/lib/x86")
endif()

################################### Build Switches ###########################################
option(BUILD_LIB "Build Core Library" ON)
option(BUILD_PYTHON_LIB "Build Python Extension Library" OFF)
option(BUILD_CPP_TESTS "Build C++ Test Infrastructure" OFF)

################################### Core Library Target ###########################################
if(BUILD_LIB OR BUILD_CPP_TESTS)
    message(STATUS "Configuring Library: robot_platform_utils")
    cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH PARENT_DIR)
    cmake_path(GET PARENT_DIR PARENT_PATH PARENT_PARENT_DIR)
    
    find_package(yaml-cpp REQUIRED)

    # Cleaned-up sibling repository lookups using fallback search paths
    if(NOT TARGET curi::udp)
        add_subdirectory("${PARENT_PARENT_DIR}/curi_udp/c" "${CMAKE_BINARY_DIR}/curi_udp")
    endif()

    if(NOT TARGET curi::tcp)
        add_subdirectory("${PARENT_PARENT_DIR}/curi_tcp/c" "${CMAKE_BINARY_DIR}/curi_tcp")
    endif()

    file(GLOB LIB_SOURCES "src/*.cpp")
    set(LIB_SOURCES
        ${LIB_SOURCES}
    )

    add_library(robot_platform_utils ${LIB_SOURCES})
    
    # Modernized modern target aliases
    add_library(robot::platform_utils ALIAS robot_platform_utils)

    # Use namespaced linkages where possible
    target_link_libraries(robot_platform_utils PUBLIC 
        yaml-cpp::yaml-cpp
        curi::udp
        curi::tcp
    )
    
    target_include_directories(robot_platform_utils PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
endif()


################################### Python Extension Module ###########################################
if(BUILD_PYTHON_LIB)
    message(STATUS "Configuring Python Bindings...")
    
    # --- Use python3 in the same environment ---
    execute_process(
        COMMAND which python3
        OUTPUT_VARIABLE PYTHON_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(Python3_EXECUTABLE ${PYTHON_PATH} CACHE FILEPATH "Python 3 interpreter")
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    # --- Force pybind11 from the same environment ---
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
        OUTPUT_VARIABLE pybind11_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    find_package(pybind11 REQUIRED CONFIG PATHS ${pybind11_DIR})

    set(PYTHON_SOURCES
        src/pybind11/bindings.cpp 
        src/config_loader.cpp
    )

    pybind11_add_module(config_loader ${PYTHON_SOURCES})

    target_link_libraries(config_loader PRIVATE yaml-cpp::yaml-cpp)
    target_include_directories(config_loader PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")

    set_target_properties(config_loader PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${LIB_PATH}"
    )
endif()

################################### Test Suite ###########################################
if(BUILD_CPP_TESTS)
    message(STATUS "Configuring C++ Testing Executables...")
    
    set(TEST_NAMES 
        test_config_loader 
        test_mcast_client1 
        test_mcast_client2 
        test_mcast_priority_server
    )

    foreach(test_target ${TEST_NAMES})
        add_executable(${test_target} "${CMAKE_CURRENT_LIST_DIR}/test/${test_target}.cpp")
        target_link_libraries(${test_target} PRIVATE robot::platform_utils)
    endforeach()
endif()