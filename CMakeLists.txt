cmake_minimum_required(VERSION 3.16)
project(ros2_gst_meta
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Dependencies -----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0
    gstreamer-base-1.0
)

# Fetch gst-metadata (header-only library)
include(FetchContent)
set(GST_METADATA_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GST_METADATA_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(gst-metadata
    GIT_REPOSITORY https://github.com/PavelGuzenfeld/gst-metadata.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(gst-metadata)

# --- Sanitizers -------------------------------------------------------------
option(ROS2_GST_META_SANITIZER_ASAN "Enable ASan + UBSan" OFF)
option(ROS2_GST_META_SANITIZER_TSAN "Enable TSan" OFF)

if(ROS2_GST_META_SANITIZER_ASAN AND ROS2_GST_META_SANITIZER_TSAN)
    message(FATAL_ERROR "ASan and TSan cannot be used together")
endif()

if(ROS2_GST_META_SANITIZER_ASAN)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

if(ROS2_GST_META_SANITIZER_TSAN)
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
endif()

# --- GStreamer plugin -------------------------------------------------------
add_library(gstros2meta SHARED
    src/ros2_attach.cpp
    src/ros2_detach.cpp
    src/plugin.cpp
)
target_include_directories(gstros2meta PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(gstros2meta PRIVATE
    gst-metadata
    PkgConfig::GST
)
ament_target_dependencies(gstros2meta PUBLIC rclcpp)

target_compile_definitions(gstros2meta PRIVATE PACKAGE="ros2-gst-meta")
set_target_properties(gstros2meta PROPERTIES PREFIX "libgst" OUTPUT_NAME "ros2meta")

# Install plugin to both ament and GStreamer plugin dirs
install(TARGETS gstros2meta
    LIBRARY DESTINATION lib
)

# --- Headers (for downstream) -----------------------------------------------
install(DIRECTORY include/ros2_gst_meta/
    DESTINATION include/ros2_gst_meta
)

# --- Tests ------------------------------------------------------------------
option(ROS2_GST_META_BUILD_TESTS "Build tests" ON)
if(ROS2_GST_META_BUILD_TESTS)
    enable_testing()

    FetchContent_Declare(doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG v2.4.11
    )
    FetchContent_MakeAvailable(doctest)

    add_executable(test_sync_buffer test/test_sync_buffer.cpp)
    target_include_directories(test_sync_buffer PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_sync_buffer PRIVATE
        gst-metadata
        doctest::doctest
        PkgConfig::GST
    )
    add_test(NAME sync_buffer COMMAND test_sync_buffer)
endif()

ament_package()
