cmake_minimum_required(VERSION 3.16)
project(geoproc)

find_package(OpenMP REQUIRED)

option(GEOPROC_DISABLE_ASAN "" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# We use generator expressions to only enable ASAN in "Release with debug info"
# config and support multi-config generators like the Visual Studio generator
# and Ninja Multiconfig, because ASAN is not compatible with Debug flags, nor is
# it much useful in "Release" only mode without debug info, and because
# CMAKE_BUILD_TYPE does not get populated automatically when using a
# multi-config generator we use generator expressions
if (NOT GEOPROC_DISABLE_ASAN)
    if (MSVC)
        add_compile_options($<$<CONFIG:RelWithDebInfo>:/fsanitize=address>)
        # Disable incremental linking because MSVC ASAN is not compatible with
        # it.
        add_link_options($<$<CONFIG:RelWithDebInfo>:/INCREMENTAL:NO>)
    else ()
        add_compile_options($<$<CONFIG:RelWithDebInfo>:-fsanitize=address
                            -fno-omit-frame-pointer>)
        add_link_options($<$<CONFIG:RelWithDebInfo>:-fsanitize=address>)
    endif ()
endif ()

add_library(mesh_io mesh_io.cpp)
target_compile_features(mesh_io PRIVATE cxx_std_17)

add_library(write_ply write_ply.cpp)
target_compile_features(write_ply PRIVATE cxx_std_17)

add_library(intersect intersect.cpp)
target_compile_features(intersect PRIVATE cxx_std_17)

add_library(distance distance.cpp)
target_compile_features(distance PRIVATE cxx_std_17)

add_library(bvh bvh.cpp)
target_link_libraries(bvh PRIVATE distance)
target_compile_features(bvh PRIVATE cxx_std_17)

add_executable(sample_volume sample_volume.cpp)
target_link_libraries(sample_volume mesh_io write_ply intersect bvh
                      OpenMP::OpenMP_CXX)
target_compile_features(sample_volume PRIVATE cxx_std_17)

add_executable(sample_surface sample_surface.cpp)
target_link_libraries(sample_surface mesh_io write_ply)
target_compile_features(sample_surface PRIVATE cxx_std_17)

add_executable(fixed_point_demo fixed_point_demo.cpp)
target_compile_features(fixed_point_demo PRIVATE cxx_std_17)

add_executable(sample_cube sample_cube.cpp)
target_link_libraries(sample_cube write_ply)
target_compile_features(sample_cube PRIVATE cxx_std_17)

add_executable(mesh_boolean mesh_boolean.cpp)
target_link_libraries(mesh_boolean mesh_io intersect bvh)
target_compile_features(mesh_boolean PRIVATE cxx_std_17)

add_executable(delaunay delaunay.cpp)
target_link_libraries(delaunay bvh distance)
target_compile_features(delaunay PRIVATE cxx_std_17)

add_executable(project_surface project_surface.cpp)
target_link_libraries(project_surface mesh_io write_ply)
target_compile_features(project_surface PRIVATE cxx_std_17)

include(CTest)

add_executable(test_intersection intersect_test.cpp)
target_link_libraries(test_intersection PRIVATE intersect)
target_compile_features(test_intersection PRIVATE cxx_std_17)
add_test(NAME test_intersection COMMAND test_intersection)

add_executable(test_distance distance_test.cpp)
target_link_libraries(test_distance PRIVATE distance)
target_compile_features(test_distance PRIVATE cxx_std_17)
add_test(NAME test_distance COMMAND test_distance)
