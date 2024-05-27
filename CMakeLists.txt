cmake_minimum_required(VERSION 3.16)
project(geoproc)

option(GEOPROC_DISABLE_ASAN "" OFF)

if (NOT GEOPROC_DISABLE_ASAN)
    if (MSVC)
        add_compile_options(/fsanitize=address)
        add_link_options(/INCREMENTAL:NO) # Disable incremental linking because MSVC ASAN is not compatible with it.
    else ()
        add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address)
    endif ()
endif ()

add_library(mesh_io mesh_io.cpp)
target_compile_features(mesh_io PRIVATE cxx_std_17)

add_library(write_ply write_ply.cpp)
target_compile_features(write_ply PRIVATE cxx_std_17)

add_executable(sample_surface sample_surface.cpp)
target_link_libraries(sample_surface mesh_io write_ply)
target_compile_features(sample_surface PRIVATE cxx_std_17)

add_executable(sample_volume sample_volume.cpp)
target_link_libraries(sample_volume mesh_io write_ply)
target_compile_features(sample_volume PRIVATE cxx_std_17)

add_executable(fixed_point_demo fixed_point_demo.cpp)
target_compile_features(fixed_point_demo PRIVATE cxx_std_17)
