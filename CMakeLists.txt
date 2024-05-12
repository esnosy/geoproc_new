cmake_minimum_required(VERSION 3.16)
project(geoproc)

add_executable(sample_surface sample_surface.cpp)
target_compile_features(sample_surface PRIVATE cxx_std_17)
