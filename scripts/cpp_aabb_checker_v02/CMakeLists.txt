cmake_minimum_required(VERSION 3.18)
project(cpp_aabb_checker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(cpp_aabb_checker checker.cpp)

if (MSVC)
    target_compile_options(cpp_aabb_checker PRIVATE /O2)
else()
    target_compile_options(cpp_aabb_checker PRIVATE -O3 -march=native)
endif()
