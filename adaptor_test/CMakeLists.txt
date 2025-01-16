cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsycl")

project(adaptortest LANGUAGES CXX)

find_package(syclbvh CONFIG REQUIRED)

add_executable(adaptortest minimal.cpp)
target_link_libraries(adaptortest PRIVATE syclbvh::syclbvh)
