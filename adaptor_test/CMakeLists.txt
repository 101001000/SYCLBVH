cmake_minimum_required(VERSION 3.16)
project(adaptortest LANGUAGES CXX)

# Locate the installed mylib
find_package(syclbvh CONFIG REQUIRED)

add_executable(adaptortest minimal.cpp)
target_link_libraries(adaptortest PRIVATE syclbvh::syclbvh)
