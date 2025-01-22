cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_COMPILER "acpp")
project(syclbvh LANGUAGES CXX)

# Create the library
add_library(syclbvh STATIC
    src/syclbvh.cpp
)

# Make headers available when users #include <mylib/somefile.h>
target_include_directories(syclbvh PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Export the target to allow find_package usage
install(TARGETS syclbvh
        EXPORT syclbvhConfig
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install public headers
install(DIRECTORY include/
        DESTINATION include)

# Export the targets file for find_package
install(EXPORT syclbvhConfig
        NAMESPACE syclbvh::
        DESTINATION lib/cmake/syclbvh
        FILE syclbvhConfig.cmake)