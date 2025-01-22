cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_COMPILER "acpp")

project(AdaptorTests LANGUAGES CXX)

find_package(syclbvh CONFIG REQUIRED)


configure_file(kernels/config.h.in ${CMAKE_SOURCE_DIR}/kernels/config.h)

add_executable(minimal tutorials/minimal/minimal.cpp)
add_executable(pathtracer tutorials/pathtracer/pathtracer.cpp
                          tutorials/common/tutorial/tutorial.cpp
                          tutorials/common/tutorial/benchmark.cpp
                          tutorials/common/tutorial/noise.cpp
                          tutorials/common/tutorial/tasksys.cpp  
                          tutorials/common/tutorial/application.cpp
                          tutorials/common/tutorial/scene.cpp
                          tutorials/common/tutorial/scene_device.cpp
#                          tutorials/common/tutorial/tutorial_device.cpp
                          tutorials/common/image/image.cpp
                          tutorials/common/image/exr.cpp
                          tutorials/common/image/ppm.cpp
                          tutorials/common/image/stb.cpp
                          tutorials/common/image/pfm.cpp
                          tutorials/common/image/tga.cpp
                          tutorials/common/scenegraph/obj_loader.cpp
                          tutorials/common/scenegraph/scenegraph.cpp
                          tutorials/common/scenegraph/texture.cpp
                          tutorials/common/scenegraph/corona_loader.cpp
                          tutorials/common/scenegraph/geometry_creation.cpp
                          tutorials/common/scenegraph/ply_loader.cpp
                          tutorials/common/scenegraph/xml_loader.cpp
                          tutorials/common/scenegraph/xml_parser.cpp
                          tutorials/common/scenegraph/xml_writer.cpp
                          common/sys/alloc.cpp
                          common/sys/barrier.cpp
                          common/sys/condition.cpp
                          common/sys/estring.cpp
                          common/sys/filename.cpp
                          common/sys/mutex.cpp
                          common/sys/regression.cpp
                          common/sys/sysinfo.cpp
                          common/sys/thread.cpp
                          )


add_compile_definitions(TASKING_TBB=ON)
add_compile_definitions(EMBREE_MAX_INSTANCE_LEVEL_COUNT=1)

target_link_libraries(minimal PRIVATE syclbvh::syclbvh)
target_link_libraries(pathtracer PRIVATE syclbvh::syclbvh)
