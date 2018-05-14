cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_align)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ./src/libForPCL/include)
link_directories(${PCL_LIBRARY_DIRS})

aux_source_directory(./src/libForPCL/src lib_src_new)
aux_source_directory(./src current_src_new)

add_definitions(${PCL_DEFINITIONS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(cluster_align ${current_src_new} ${lib_src_new})
target_link_libraries(cluster_align ${PCL_LIBRARIES})

