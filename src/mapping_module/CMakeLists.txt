cmake_minimum_required(VERSION 3.10)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_library(mapping_core STATIC
    src/Mapping.cpp
)

target_include_directories(mapping_core PUBLIC include)
target_link_libraries(mapping_core ${PCL_LIBRARIES})
