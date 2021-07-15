cmake_minimum_required(VERSION 3.4...3.18)
project(orbslam2)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_subdirectory(pybind11)

find_package(DBoW2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM2 REQUIRED)

pybind11_add_module(orbslam2 
src/py_orbslam2.cpp
)

target_link_libraries(orbslam2
PRIVATE
Eigen3::Eigen
${OpenCV_LIBS}
${DBoW2_LIBRARIES}
${Pangolin_LIBRARIES}
${ORB_SLAM2_LIBRARIES}
)

target_compile_definitions(orbslam2 
PRIVATE 
VERSION_INFO=${EXAMPLE_VERSION_INFO}
)
