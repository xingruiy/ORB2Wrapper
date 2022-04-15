cmake_minimum_required(VERSION 3.4...3.18)
project(ORB2Wrapper)

LIST(APPEND CMAKE_MODULE_PATH 
    ${PROJECT_SOURCE_DIR}/cmake
)

add_subdirectory(pybind11)

find_package(DBoW2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM2 REQUIRED)

pybind11_add_module(ORB2Wrapper 
    src/ORB2Wrapper.cpp
    src/NDArrayConverter.cpp
    ORB_SLAM2/src/Frame.cc
)

target_link_libraries(ORB2Wrapper
PRIVATE
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${DBoW2_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${ORB_SLAM2_LIBRARIES}
)

target_compile_definitions(ORB2Wrapper 
PRIVATE 
    VERSION_INFO=${EXAMPLE_VERSION_INFO}
)

target_include_directories(ORB2Wrapper
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ORB_SLAM2>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ORB_SLAM2/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ORB_SLAM2/Thirdparty/DBoW2/DBoW2>
)