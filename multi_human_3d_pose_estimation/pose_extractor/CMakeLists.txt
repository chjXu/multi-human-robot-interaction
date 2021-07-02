cmake_minimum_required(VERSION 3.10)
project(pose_extractor)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)
message(STATUS "Python Version: " ${PYTHON_EXECUTABLE})

exec_program(${PYTHON_EXECUTABLE}
            ARGS "-c \"import numpy; print(numpy.get_include())\""
            OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
            RETURN_VALUE NUMPY_NOT_FOUND
            )

if(NUMPY_NOT_FOUND)
    message(FATAL_ERROR "Numpy headers not found")
endif()

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})
if(OPENCV_NOT_FOUND)
    message(FATAL_ERROR "OpenCV not found")
endif()

set(CMAKE_CXX_STANDARD 11)
set(target_name pose_extractor)
add_library(${target_name} SHARED
                wrapper.cpp
                src/extract_poses.hpp
                src/human_pose.hpp
                src/peak.hpp
           )

target_include_directories(${target_name} PRIVATE src/ ${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${target_name} ${PYTHON_LIBRARIES} ${OpenCV_LIBRARIES})

set_target_properties(${target_name} PROPERTIES PREFIX "" OUTPUT_NAME "${target_name}")