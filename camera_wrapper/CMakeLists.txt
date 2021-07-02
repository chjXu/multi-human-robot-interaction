cmake_minimum_required(VERSION 2.8.3)
project(camera_wrapper)

set(CMAKE_BUILD_TYPE Debug)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  tf
  #openpose_ros_msgs
  #inertial_poser
)
#find_package(OpenCL REQUIRED)
find_package(OpenCV 4 REQUIRED)
INCLUDE(FindPkgConfig)

if(OpenCL_FOUND)
	message(STATUS "OpenCL based depth registration enabled")
	set(EXPORTED_DEPENDENCIES OpenCL)

	if(UNIX AND NOT APPLE)
		include(CheckOpenCLICDLoader)
		if(OpenCL_C_WORKS AND NOT OpenCL_CXX_WORKS)
			set(KINECT2_OPENCL_ICD_LOADER_IS_OLD 1)
			message(WARNING "Your libOpenCL.so is incompatible with CL/cl.h. Install ocl-icd-opencl-dev to update libOpenCL.so?")
		endif()

		# Major Linux distro stable releases have buggy OpenCL ICD loader.
		# The workaround of disabling exceptions can only be set up during compile time.
		# Diabling it for all should be harmless. The flag is the same for GCC/Clang/ICC.
		#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions")
	endif()
    #include_directories(${OpenCL_INCLUDE_DIRS})
    include_directories("/usr/local/cuda-10.0/include/")

	set(DEPTH_REG_OPENCL ON)
	add_definitions(-DDEPTH_REG_OPENCL)
else()
		message(STATUS "OpenCL based depth registration disabled")
		set(DEPTH_REG_OPENCL OFF)
endif()

if(DEPTH_REG_OPENCL)
	add_definitions(-DREG_OPENCL_FILE="${PROJECT_SOURCE_DIR}/src/depth_registration.cl")
	set(MODULES ${MODULES} src/depth_registration.cpp)
    set(MODULE_LIBS ${MODULE_LIBS} ${OPENCL_LIBRARIES})
    message(${OPENCL_LIBRARIES})
    #set(MODULE_LIBS ${MODULE_LIBS} "/usr/local/cuda-10.0/lib64/libOpenCL.so" )
    #set(MODULE_LIBS ${MODULE_LIBS} "/usr/lib/x86_64-linux-gnu/libOpenCL.so" )
endif()

include_directories(
    $ENV{OPENNI2_INCLUDE}
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    )

link_directories(
    $ENV{OPENNI2_REDIST}
    )

link_libraries(
    -lOpenNI2
    ${MODULE_LIBS}
    )

#add_library(depth_registration SHARED src/depth_registration.cpp ${MODULES})

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(image_publisher src/openni_opencv_3.cpp)
#target_link_libraries(image_publisher ${catkin_LIBRARIES})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_wrapper
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/camera_wrapper.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(image_publisher src/openni_opencv_3.cpp)
#add_executable(cpm_listener src/cpm_display.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(image_publisher
#   depth_registration
#   ${catkin_LIBRARIES}
#   ${MODULE_LIBS}
#   ${OpenCV_LIBS})
#add_executable(image_listener src/my_subscriber.cpp)
#target_link_libraries(image_listener ${catkin_LIBRARIES}
#				     ${OpenCV_LIBS})
#target_link_libraries(cpm_listener ${catkin_LIBRARIES}
#				   ${OpenCV_LIBS})

#add_executable(dual_kinect_listener src/dual_kinect.cpp)
#target_link_libraries(dual_kinect_listener ${catkin_LIBRARIES}
#					   ${OpenCV_LIBS})

#add_executable(display_scene src/display_scene.cpp)
#target_link_libraries(display_scene ${catkin_LIBRARIES}
#					   ${OpenCV_LIBS})

#add_executable(tracker src/tracker.cpp)
#target_link_libraries(tracker ${catkin_LIBRARIES}
#					   ${OpenCV_LIBS})
add_executable(multi_cam src/multi_view.cpp)
target_link_libraries(multi_cam ${catkin_LIBRARIES}
					   ${OpenCV_LIBS})
#add_executable(marker_tracker src/multi_view_tracker.cpp)
#target_link_libraries(marker_tracker ${catkin_LIBRARIES}
#					   ${OpenCV_LIBS})
