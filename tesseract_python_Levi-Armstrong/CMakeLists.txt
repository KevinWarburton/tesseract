cmake_minimum_required(VERSION 2.8.3)
#project(tesseract_python)
project(tesseract_python_Levi-Armstrong)


add_compile_options(-std=c++11 -Wall -Wextra -Wno-write-strings)

find_package(catkin REQUIRED COMPONENTS
#  NOTE_TO KEVIN: ADD THREE PACKAGES BELOW LATER
#  tesseract_ros
#  tesseract_core
#  tesseract_msgs

#  Both packages below moved to tesseract_core
#  tesseract_collision
#  tesseract_planning
  roscpp
  urdf
  srdfdom
  geometric_shapes
  kdl_parser
  eigen_conversions
  pluginlib
#  NOTE_TO KEVIN: ADD TRAJOPT BACK LATER
#  trajopt

#  New stuff that KEVIN is changing below:
  tesseract_geometry
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(octomap REQUIRED)
find_package(orocos_kdl REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

INCLUDE(FindSWIG)
# INCLUDE(/home/ros-industrial/repos_KevinWarburton/swig/install_test/bin/ccache-swig)
FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(UseSWIG)

catkin_package(      
  CATKIN_DEPENDS
#    tesseract_ros
#    tesseract_core
#    tesseract_msgs
#    tesseract_collision
    roscpp
    urdf
    srdfdom
    geometric_shapes
    kdl_parser
    eigen_conversions
    pluginlib
  #  New stuff that KEVIN is changing below:
    tesseract_geometry
  DEPENDS
    EIGEN3
    Boost
    OCTOMAP
    orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
  SYSTEM ${PYTHON_INCLUDE_DIRS}
)


SET(SWIG_CXX_EXTENSION cxx)
set_property(SOURCE swig/tesseract_python.i PROPERTY CPLUSPLUS ON)
set_property(SOURCE swig/tesseract_python.i PROPERTY SWIG_FLAGS -relativeimport -threads)

set(CMAKE_SWIG_OUTDIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/tesseract)

SWIG_ADD_MODULE(tesseract_python python swig/tesseract_python.i)
SWIG_LINK_LIBRARIES(tesseract_python ${PYTHON_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})

set_target_properties(${SWIG_MODULE_tesseract_python_REAL_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/tesseract)

catkin_python_setup()

install(TARGETS ${SWIG_MODULE_tesseract_python_REAL_NAME}
  LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/tesseract
)

