cmake_minimum_required(VERSION 2.8)

include(cmake/PICMake.cmake)

set(CMAKE_CXX_STANDARD 11)
pi_add_target(gslam_dso SHARED src REQUIRED GSLAM Eigen3 OpenGL GLEW GLUT DEPENDENCY boost_system boost_thread csparse cxsparse)

pi_report_target()

INCLUDE_DIRECTORIES(src)

pi_install(TARGETS gslam_dso LIB_DESTINATION lib)
