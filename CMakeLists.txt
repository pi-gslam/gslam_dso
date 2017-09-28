cmake_minimum_required(VERSION 2.8)

include(PICMake)

pi_add_target(gslam_dso SHARED src REQUIRED GSLAM Eigen3 OpenGL GLEW GLUT DEPENDENCY boost_system boost_thread csparse cxsparse)

pi_report_target()

INCLUDE_DIRECTORIES(src)

pi_install(TARGETS gslam_optimizerPBA LIB_DESTINATION lib)
