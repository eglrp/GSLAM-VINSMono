cmake_minimum_required(VERSION 2.8.3)
project(self_calibration_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

include(PICMake)
INCLUDE_DIRECTORIES(src/camera_model/include)
pi_add_target(self_calibration_estimator SHARED src REQUIRED OpenCV Ceres)
pi_report_target()
