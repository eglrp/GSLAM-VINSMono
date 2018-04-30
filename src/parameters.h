#pragma once

#include "ros/ros.h"
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility.h"

#define GPU 0

const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
const int WINDOW_SIZE = 10;

extern int ROW;
extern int COL ;
extern double FOCAL_LENGTH;
extern std::string CALIB_DIR;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int FREQ;
extern double F_THRESHOLD;
extern double T_THRESHOLD;
extern bool SHOW_TRACK;
extern bool STEREO_TRACK;
extern bool USE_F;
extern bool USE_E;
extern bool EQUALIZE;

// Estimator parameters

//#define INV_DEP
//#define DEPTH_PRIOR
//#define GT

extern int MAX_FEATURE_CNT;
extern int NUM_OF_ITER;
extern double CALIB_THRESHOLD_TIC;
extern double CALIB_THRESHOLD_RIC;
extern double INIT_DEPTH;
extern double GRADIENT_THRESHOLD;
extern double FEATURE_THRESHOLD;
extern double MIN_PARALLAX;
extern double MIN_PARALLAX_POINT;
extern double ERROR_THRESHOLD;
extern bool SHOW_HISTOGRAM;
extern bool MULTI_THREAD;
extern bool SHOW_GRAPH;
extern bool SHOW_HTML;

extern double IMU_RATE;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<bool> RIC_OK, TIC_OK;
extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern bool COMPENSATE_ROTATION;

void readParameters();

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
#ifdef HAS_ROS
void readParameters(ros::NodeHandle &n);
#endif
