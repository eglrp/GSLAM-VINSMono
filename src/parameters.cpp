#include "parameters.h"

std::string CALIB_DIR;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
int FREQ;
double F_THRESHOLD;
double T_THRESHOLD;
bool SHOW_TRACK;
bool STEREO_TRACK;
bool USE_F;
bool USE_E;
bool EQUALIZE;

int MAX_FEATURE_CNT;
int NUM_OF_ITER;
double CALIB_THRESHOLD_TIC;
double CALIB_THRESHOLD_RIC;
double INIT_DEPTH;
double GRADIENT_THRESHOLD;
double FEATURE_THRESHOLD;
double MIN_PARALLAX;
double MIN_PARALLAX_POINT;
double ERROR_THRESHOLD;
bool SHOW_HISTOGRAM;
bool SHOW_GRAPH;
bool SHOW_HTML;
bool MULTI_THREAD;
double IMU_RATE;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<bool> RIC_OK, TIC_OK;
std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.70};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
bool COMPENSATE_ROTATION;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    CALIB_DIR = readParam<std::string>(n, "calib_dir");
    for (int i = 0; i < NUM_OF_CAM; i++)
        CAM_NAMES.push_back(readParam<std::string>(n, "cam_name" + std::to_string(i)));
    MAX_CNT = readParam<int>(n, "max_cnt");
    MIN_DIST = readParam<int>(n, "min_dist");
    FREQ = readParam<int>(n, "freq");
//    WINDOW_SIZE = readParam<int>(n, "window_size");
    F_THRESHOLD = readParam<double>(n, "F_threshold");
    T_THRESHOLD = readParam<double>(n, "T_threshold");
    SHOW_TRACK = readParam<bool>(n, "show_track");
    STEREO_TRACK = readParam<bool>(n, "stereo_track");
    USE_F = readParam<bool>(n, "use_F");
    USE_E = readParam<bool>(n, "use_E");
    EQUALIZE = readParam<bool>(n, "equalize");

    MAX_FEATURE_CNT = readParam<int>(n, "max_feature_cnt");
    NUM_OF_ITER = readParam<int>(n, "num_of_iter");
    CALIB_THRESHOLD_RIC = readParam<double>(n, "calib_threshold_ric");
    CALIB_THRESHOLD_TIC = readParam<double>(n, "calib_threshold_tic");
    INIT_DEPTH = readParam<double>(n, "init_depth");
    GRADIENT_THRESHOLD = readParam<double>(n, "gradient_threshold") / FOCAL_LENGTH;
    FEATURE_THRESHOLD = readParam<double>(n, "feature_threshold") / FOCAL_LENGTH;
    MIN_PARALLAX = readParam<double>(n, "min_parallax") / FOCAL_LENGTH;
    MIN_PARALLAX_POINT = readParam<double>(n, "min_parallax_point") / FOCAL_LENGTH;
    ERROR_THRESHOLD = readParam<double>(n, "error_threshold");
    SHOW_HISTOGRAM = readParam<bool>(n, "show_histogram");
    SHOW_GRAPH = readParam<bool>(n, "show_graph");
    SHOW_HTML = readParam<bool>(n, "show_html");
    MULTI_THREAD = readParam<bool>(n, "multi_thread");

    IMU_RATE = readParam<double>(n, "imu_rate");
    ACC_N = readParam<double>(n, "acc_n");
    ACC_W = readParam<double>(n, "acc_w");
    GYR_N = readParam<double>(n, "gyr_n");
    GYR_W = readParam<double>(n, "gyr_w");
    BIAS_ACC_THRESHOLD = readParam<double>(n, "bias_acc_threshold");
    BIAS_GYR_THRESHOLD = readParam<double>(n, "bias_gyr_threshold");
    SOLVER_TIME = readParam<double>(n, "solver_time");
    COMPENSATE_ROTATION = readParam<bool>(n, "compensate_rotation");

    Eigen::Matrix3d tmp;
    tmp << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    std::cout << Utility::R2ypr(tmp) << std::endl;

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RIC_OK.push_back(readParam<bool>(n, std::string("ric_ok") + std::to_string(i)));
        if (RIC_OK.back())
        {
            RIC.push_back(Utility::ypr2R(Eigen::Vector3d(
                readParam<double>(n, std::string("ric_y") + std::to_string(i)),
                readParam<double>(n, std::string("ric_p") + std::to_string(i)),
                readParam<double>(n, std::string("ric_r") + std::to_string(i)))));
            std::cout << RIC[i] << std::endl;
        }
        TIC_OK.push_back(readParam<bool>(n, std::string("tic_ok") + std::to_string(i)));
        if (TIC_OK.back())
        {
            TIC.push_back(Eigen::Vector3d(
                readParam<double>(n, std::string("tic_x") + std::to_string(i)),
                readParam<double>(n, std::string("tic_y") + std::to_string(i)),
                readParam<double>(n, std::string("tic_z") + std::to_string(i))));
        }
    }
}

void readParameters()
{
    CALIB_DIR = "/data/zhaoyong/Program/Thirdparty/VI-MEAN/feature_tracker/config/";
    for (int i = 0; i < NUM_OF_CAM; i++)
        CAM_NAMES.push_back("sample");
    MAX_CNT = 100;
    MIN_DIST = 30;
    FREQ = 3;
//    WINDOW_SIZE = readParam<int>(n, "window_size");
    F_THRESHOLD = 1.;
    T_THRESHOLD = 0.5;
    SHOW_TRACK = false;
    STEREO_TRACK = false;
    USE_F = true;
    USE_E = false;
    EQUALIZE = true;

    MAX_FEATURE_CNT = 2000000;
    NUM_OF_ITER = 10;
    CALIB_THRESHOLD_RIC = 0.0;
    CALIB_THRESHOLD_TIC = 1.0;
    INIT_DEPTH = 15.;
    GRADIENT_THRESHOLD = 1.0 / FOCAL_LENGTH;
    FEATURE_THRESHOLD = 5.0 / FOCAL_LENGTH;
    MIN_PARALLAX = 10.0 / FOCAL_LENGTH;
    MIN_PARALLAX_POINT = 2.0 / FOCAL_LENGTH;
    ERROR_THRESHOLD =10000000000.0;
    SHOW_HISTOGRAM = false;
    SHOW_GRAPH = false;
    SHOW_HTML = false;
    MULTI_THREAD = false;

    IMU_RATE = 200;
    ACC_N = 0.01;
    ACC_W = 1.0e-5;
    GYR_N = 0.05;
    GYR_W = 5.0e-5;
    BIAS_ACC_THRESHOLD = 0.5;
    BIAS_GYR_THRESHOLD = 0.1;
    SOLVER_TIME = 0.040;
    COMPENSATE_ROTATION = true;

    Eigen::Matrix3d tmp;
    tmp << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    std::cout << Utility::R2ypr(tmp) << std::endl;

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RIC_OK.push_back(true);
        if (RIC_OK.back())
        {
            RIC.push_back(Utility::ypr2R(Eigen::Vector3d(
                -90.0,
                0.0,
                90.0)));
            std::cout << RIC[i] << std::endl;
        }
        TIC_OK.push_back(true);
        if (TIC_OK.back())
        {
            TIC.push_back(Eigen::Vector3d(
                0,
                0,
                0));
        }
    }
}
