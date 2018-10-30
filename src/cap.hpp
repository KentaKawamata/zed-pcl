// ZED includes
#include <sl_zed/Camera.hpp>

// PCL includes
// Undef on Win32 min/max for PCL
#ifdef _WIN32
#undef max
#undef min
#endif
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Sample includes
#include <thread>
#include <mutex>


class CaptureZED {

private:

    sl::Camera zed;
    sl::Mat data_cloud;
    std::thread zed_callback;
    std::mutex mutex_input;
    bool stop_signal;
    bool has_data;

    // Enable positional tracking with default parameters
    sl::TrackingParameters tracking_parameters;
    sl::Pose zed_pose;

    bool zed_mini = (zed.getCameraInformation().camera_model == sl::MODEL_ZED_M);
    sl::IMUData imu_data;

public:

    CaptureZED();
    ~CaptureZED();
    void run();

    void startZED();
    void closeZED();
    void runZED();
    void setparam();
    void setIMU();
    void saveRotation();
    std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    inline float convertColor(float colorIn);

};