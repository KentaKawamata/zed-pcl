///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/************************************************************************************
 ** This sample demonstrates how to use PCL (Point Cloud Library) with the ZED SDK **
 ************************************************************************************/

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

// Namespace
using namespace sl;
using namespace std;

// Global instance (ZED, Mat, callback)
Camera zed;
Mat data_cloud;
std::thread zed_callback;
std::mutex mutex_input;
bool stop_signal;
bool has_data;

// Sample functions
void startZED();
void run();
void closeZED();
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
inline float convertColor(float colorIn);

void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing) {

    if(event.getKeySym() == "space" && event.keyDown()){
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->points.resize(zed.getResolution().area());

        float *data = data_cloud.getPtr<float>();
        int index = 0;

        for (auto &it : cloud->points) {
            float X = data[index];
            if (!isValidMeasure(X)) // Checking if it's a valid point
                it.x = it.y = it.z = it.rgb = 0;
            else {
                it.x = X;
                it.y = data[index + 1];
                it.z = data[index + 2];
                it.rgb = convertColor(data[index + 3]); // Convert a 32bits float into a pcl .rgb format
            }
                index += 4;
        }

        pcl::io::savePLYFileASCII("test.ply", *cloud);
        std::cout << "---------- SAVE DATA !!!!! ----------" << std::endl;
    }
        //next_iteration = true;
}

// Main process
int main(int argc, char **argv) {

    if (argc > 2) {
        cout << "Only the path of a SVO can be passed in arg" << endl;
        return -1;
    }

    // Set configuration parameters
    InitParameters init_params;
    if (argc == 2)
        init_params.svo_input_filename = argv[1];
    else {
        init_params.camera_resolution = RESOLUTION_VGA;
        init_params.camera_fps = 30;
    }
    init_params.coordinate_units = UNIT_METER;
    init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        cout << toString(err) << endl;
        zed.close();
        return 1;
    }

    // Allocate PCL point cloud at the resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(zed.getResolution().area());

    // Create the PCL point cloud visualizer
    shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(p_pcl_point_cloud);
    viewer->registerKeyboardCallback(&keyboardEvent, (void*)NULL);

    // Start ZED callback
    startZED();

    // Loop until viewer catches the stop signal
    while (!viewer->wasStopped()) {

        // Try to lock the data if possible (not in use). Otherwise, do nothing.
        if (mutex_input.try_lock()) {
            float *p_data_cloud = data_cloud.getPtr<float>();
            int index = 0;

            // Check and adjust points for PCL format
            for (auto &it : p_pcl_point_cloud->points) {
                float X = p_data_cloud[index];
                if (!isValidMeasure(X)) // Checking if it's a valid point
                    it.x = it.y = it.z = it.rgb = 0;
                else {
                    it.x = X;
                    it.y = p_data_cloud[index + 1];
                    it.z = p_data_cloud[index + 2];
                    it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
                }
                index += 4;
            }

            // Unlock data and update Point cloud
            mutex_input.unlock();
            viewer->updatePointCloud(p_pcl_point_cloud);
            viewer->spinOnce(10);

        } else {
            sleep_ms(1);
        }
    }

    //pcl::io::savePLYFileASCII("test.ply", *p_pcl_point_cloud);

    // Close the viewer
    viewer->close();

    // Close the zed
    closeZED();

    return 0;
}

/**
 *  This functions start the ZED's thread that grab images and data.
 **/
void startZED() {
    // Start the thread for grabbing ZED data
    stop_signal = false;
    has_data = false;
    zed_callback = std::thread(run);

    //Wait for data to be grabbed
    while (!has_data)
        sleep_ms(1);
}

/**
 *  This function loops to get the point cloud from the ZED. It can be considered as a callback.
 **/
void run() {

    // Enable positional tracking with default parameters
    TrackingParameters tracking_parameters;
    ERROR_CODE err = zed.enableTracking(tracking_parameters);
    if (err != SUCCESS) {
        exit(-1);
    }

    // Track the camera position during 1000 frames
    Pose zed_pose;

    // Check if the camera is a ZED M and therefore if an IMU is available
    bool zed_mini = (zed.getCameraInformation().camera_model == MODEL_ZED_M);
    IMUData imu_data;



    while (!stop_signal) {
        if (zed.grab(SENSING_MODE_STANDARD) == SUCCESS) {

            zed.getPosition(zed_pose, REFERENCE_FRAME_WORLD); // Get the pose of the left eye of the camera with reference to the world frame

            // Display the translation and timestamp
            printf("\nTranslation: Tx: %.3f, Ty: %.3f, Tz: %.3f, Timestamp: %llu\n", zed_pose.getTranslation().tx,
                    zed_pose.getTranslation().ty, zed_pose.getTranslation().tz, zed_pose.timestamp);

            // Display the orientation quaternion
            printf("Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n", zed_pose.getOrientation().ox,
                    zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);



            
            if (zed_mini) { // Display IMU data

                 // Get IMU data
                zed.getIMUData(imu_data, TIME_REFERENCE_IMAGE);

                // Filtered orientation quaternion
                printf("IMU Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n", imu_data.getOrientation().ox,
                        imu_data.getOrientation().oy, imu_data.getOrientation().oz, zed_pose.getOrientation().ow);
                // Raw acceleration
                printf("IMU Acceleration: x: %.3f, y: %.3f, z: %.3f\n", imu_data.linear_acceleration.x,
                        imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
            }
         //}

        //if (zed.grab(SENSING_MODE_STANDARD) == SUCCESS) {
            mutex_input.lock(); // To prevent from data corruption
            zed.retrieveMeasure(data_cloud, MEASURE_XYZRGBA);
            mutex_input.unlock();
            has_data = true;
        } else {
            sleep_ms(1);
        }
    }
}

/**
 *  This function frees and close the ZED, its callback(thread) and the viewer
 **/
void closeZED() {
    // Stop the thread
    stop_signal = true;
    zed_callback.join();
    zed.close();
}

/**
 *  This function creates a PCL visualizer
 **/
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

/**
 *  This function convert a RGBA color packed into a packed RGBA PCL compatible format
 **/
inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}
