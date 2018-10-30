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

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <fstream>

#include "cap.hpp"

using namespace sl;
using namespace std;

// Global instance (ZED, Mat, callback)
sl::Camera zed;
sl::Mat data_cloud;
std::thread zed_callback;
std::mutex mutex_input;
int signal=0;
bool stop_signal;
bool has_data;

void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing) {

    if(event.getKeySym() == "space" && event.keyDown()){
        signal=1;
    }
}

CaptureZED::CaptureZED(){
    startZED();
    sl::InitParameters init_params;
    setparam();
}

CaptureZED::~CaptureZED(){
   closeZED();
}

void CaptureZED::setparam() {

    init_params.camera_resolution = RESOLUTION_VGA;
    init_params.camera_fps = 30;
    init_params.coordinate_units = UNIT_METER;
    init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
}

// Main process
void CaptureZED::runZED() {

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::SUCCESS) {
        std::cout << toString(err) << std::endl;
        zed.close();
        std::exit(-1);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(zed.getResolution().area());

    // Create the PCL point cloud visualizer
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(p_pcl_point_cloud);

    while (!viewer->wasStopped()) {

        // Try to lock the data if possible (not in use). Otherwise, do nothing.
        if (mutex_input.try_lock()) {
            float *p_data_cloud = data_cloud.getPtr<float>();
            int index = 0;

            // Check and adjust points for PCL format
            for (auto &it : p_pcl_point_cloud->points) {
                float X = p_data_cloud[index];

                // Checking if it's a valid point
                if (!isValidMeasure(X)) {
                    it.x = it.y = it.z = it.rgb = 0;
                } else {
                    it.x = X;
                    it.y = p_data_cloud[index + 1];
                    it.z = p_data_cloud[index + 2];
                    // Convert a 32bits float into a pcl .rgb format
                    it.rgb = convertColor(p_data_cloud[index + 3]);
                }
                index += 4;
            }

            // Unlock data and update Point cloud
            mutex_input.unlock();
            viewer->updatePointCloud(p_pcl_point_cloud);
            viewer->spinOnce(10);

            if(signal==1){
                saveRotation();
                pcl::io::savePLYFileASCII("test.ply", *cloud);
                std::cout << "---------- SAVE DATA !!!!! ----------" << std::endl;
                signal=0;
            }

        } else {
            sleep_ms(1);
        }
    }

    viewer->close();
}

void CaptureZED::saveRotation() {

    std::vector<float> R(9);

    R[0] = zed_pose.getRotation().r00;
    R[1] = zed_pose.getRotation().r01;
    R[2] = zed_pose.getRotation().r02;
    R[3] = zed_pose.getRotation().r10;
    R[4] = zed_pose.getRotation().r11;
    R[5] = zed_pose.getRotation().r12;
    R[6] = zed_pose.getRotation().r20;
    R[7] = zed_pose.getRotation().r21;
    R[8] = zed_pose.getRotation().r22;

    std::ofstream myfile;
    myfile.open("example.csv");
    myfile << R[0] << "," << R[1] << "," R[2] << "," \
           << R[3] << "," << R[4] << "," R[5] << "," \
           << R[6] << "," << R[7] << "," R[8] << "\n" << std::endl;
    myfile.close();

}

/**
 *  This functions start the ZED's thread that grab images and data.
 **/
void CaptureZED::startZED() {

    // Start the thread for grabbing ZED data
    stop_signal = false;
    has_data = false;
    zed_callback = std::thread(run);

    //Wait for data to be grabbed
    while (!has_data) {
        sleep_ms(1);
    }
}

/**
 *  This function loops to get the point cloud from the ZED. It can be considered as a callback.
 **/
void CaptureZED::run() {

    sl::ERROR_CODE err = zed.enableTracking(tracking_parameters);
    if (err != sl::SUCCESS) {
        exit(-1);
    }

    // Check if the camera is a ZED M and therefore if an IMU is available
    bool zed_mini = (zed.getCameraInformation().camera_model == sl::MODEL_ZED_M);
    sl::IMUData imu_data;

    while (!stop_signal) {

        if (zed.grab(sl::SENSING_MODE_STANDARD) == SUCCESS) {

            // Get the pose of the left eye of the camera with reference to the world frame
            zed.getPosition(zed_pose, REFERENCE_FRAME_WORLD);

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
void CaptureZED::closeZED() {
    // Stop the thread
    stop_signal = true;
    zed_callback.join();
    zed.close();
}

/**
 *  This function creates a PCL visualizer
 **/
shared_ptr<pcl::visualization::PCLVisualizer> CaptureZED::createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {

    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->registerKeyboardCallback(&keyboardEvent, (void*)NULL);

    return viewer;
}

/**
 *  This function convert a RGBA color packed into a packed RGBA PCL compatible format
 **/
inline float CaptureZED::convertColor(float colorIn) {

    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);

    return *reinterpret_cast<float *> (&color_uint);
}

int main(int argc, char *argv[]){

    CaptureZED ZED;
    ZED.runZED();

    return 0;
}