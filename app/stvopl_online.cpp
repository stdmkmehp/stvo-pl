/*****************************************************************************
**      Stereo VO and SLAM by combining point and line segment features     **
******************************************************************************
**                                                                          **
**  Copyright(c) 2016-2018, Ruben Gomez-Ojeda, University of Malaga         **
**  Copyright(c) 2016-2018, David Zuñiga-Noël, University of Malaga         **
**  Copyright(c) 2016-2018, MAPIR group, University of Malaga               **
**                                                                          **
**  This program is free software: you can redistribute it and/or modify    **
**  it under the terms of the GNU General Public License (version 3) as     **
**  published by the Free Software Foundation.                              **
**                                                                          **
**  This program is distributed in the hope that it will be useful, but     **
**  WITHOUT ANY WARRANTY; without even the implied warranty of              **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            **
**  GNU General Public License for more details.                            **
**                                                                          **
**  You should have received a copy of the GNU General Public License       **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.   **
**                                                                          **
*****************************************************************************/

#ifdef HAS_MRPT
#include <sceneRepresentation.h>
#endif

#include <stereoFrame.h>
#include <stereoFrameHandler.h>
#include <boost/filesystem.hpp>

#include "dataset.h"
#include "timer.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include "rosImg2cvMat.h"

using namespace StVO;

void showHelp();

cv::Mat leftIm(cvSize(1280, 720), CV_8UC3);
cv::Mat rightIm(cvSize(1280, 720), CV_8UC3);
cv::Mat leftCompressedIm(cvSize(1280, 720), CV_8UC3);
//cv::Mat leftDepth16uc1(cvSize(1280, 720), CV_16UC1);

int main(int argc, char **argv)
{

    // read inputs
    ros::init(argc, argv, "stvopl_online");

    ros::NodeHandle nh;

    //ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    ros::Subscriber sub_leftIm = nh.subscribe("/zed/left/image_rect_color", 1000, leftImCallback);
    ros::Subscriber sub_rightIm = nh.subscribe("/zed/right/image_rect_color", 1000, rightImCallback);
    ros::Subscriber sub_leftCompressedIm = nh.subscribe("/zed/right/image_raw_color/compressed", 1000, leftCompressedImCallback);

    // read inputs
    string config_file, camera_params, scene_config;
    ros::param::get("config_file", config_file);
    ros::param::get("camera_params", camera_params);
    ros::param::get("scene_config", scene_config);
    cout << config_file << endl;
    cout << camera_params << endl;
    cout << scene_config << endl;

    if (!config_file.empty()) Config::loadFromFile(config_file);

    PinholeStereoCamera*  cam_pin = new PinholeStereoCamera(camera_params);
    // Dataset dataset(dataset_dir, *cam_pin, frame_offset, frame_number, frame_step);

    // create scene
    Matrix4d Tcw, T_inc = Matrix4d::Identity();
    Vector6d cov_eig;
    Matrix6d cov;
    Tcw = Matrix4d::Identity();
    Tcw << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1;

    #ifdef HAS_MRPT
    sceneRepresentation scene(scene_config);
    scene.initializeScene(Tcw, false);
    #endif

    Timer timer;

    // initialize and run PL-StVO
    int frame_counter = 0;
    double t1;
    StereoFrameHandler* StVO = new StereoFrameHandler(cam_pin);
    cv::Mat& img_l = leftIm;
    cv::Mat& img_r = rightIm;
    // while (dataset.nextFrame(img_l, img_r))
    ros::Rate loop_rate(10);    // 10Hz
    while (ros::ok())
    {
        if( frame_counter == 0 ) // initialize
            StVO->initialize(img_l,img_r,0);
        else // run
        {
            // PL-StVO
            timer.start();
            StVO->insertStereoPair( img_l, img_r, frame_counter );
            StVO->optimizePose();
            t1 = timer.stop();

            T_inc   = StVO->curr_frame->DT;
            cov     = StVO->curr_frame->DT_cov;
            cov_eig = StVO->curr_frame->DT_cov_eig;

            // update scene
            #ifdef HAS_MRPT
            scene.setText(frame_counter,t1,StVO->n_inliers_pt,StVO->matched_pt.size(),StVO->n_inliers_ls,StVO->matched_ls.size());
            scene.setCov( cov );
            scene.setPose( T_inc );
            scene.setImage(StVO->curr_frame->plotStereoFrame());
            scene.updateScene(StVO->matched_pt, StVO->matched_ls);
            #endif

            // console output
            cout.setf(ios::fixed,ios::floatfield); cout.precision(8);
            cout << "Frame: " << frame_counter << "\tRes.: " << StVO->curr_frame->err_norm;
            cout.setf(ios::fixed,ios::floatfield); cout.precision(3);
            cout << " \t Proc. time: " << t1 << " ms\t ";
            if( Config::adaptativeFAST() )  cout << "\t FAST: "   << StVO->orb_fast_th;
            if( Config::hasPoints())        cout << "\t Points: " << StVO->matched_pt.size() << " (" << StVO->n_inliers_pt << ") " ;
            if( Config::hasLines() )        cout << "\t Lines:  " << StVO->matched_ls.size() << " (" << StVO->n_inliers_ls << ") " ;
            cout << endl;

            // update StVO
            StVO->updateFrame();
        }

        frame_counter++;

        ros::spinOnce();
        loop_rate.sleep();
    }

    // wait until the scene is closed
    #ifdef HAS_MRPT
    while( scene.isOpen() );
    #endif

    return 0;
}

void showHelp() {
    cout << endl << "Usage: ./imagesStVO <dataset_name> [options]" << endl
         << "Options:" << endl
         << "\t-c Config file" << endl
         << "\t-o Offset (number of frames to skip in the dataset directory" << endl
         << "\t-n Number of frames to process the sequence" << endl
         << "\t-s Parameter to skip s-1 frames (default 1)" << endl
         << endl;
}