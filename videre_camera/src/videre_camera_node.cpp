/*
 * Copyright (C) 2012
 * Laboratório de Automação e Robótica, Universidade de Brasília (LARA/UnB)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the names of Laboratório de Automação e Robótica or Universidade
 *     de Brasília nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific prior
 *     written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file	videre_camera_node.cpp
 * @author	George Andrew Brindeiro
 * @date	17/07/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <videre_camera/videre_camera.h>
#include <videre_camera/videre_camera_node.h>

// Standard C libraries
#include <signal.h>
#include <stdio.h>

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS libraries
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

VidereCamera* vc;

int main(int argc, char **argv)
{
    // Setup signal handlers
    setup_sig_handler();

    // Initialize CV images
    cv::Mat left_image(240, 320, CV_8UC3);
    cv::Mat right_image(240, 320, CV_8UC3);
    cv::Mat disp_image(240, 320, CV_16SC3);

    // Create VidereCamera object
    vc = new VidereCamera(true);

    // Setup ROS structures
    ros::init(argc, argv, "videre_camera_node", ros::init_options::AnonymousName);
    ros::NodeHandle vc_nh;

    // image_transport deals with image topics
    image_transport::ImageTransport it(vc_nh);
    image_transport::Publisher pub_left = it.advertise("videre_camera/left/image_raw", 1);
    image_transport::Publisher pub_right = it.advertise("videre_camera/right/image_raw", 1);

    // cv_bridge deals with conversion between cv::Mat and sensor_msgs::Image
    cv_bridge::CvImage msg_left;
    cv_bridge::CvImage msg_right;

    // regular publishers for camera info topics
    ros::Publisher pub_info_left = vc_nh.advertise<sensor_msgs::CameraInfo>("videre_camera/left/camera_info", 1);
    ros::Publisher pub_info_right = vc_nh.advertise<sensor_msgs::CameraInfo>("videre_camera/right/camera_info", 1);

    // camera info messages
    sensor_msgs::CameraInfo info_left;
    sensor_msgs::CameraInfo info_right;

    // We can set encoding prior to loop because it doesn't change
    msg_left.encoding = sensor_msgs::image_encodings::BGR8;
    msg_right.encoding = sensor_msgs::image_encodings::BGR8;

    // Warning that we are testing
    ROS_INFO("Hard-coded calibration in videre_camera_node");

    // We can set camera info params prior to loop because they don't change
    info_left.height = vc->cp()->height();
    info_left.width = vc->cp()->width();
    info_left.distortion_model = vc->cp()->distortion_model();
    //vc->cp()->left_D(info_left.D);
    //vc->cp()->left_K(info_left.K);
    //vc->cp()->left_R(info_left.R);
    //vc->cp()->left_P(info_left.P);

    /* OST Parameters */
    /*info_left.D.push_back(-0.369133);
    info_left.D.push_back(0.147212);
    info_left.D.push_back(-0.002032);
    info_left.D.push_back(0.002022);
    info_left.D.push_back(0.000000);
    info_left.K[0] = 292.585285;
    info_left.K[1] = 0.000000;
    info_left.K[2] = 159.848404;
    info_left.K[3] = 0.000000;
    info_left.K[4] = 292.076485;
    info_left.K[5] = 106.437630;
    info_left.K[6] = 0.000000;
    info_left.K[7] = 0.000000;
    info_left.K[8] = 1.000000;
    info_left.R[0] = 0.997261;
    info_left.R[1] = -0.002049;
    info_left.R[2] = -0.073932;
    info_left.R[3] = 0.002638;
    info_left.R[4] = 0.999965;
    info_left.R[5] = 0.007881;
    info_left.R[6] = 0.073914;
    info_left.R[7] = -0.008055;
    info_left.R[8] = 0.997232;
    info_left.P[0] = 304.389818;
    info_left.P[1] = 0.000000;
    info_left.P[2] = 184.028139;
    info_left.P[3] = 0.000000;
    info_left.P[4] = 0.000000;
    info_left.P[5] = 304.389818;
    info_left.P[6] = 112.227717;
    info_left.P[7] = 0.000000;
    info_left.P[8] = 0.000000;
    info_left.P[9] = 0.000000;
    info_left.P[10] = 1.000000;
    info_left.P[11] = 0.000000;*/

    /* SVS Parameters */
    info_left.D.push_back(-0.283421);
    info_left.D.push_back(-0.137791);
    info_left.D.push_back(0.000000);
    info_left.D.push_back(0.000000);
    info_left.D.push_back(0.000000);
    info_left.K[0] = 298.324130;
    info_left.K[1] = 0.000000;
    info_left.K[2] = 165.194788;
    info_left.K[3] = 0.000000;
    info_left.K[4] = 298.066606;
    info_left.K[5] = 102.797277;
    info_left.K[6] = 0.000000;
    info_left.K[7] = 0.000000;
    info_left.K[8] = 1.000000;
    info_left.R[0] = 9.999927e-01;
    info_left.R[1] = -3.466162e-03;
    info_left.R[2] = -1.641216e-03;
    info_left.R[3] = 3.468780e-03;
    info_left.R[4] = 9.999927e-01;
    info_left.R[5] = 1.594634e-03;
    info_left.R[6] = 1.635677e-03;
    info_left.R[7] = -1.600315e-03;
    info_left.R[8] = 9.999974e-01;
    info_left.P[0] = 2.980000e+02;
    info_left.P[1] = 0.000000;
    info_left.P[2] = 1.663312e+02;
    info_left.P[3] = 0.000000;
    info_left.P[4] = 0.000000;
    info_left.P[5] = 2.980000e+02;
    info_left.P[6] = 1.112314e+02;
    info_left.P[7] = 0.000000;
    info_left.P[8] = 0.000000;
    info_left.P[9] = 0.000000;
    info_left.P[10] = 1.000000;
    info_left.P[11] = 0.000000;

    info_right.height = vc->cp()->height();
    info_right.width = vc->cp()->width();
    info_right.distortion_model = vc->cp()->distortion_model();
    //vc->cp()->right_D(info_right.D);
    //vc->cp()->right_K(info_right.K);
    //vc->cp()->right_R(info_right.R);
    //vc->cp()->right_P(info_right.P);

    /* OST Parameters */
    /*info_right.D.push_back(-0.361614);
    info_right.D.push_back(0.141255);
    info_right.D.push_back(0.000429);
    info_right.D.push_back(-0.003258);
    info_right.D.push_back(0.000000);
    info_right.K[0] = 295.503697;
    info_right.K[1] = 0.000000;
    info_right.K[2] = 163.445080;
    info_right.K[3] = 0.000000;
    info_right.K[4] = 296.055638;
    info_right.K[5] = 120.811073;
    info_right.K[6] = 0.000000;
    info_right.K[7] = 0.000000;
    info_right.K[8] = 1.000000;
    info_right.R[0] = 0.998604;
    info_right.R[1] = 0.004671;
    info_right.R[2] = -0.052616;
    info_right.R[3] = -0.005091;
    info_right.R[4] = 0.999956;
    info_right.R[5] = -0.007842;
    info_right.R[6] = 0.052577;
    info_right.R[7] = 0.008099;
    info_right.R[8] = 0.998584;
    info_right.P[0] = 304.389818;
    info_right.P[1] = 0.000000;
    info_right.P[2] = 184.028139;
    info_right.P[3] = -47.698098;
    info_right.P[4] = 0.000000;
    info_right.P[5] = 304.389818;
    info_right.P[6] = 112.227717;
    info_right.P[7] = 0.000000;
    info_right.P[8] = 0.000000;
    info_right.P[9] = 0.000000;
    info_right.P[10] = 1.000000;
    info_right.P[11] = 0.000000;*/

    /* SVS Parameters */
	info_right.D.push_back(-0.407392);
	info_right.D.push_back(0.267797);
	info_right.D.push_back(0.000000);
	info_right.D.push_back(0.000000);
	info_right.D.push_back(0.000000);
	info_right.K[0] = 300.299950;
	info_right.K[1] = 0.000000;
	info_right.K[2] = 180.776418;
	info_right.K[3] = 0.000000;
	info_right.K[4] = 296.883585;
	info_right.K[5] = 120.758520;
	info_right.K[6] = 0.000000;
	info_right.K[7] = 0.000000;
	info_right.K[8] = 1.000000;
	info_right.R[0] = 9.973115e-01;
	info_right.R[1] = 2.665144e-03;
	info_right.R[2] = 7.323095e-02;
	info_right.R[3] = -2.548077e-03;
	info_right.R[4] = 9.999954e-01;
	info_right.R[5] = -1.691977e-03;
	info_right.R[6] = -7.323512e-02;
	info_right.R[7] = 1.500830e-03;
	info_right.R[8] = 9.973136e-01;
	info_right.P[0] = 2.980000e+02;
	info_right.P[1] = 0.000000;
	info_right.P[2] = 1.663312e+02;
	info_right.P[3] = -4.632412e+04*0.001;
	info_right.P[4] = 0.000000;
	info_right.P[5] = 2.980000e+02;
	info_right.P[6] = 1.112314e+02;
	info_right.P[7] = 0.000000;
	info_right.P[8] = 0.000000;
	info_right.P[9] = 0.000000;
	info_right.P[10] = 1.000000;
	info_right.P[11] = 0.000000;

    // Set frame_ids
    msg_left.header.frame_id = "camera_left";
    msg_right.header.frame_id = "camera_left";
    info_left.header.frame_id = "camera_left";
    info_right.header.frame_id = "camera_left";

    // TO DO: USE CAMERAINFO_MANAGER

    ros::Rate loop_rate(15);

    while(ros::ok())
    {
        ros::Time ts = ros::Time::now();

        bool got_image = vc->GetData();

        left_image = vc->left();
        right_image = vc->right();

        if(!got_image)
        {
            ROS_INFO("Error in camera_getimagepair, exiting program");
            break;
        }

        // Set image timestamps
        msg_left.header.stamp = ts;
        msg_right.header.stamp = ts;

        // Set image data
        msg_left.image = left_image;
        msg_right.image = right_image;

        // Publish images
        pub_left.publish(msg_left.toImageMsg());
        pub_right.publish(msg_right.toImageMsg());

        // Set camera info timestamps
        info_left.header.stamp = ts;
        info_right.header.stamp = ts;

        // Publish camera info
        pub_info_left.publish(info_left);
        pub_info_right.publish(info_right);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void setup_sig_handler()
{
    signal(SIGSEGV, &sig_handler);
    signal(SIGINT, &sig_handler);
    signal(SIGTSTP, &sig_handler);
}

void sig_handler(int sig)
{
    switch(sig)
    {
        case SIGSEGV:
            signal(SIGSEGV, SIG_DFL);
            printf("Signal caught: SIGSEGV\n");
            break;
        case SIGINT:
            signal(SIGINT, SIG_IGN);
            printf("Signal caught: SIGINT\n");
            break;
        case SIGTSTP:
            signal(SIGTSTP, SIG_IGN);
            printf("Signal caught: SIGTSTP\n");
            break;
    }

    printf("Closing camera nicely...\n");
    vc->~VidereCamera();

    exit(0);
}
