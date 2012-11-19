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
    int width = 480;
    int height = 640;

    cv::Mat left_image(height, width, CV_8UC3);
    cv::Mat right_image(height, width, CV_8UC3);
    cv::Mat disp_image(height, width, CV_16SC3);

    // Create VidereCamera object
    vc = new VidereCamera();

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

    // We can set camera info params prior to loop because they don't change
    info_left.height = vc->cp()->height();
    info_left.width = vc->cp()->width();
    info_left.distortion_model = vc->cp()->distortion_model();
    vc->cp()->left_D(info_left.D);
    vc->cp()->left_K(info_left.K);
    vc->cp()->left_R(info_left.R);
    vc->cp()->left_P(info_left.P);

    info_right.height = vc->cp()->height();
    info_right.width = vc->cp()->width();
    info_right.distortion_model = vc->cp()->distortion_model();
    vc->cp()->right_D(info_right.D);
    vc->cp()->right_K(info_right.K);
    vc->cp()->right_R(info_right.R);
    vc->cp()->right_P(info_right.P);

    // Set frame_ids
    msg_left.header.frame_id = "/videre_camera/left";
    msg_right.header.frame_id = "/videre_camera/left";
    info_left.header.frame_id = "/videre_camera/left";
    info_right.header.frame_id = "/videre_camera/left";

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
