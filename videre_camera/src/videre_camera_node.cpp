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

VidereCamera* vc;

int main(int argc, char **argv)
{
    // Setup signal handlers
    setup_sig_handler();

    // Initialize CV images
    cv::Mat left_image(240, 320, CV_8UC3);
    cv::Mat right_image(240, 320, CV_8UC3);

    // Create VidereCamera object
    vc = new VidereCamera(true);

    // Setup ROS structures
    ros::init(argc, argv, "videre_camera_node", ros::init_options::AnonymousName);
    ros::NodeHandle vc_nh;

    ros::Rate loop_rate(15);

    while(ros::ok())
    {
        bool got_image = vc->GetImagePair(left_image, right_image);

        if(!got_image)
        {
            ROS_INFO("Error in camera_getimagepair, exiting program");
            break;
        }

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
