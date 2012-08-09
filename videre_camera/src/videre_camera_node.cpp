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
    image_transport::Publisher pub_left = it.advertise("videre_camera/left/image_rect_color", 1);
    image_transport::Publisher pub_right = it.advertise("videre_camera/right/image_rect_color", 1);

    // cv_bridge deals with conversion between cv::Mat and sensor_msgs::Image
    cv_bridge::CvImage msg_left;
    cv_bridge::CvImage msg_right;

    // We can set encoding prior to loop because it doesn't change
    msg_left.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    msg_right.encoding = sensor_msgs::image_encodings::TYPE_8UC3;

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
