/**
 * @file    image_display.cpp
 * @author  George Andrew Brindeiro
 * @date    08/08/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <videre_camera/image_display.h>
#include <videre_camera/videre_log.h>
//#include <colormaps/colormap.hpp>

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define LEFT_WINDOW "Left Camera"
#define RIGHT_WINDOW "Right Camera"
#define DISP_WINDOW "Disparity"

ImageDisplay::~ImageDisplay()
{
    Close();
}

void ImageDisplay::Init()
{
    VC_LOG(INFO, "Initializing Display");

    cv::namedWindow(LEFT_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(RIGHT_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(DISP_WINDOW, CV_WINDOW_AUTOSIZE);

    // C function is still used for move window
    cvMoveWindow(LEFT_WINDOW, 0, 0);
    cvMoveWindow(RIGHT_WINDOW, 330, 0);
    cvMoveWindow(DISP_WINDOW, 660, 0);
}

void ImageDisplay::Display(const cv::Mat& cv_left_image, const cv::Mat& cv_right_image, const cv::Mat& cv_disp_image)
{
    //cv::colormap::Jet cm;

    cv::imshow(LEFT_WINDOW, cv_left_image);
    cv::imshow(RIGHT_WINDOW, cv_right_image);
    cv::imshow(DISP_WINDOW, cv_disp_image);
    //cv::imshow(DISP_WINDOW, cm(cv_disp_image));

    cv::waitKey(3);
}

void ImageDisplay::Close()
{
    VC_LOG(INFO, "Closing Display");

    cv::destroyWindow(LEFT_WINDOW);
    cv::destroyWindow(RIGHT_WINDOW);
    cv::destroyWindow(DISP_WINDOW);
}
