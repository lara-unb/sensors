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
