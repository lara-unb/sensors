/**
 * @file    videre_camera.cpp
 * @author  George Andrew Brindeiro
 * @date    25/07/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <videre_camera/videre_camera.h>
#include <videre_camera/videre_log.h>

// Standard C libraries
#include <cstdio>

// SVS library
#include <svsclass.h>

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define LEFT_WINDOW "Left Camera"
#define RIGHT_WINDOW "Right Camera"

bool VidereCamera::gamma_table_initialized_ = false;
unsigned char VidereCamera::gamma_table_[256];

VidereCamera::~VidereCamera()
{
    CloseCapture();

    if(display_)
        CloseDisplay();
}

bool VidereCamera::GetImagePair(cv::Mat& left_image, cv::Mat& right_image)
{
    //VC_LOG(INFO,"Grabbing image pair");

    stereo_image_ = video_object_->GetImage(timeout_);

    if(stereo_image_ == NULL)
    {
        VC_LOG(ERROR,"GetImagePair() failed");
        return false;
    }
    else
    {
        unsigned long* svs_left_image = stereo_image_->Color();
        unsigned long* svs_right_image = stereo_image_->ColorRight();

        SVStoCV(svs_left_image, cv_left_image_);
        SVStoCV(svs_right_image, cv_right_image_);

        if(display_)
            DisplayImagePair();

        cv_left_image_.copyTo(left_image);
        cv_right_image_.copyTo(right_image);

        //VC_LOG(INFO,"Image Count = %d", count_++);

        return true;
    }
}

void VidereCamera::Init()
{
    VC_LOG(INFO,"Initializing Videre Camera");

    cv_left_image_ = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_8U, 3);
    cv_right_image_ = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_8U, 3);

    if(!gamma_table_initialized_)
        InitGammaTable();

    InitCapture();

    if(display_)
        InitDisplay();
}

void VidereCamera::InitGammaTable()
{
    VC_LOG(INFO,"Initializing Gamma Table");

    for(int ii = 0; ii < 256; ii++)
    {
        gamma_table_[ii] = static_cast<unsigned char>(255.0*(pow(ii/255.0, gamma_)));
    }
}

bool VidereCamera::InitCapture()
{
    VC_LOG(INFO,"Initializing Capture");
    VC_LOG(INFO,"Framegrabber is %s", svsVideoIdent);

    video_object_ = getVideoObject();

    VC_LOG(INFO,"Opening video capture");

    bool video_open = video_object_->Open();
    if(!video_open)
    {
        VC_LOG(ERROR,"video_object_->Open() failed");

        CloseCapture();
        return false;
    }

    VC_LOG(INFO,"Found %i cameras", video_object_->Enumerate());

    VC_LOG(INFO,"Reading camera parameters from %s", "cfg/calibration.ini");

    video_object_->ReadParams((char*) "cfg/calibration.ini");

    VC_LOG(INFO,"Setting camera parameters");

    video_object_->SetColor(40, 40);
    video_object_->SetSize(width_, height_);
    video_object_->SetExposure(83, 100);
    video_object_->SetBrightness(0, 30);
    video_object_->SetRate(30);

    VC_LOG(INFO,"Setting up image rectification");

    bool video_rectified = video_object_->SetRect(true);
    if(!video_rectified)
        VC_LOG(ERROR,"video_object_->SetRect() failed");

    VC_LOG(INFO,"Starting video capture");

    bool video_started = video_object_->Start();
    if(!video_started)
    {
        VC_LOG(ERROR,"video_object_->Start() failed");

        CloseCapture();
        return false;
    }

    return true;
}

void VidereCamera::InitDisplay()
{
    VC_LOG(INFO,"Initializing Display");

    cv::namedWindow(LEFT_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(RIGHT_WINDOW, CV_WINDOW_AUTOSIZE);

    // C function is still used for move window
    cvMoveWindow(LEFT_WINDOW, 0, 0);
    cvMoveWindow(RIGHT_WINDOW, 330, 0);
}

void VidereCamera::DisplayImagePair()
{
    cv::imshow(LEFT_WINDOW, cv_left_image_);
    cv::imshow(RIGHT_WINDOW, cv_right_image_);

    cv::waitKey(3);
}

void VidereCamera::CloseCapture()
{
    VC_LOG(INFO,"Closing Capture");
    video_object_->Close();
}

void VidereCamera::CloseDisplay()
{
    VC_LOG(INFO,"Closing Display");

    cv::destroyWindow(LEFT_WINDOW);
    cv::destroyWindow(RIGHT_WINDOW);
}

void VidereCamera::SVStoCV(unsigned long* svs_image, cv::Mat& cv_image)
{
    if(cv_image.rows != height_)
        cv_image.rows = height_;

    if(cv_image.cols != width_)
        cv_image.cols = width_;

    int nl = cv_image.rows;
    int nc = cv_image.cols*cv_image.channels();

    uchar* svs_pixel;
    uchar* cv_line;

    unsigned long svs_pixel_value;
    svs_pixel = (uchar *) &svs_pixel_value;

    for(long ii = 0; ii < nl; ii++)
    {
        cv_line = cv_image.ptr<uchar>(ii);

        for(long jj = 0; jj < nc; jj++)
        {
            svs_pixel_value = (svs_image + width_*ii)[jj];

            cv_line[jj * 3 + 0] = gamma_table_[svs_pixel[2]];
            cv_line[jj * 3 + 1] = gamma_table_[svs_pixel[1]];
            cv_line[jj * 3 + 2] = gamma_table_[svs_pixel[0]];
        }
    }
}

void VidereCamera::PrintSVSInfo()
{
    svsImageParams* svs_params = video_object_->GetIP();

    VC_LOG(INFO,"SVS Image Info:");
    VC_LOG(INFO,"   linelen = %i",  svs_params->linelen);
    VC_LOG(INFO,"   lines = %i",    svs_params->lines);
    VC_LOG(INFO,"   ix = %i",       svs_params->ix);
    VC_LOG(INFO,"   iy = %i",       svs_params->iy);
    VC_LOG(INFO,"   width = %i",    svs_params->width);
    VC_LOG(INFO,"   height = %i",   svs_params->height);
    VC_LOG(INFO,"   vergence = %f", svs_params->vergence);
    VC_LOG(INFO,"   gamma = %f",    svs_params->gamma);
}

void VidereCamera::PrintCVInfo(cv::Mat& cv_image)
{
    VC_LOG(INFO,"OpenCV Image Info:");
    VC_LOG(INFO,"   channels = %i", cv_image.channels());
    VC_LOG(INFO,"   depth = %i",    cv_image.depth());
    VC_LOG(INFO,"   width = %i",    cv_image.cols);
    VC_LOG(INFO,"   height = %i",   cv_image.rows);
}
