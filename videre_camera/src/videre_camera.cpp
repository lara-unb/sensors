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
#include <cmath>

// SVS library
#include <svsclass.h>

// OpenCV library
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#define LEFTWINDOW "Left Camera"
#define RIGHTWINDOW "Right Camera"

bool VidereCamera::gamma_table_initialized_ = false;
unsigned char VidereCamera::gamma_table_[256];

VidereCamera::~VidereCamera()
{
    CloseCapture();

    if(display_)
        CloseDisplay();
}

bool VidereCamera::GetImagePair(IplImage** left_image, IplImage** right_image)
{
    //printf("[VC] Grabbing image pair\n");

    stereo_image_ = video_object_->GetImage(timeout_);

    if(stereo_image_ == NULL)
    {
        printf("[VC] ERROR: GetImagePair() failed\n");
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

        *left_image = cv_left_image_;
        *right_image = cv_right_image_;

        //printf("[VC] Image Count = %d\n", count_++);

        return true;
    }
}

void VidereCamera::Init()
{
    printf("[VC] Initializing Videre Camera\n");

    if(!gamma_table_initialized_)
        InitGammaTable();

    InitCapture();

    if(display_)
        InitDisplay();
}

void VidereCamera::InitGammaTable()
{
    printf("[VC] Initializing Gamma Table\n");

    for(int ii = 0; ii < 256; ii++)
    {
        gamma_table_[ii] = static_cast<unsigned char>(255.0*(pow(ii/255.0, gamma_)));
    }
}

bool VidereCamera::InitCapture()
{
    printf("[VC] Initializing Capture\n");
    printf("[VC] Framegrabber is %s\n", svsVideoIdent);

    video_object_ = getVideoObject();

    printf("[VC] Opening video capture\n");

    bool video_open = video_object_->Open();
    if(!video_open)
    {
        printf("[VC] ERROR: video_object_->Open() failed\n");

        CloseCapture();
        return false;
    }

    printf("[VC] Found %i cameras\n", video_object_->Enumerate());

    printf("[VC] Reading camera parameters from %s\n", "cfg/calibration.ini");

    video_object_->ReadParams((char*) "cfg/calibration.ini");

    printf("[VC] Setting camera parameters\n");

    video_object_->SetColor(40, 40);
    video_object_->SetSize(width_, height_);
    video_object_->SetExposure(83, 100);
    video_object_->SetBrightness(0, 30);
    video_object_->SetRate(30);

    printf("[VC] Setting up image rectification\n");

    bool video_rectified = video_object_->SetRect(true);
    if(!video_rectified)
        printf("[VC] ERROR: video_object_->SetRect() failed\n");

    printf("[VC] Starting video capture\n");

    bool video_started = video_object_->Start();
    if(!video_started)
    {
        printf("[VC] ERROR: video_object_->Start() failed\n");

        CloseCapture();
        return false;
    }

    return true;
}

void VidereCamera::InitDisplay()
{
    printf("[VC] Initializing Display\n");

    cv_left_image_ = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_8U, 3);
    cv_right_image_ = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_8U, 3);

    cvNamedWindow(LEFTWINDOW, CV_WINDOW_AUTOSIZE);
    cvNamedWindow(RIGHTWINDOW, CV_WINDOW_AUTOSIZE);

    cvMoveWindow(LEFTWINDOW, 0, 0);
    cvMoveWindow(RIGHTWINDOW, 330, 0);
}

void VidereCamera::DisplayImagePair()
{
    cvShowImage(LEFTWINDOW, cv_left_image_);
    cvShowImage(RIGHTWINDOW, cv_right_image_);

    cvWaitKey(3);
}

void VidereCamera::CloseCapture()
{
    printf("[VC] Closing Capture\n");
    video_object_->Close();
}

void VidereCamera::CloseDisplay()
{
    printf("[VC] Closing Display\n");

    cvReleaseImage(&cv_left_image_);
    cvReleaseImage(&cv_right_image_);

    cvDestroyAllWindows();
}

void VidereCamera::SVStoCV(unsigned long* svs_image, IplImage* cv_image)
{
    if(cv_image->height != height_)
        cv_image->height = height_;

    if(cv_image->width != width_)
        cv_image->width = width_;

    uchar* svs_pixel;
    uchar* cv_line;

    unsigned long svs_pixel_value;
    svs_pixel = (uchar *) &svs_pixel_value;

    for(long ii = 0; ii < cv_image->height; ii++)
    {
        cv_line = (uchar *) (cv_image->imageData + cv_image->widthStep*ii);

        for(long jj = 0; jj < cv_image->width; jj++)
        {
            svs_pixel_value = (svs_image + width_*ii)[jj];

            cv_line[jj * 3 + 0] = gamma_table_[svs_pixel[2]];
            cv_line[jj * 3 + 1] = gamma_table_[svs_pixel[1]];
            cv_line[jj * 3 + 2] = gamma_table_[svs_pixel[0]];
        }
    }
}

void VidereCamera::PrintSVSInfo(svsImageParams *svs_params)
{
    printf("\n SVS Image Info:");
    printf("\n    linelen = %i", svs_params->linelen);
    printf("\n    lines = %i", svs_params->lines);
    printf("\n    ix = %i", svs_params->ix);
    printf("\n    iy = %i", svs_params->iy);
    printf("\n    width = %i", svs_params->width);
    printf("\n    height = %i", svs_params->height);
    printf("\n    vergence = %f", svs_params->vergence);
    printf("\n    gamma = %f", svs_params->gamma);
}

void VidereCamera::PrintCVInfo(IplImage *cv_image)
{
    printf("\n OpenCV Image Info:");
    printf("\n    nSize = %i", cv_image->nSize);
    printf("\n    nChannels = %i", cv_image->nChannels);
    printf("\n    depth = %i", cv_image->depth);
    printf("\n    dataOrder = %i", cv_image->dataOrder);
    printf("\n    origin = %i", cv_image->origin);
    printf("\n    width = %i", cv_image->width);
    printf("\n    height = %i", cv_image->height);
    printf("\n    imageSize = %i", cv_image->imageSize);
    printf("\n    widthStep = %i", cv_image->widthStep);
}
