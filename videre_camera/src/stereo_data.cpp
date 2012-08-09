/**
 * @file    stereo_data.cpp
 * @author  George Andrew Brindeiro
 * @date    08/08/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <videre_camera/stereo_data.h>
#include <videre_camera/videre_log.h>

// SVS library
#include <svsclass.h>

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

StereoData::~StereoData()
{
    Close();
}

void StereoData::Init()
{
    InitGammaTable();

    cv_left_.create(height_, width_, CV_8UC3);
    cv_right_.create(height_, width_, CV_8UC3);
    cv_disp_.create(height_, width_, CV_16SC1);

    svs_sp_ = new svsStereoProcess();
}

void StereoData::InitGammaTable()
{
    VC_LOG(INFO,"Initializing Gamma Table");

    for(int ii = 0; ii < 256; ii++)
    {
        gamma_table_[ii] = static_cast<unsigned char>(255.0*(pow(ii/255.0, gamma_)));
    }
}

void StereoData::Close()
{

}

bool StereoData::GetData(svsStereoImage* svs_si)
{
    svs_si_ = svs_si;

    if(svs_si_ == NULL)
    {
        VC_LOG(ERROR,"GetData() failed");
        return false;
    }
    else
    {
        //VC_LOG(INFO,"Grabbing image pair");
        svs_sp_->CalcStereo(svs_si_);

        svs_left_ = svs_si_->Color();
        svs_right_ = svs_si_->ColorRight();
        svs_disp_ = svs_si_->Disparity();

        SVStoCV(svs_left_, cv_left_);
        SVStoCV(svs_right_, cv_right_);
        SVStoCVDisp(svs_disp_, cv_disp_);

        return true;
    }
}

void StereoData::SVStoCV(unsigned long* svs_image, cv::Mat& cv_image)
{
    if(cv_image.rows != height_)
        cv_image.rows = height_;

    if(cv_image.cols != width_)
        cv_image.cols = width_;

    int nl = cv_image.rows;
    int nc = cv_image.cols;

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

            // BGR (CV) vs RGBX (SVS)
            cv_line[jj * 3 + 0] = gamma_table_[svs_pixel[2]];
            cv_line[jj * 3 + 1] = gamma_table_[svs_pixel[1]];
            cv_line[jj * 3 + 2] = gamma_table_[svs_pixel[0]];
        }
    }
}

void StereoData::SVStoCVDisp(short* svs_image, cv::Mat& cv_image)
{
    if(cv_image.rows != height_)
        cv_image.rows = height_;

    if(cv_image.cols != width_)
        cv_image.cols = width_;

    int nl = cv_image.rows;
    int nc = cv_image.cols;

    short svs_disp;
    short* cv_line;

    for(long ii = 0; ii < nl; ii++)
    {
        cv_line = cv_image.ptr<short>(ii);

        for(long jj = 0; jj < nc; jj++)
        {
            svs_disp = (svs_image + width_*ii)[jj];

            cv_line[jj] = svs_disp; //static_cast<int>((static_cast<double>(jj)/nc)*32767*2 - 32767);
        }
    }
}

void StereoData::PrintSVSInfo()
{
    svsImageParams* svs_params = svs_vi_->GetIP();

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

void StereoData::PrintCVInfo(cv::Mat& cv_image)
{
    VC_LOG(INFO,"OpenCV Image Info:");
    VC_LOG(INFO,"   channels = %i", cv_image.channels());
    VC_LOG(INFO,"   depth = %i",    cv_image.depth());
    VC_LOG(INFO,"   width = %i",    cv_image.cols);
    VC_LOG(INFO,"   height = %i",   cv_image.rows);
}
