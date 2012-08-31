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
