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
 * @file     stereo_data.h
 * @author   George Andrew Brindeiro
 * @date     08/08/2012
 *
 * @brief Data structure containing stereo data and enabling conversion to ROS
 *
 * This class provides a container for all stereo data (images and disparity) as
 * well as functions to convert from CV to ROS format
 *
 * Contact: georgebrindeiro@lara.unb.br
 *
 * Revisions:
 * [08/08/2012] Created
 */

#ifndef STEREO_DATA_H
#define STEREO_DATA_H

// SVS library
#include <svsclass.h>

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class StereoData
{
    public:
        StereoData(svsVideoImages* svs_vi, int width = 640, int height = 480, double gamma = 0.850)
        {
            svs_vi_ = svs_vi;
            width_ = width;
            height_ = height;
            gamma_ = gamma;

            Init();
        }

        ~StereoData();

        bool GetData(svsStereoImage* svs_si);

        inline const cv::Mat& cv_left(){ return cv_left_; }
        inline const cv::Mat& cv_right(){ return cv_right_; }

    private:
        svsVideoImages* svs_vi_;
        svsStereoImage* svs_si_;
        svsStereoProcess* svs_sp_;

        unsigned long* svs_left_;
        unsigned long* svs_right_;

        cv::Mat cv_left_;
        cv::Mat cv_right_;

        int height_;
        int width_;

        double gamma_;
        unsigned char gamma_table_[256];

        void Init();
        void InitGammaTable();
        void Close();

        // Utility functions
        void SVStoCV(unsigned long* svs_image, cv::Mat& cv_image);
        void PrintSVSInfo();
        void PrintCVInfo(cv::Mat& cv_image);
};

#endif //STEREO_DATA_H
