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
 * @file     videre_camera.h
 * @author   George Andrew Brindeiro
 * @date     25/07/2012
 *
 * @brief Interface for the STH-DCSG-VAR-C stereo cameras by Videre Design
 *
 * This driver supports the STH-DCSG-VAR-C stereo cameras by Videre Design,
 * providing an interface to capture regular and stereo images. Unlike the
 * videre_stereo_camera stack, this driver depends on the SVS library.
 *
 * Contact: georgebrindeiro@lara.unb.br
 *
 * Revisions:
 * [25/07/2012] Created
 */

#ifndef VIDERE_CAMERA_H
#define VIDERE_CAMERA_H

#include <videre_camera/image_display.h>
#include <videre_camera/camera_parameters.h>
#include <videre_camera/stereo_data.h>

// Standard C libraries
#include <cstdio>

// SVS library
#include <svsclass.h>

// OpenCV library
#include <opencv2/core/core.hpp>

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

class VidereCamera
{
    public:
        VidereCamera(bool display = false, int width = 320, int height = 240, int timeout = 5000)
        {
            printf("[VidereCamera]\n");

            width_ = width;
            height_ = height;

            display_ = display;
            timeout_ = timeout;
            count_ = 0;

            Init();
        }

        ~VidereCamera();

        bool GetData();

        inline const cv::Mat& left() { return sd_->cv_left(); }
        inline const cv::Mat& right() { return sd_->cv_right(); }

        inline StereoData* sd() { return sd_; }
        inline CameraParameters* cp() { return cp_; }

    private:
        // Camera Capture variables
        int width_;
        int height_;

        int timeout_;
        int count_;

        svsVideoImages* svs_vi_;

        // Stereo Data variables
        StereoData* sd_;

        // Camera Parameters variables
        CameraParameters* cp_;

        // Camera Display variables
        bool display_;
        ImageDisplay* id_;

        // Camera Capture functions
        void Init();
        bool InitCapture();
        void CloseCapture();

        DISALLOW_COPY_AND_ASSIGN(VidereCamera);
};

#endif //VIDERE_CAMERA_H
