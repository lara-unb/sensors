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
