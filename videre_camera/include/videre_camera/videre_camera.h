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

#include <videre_camera/videre_log.h>

// Standard C libraries
#include <cstdio>

// SVS library
#include <svsclass.h>

// OpenCV library
#include <cv.h>

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

class VidereCamera
{
    public:
        VidereCamera(bool display = false, int width = 320, int height = 240, double gamma = 0.850, int timeout = 5000)
        {
            VC_LOG(INFO, "[VidereCamera]\n");

            display_ = display;
            width_ = width;
            height_ = height;
            gamma_ = gamma;
            timeout_ = timeout;
            count_ = 0;

            Init();
        }

        ~VidereCamera();

        bool GetImagePair(IplImage** ppImageLeft, IplImage** ppImageRight);

    private:
        bool display_;

        int width_;
        int height_;
        double gamma_;

        int timeout_;
        int count_;

        svsVideoImages* video_object_;
        svsStereoImage* stereo_image_;

        IplImage* cv_left_image_;
        IplImage* cv_right_image_;

        static bool gamma_table_initialized_;
        static unsigned char gamma_table_[256];

        void Init();

        void InitGammaTable();
        void InitDisplay();
        bool InitCapture();

        void DisplayImagePair();

        void CloseDisplay();
        void CloseCapture();

        void SVStoCV(unsigned long* svs_image, IplImage* cv_image);
        void PrintSVSInfo(svsImageParams* svs_params);
        void PrintCVInfo(IplImage* cv_image);

        DISALLOW_COPY_AND_ASSIGN(VidereCamera);
};

#endif //VIDERE_CAMERA_H
