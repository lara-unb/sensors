/**
 * @file     videre_camera.h
 * @author   George Andrew Brindeiro
 * @date     17/07/2012
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

#include "svsclass.h"

class VidereCamera
{
    public:
        VidereCamera(bool display = false, int width = 320, int height = 240, double gamma = 0.850, int timeout = 5000)
        {
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
        void DisplayImagePair();

    private:
        bool display_;
        int width_;
        int height_;
        double gamma_;
        int timeout_;
        int count_;

        svsVideoImages* videoObject_;
        svsStereoImage* stereoImage_;

        IplImage* pImageLeft_;
        IplImage* pImageRight_;

        static bool GammaTableIsInitialized_;
        static unsigned char GammaTable_[256];

        void Init();

        bool InitCapture();
        void CloseCapture();

        void InitDisplay();
        void CloseDisplay();

        void InitGammaTable();

        void SVStoCV(unsigned long *pImSVS, IplImage *pImCV);
        void PrintSVSInfo(svsImageParams *pIp);
        void PrintCVInfo(IplImage *pImCV);
};

#endif //VIDERE_CAMERA_H
