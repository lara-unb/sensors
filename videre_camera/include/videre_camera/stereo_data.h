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
        StereoData(svsVideoImages* svs_vi, int width = 320, int height = 240, double gamma = 0.850)
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
        inline const cv::Mat& cv_disp(){ return cv_disp_; }

    private:
        svsVideoImages* svs_vi_;
        svsStereoImage* svs_si_;
        svsStereoProcess* svs_sp_;

        unsigned long* svs_left_;
        unsigned long* svs_right_;
        short* svs_disp_;

        cv::Mat cv_left_;
        cv::Mat cv_right_;
        cv::Mat cv_disp_;

        int height_;
        int width_;

        double gamma_;
        unsigned char gamma_table_[256];

        void Init();
        void InitGammaTable();
        void Close();

        // Utility functions
        void SVStoCV(unsigned long* svs_image, cv::Mat& cv_image);
        void SVStoCVDisp(short* svs_image, cv::Mat& cv_image);
        void PrintSVSInfo();
        void PrintCVInfo(cv::Mat& cv_image);
};

#endif //STEREO_DATA_H
