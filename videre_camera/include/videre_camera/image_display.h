/**
 * @file     image_display.h
 * @author   George Andrew Brindeiro
 * @date     08/08/2012
 *
 * @brief Simple class enabling simple display of images
 *
 * This class separates all image display functions from videre_camera
 *
 * Contact: georgebrindeiro@lara.unb.br
 *
 * Revisions:
 * [08/08/2012] Created
 */

#ifndef IMAGE_DISPLAY_H
#define IMAGE_DISPLAY_H

// OpenCV library
#include <opencv2/core/core.hpp>

class ImageDisplay
{
    public:
        ImageDisplay()
        {
            Init();
        }

        ~ImageDisplay();

        void Init();
        void Display(const cv::Mat& left_image, const cv::Mat& right_image, const cv::Mat& disp_image);
        void Close();

    private:
};

#endif //IMAGE_DISPLAY_H
