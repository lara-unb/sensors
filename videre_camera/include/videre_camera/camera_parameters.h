/**
 * @file     camera_parameters.h
 * @author   George Andrew Brindeiro
 * @date     08/08/2012
 *
 * @brief Data structure containing camera parameters and enabling conversion to ROS
 *
 * This class provides a container for all camera parameters, as well as functions
 * functions to convert to ROS format
 *
 * Contact: georgebrindeiro@lara.unb.br
 *
 * Revisions:
 * [08/08/2012] Created
 */

#ifndef CAMERA_PARAMETERS_H
#define CAMERA_PARAMETERS_H

// Standard C/C++ libraries
#include <cstring>
#include <string>

// OpenCV library
#include <svsclass.h>

class CameraParameters
{
    public:
        CameraParameters(svsVideoImages* svs_vi, const char* params_file = "cfg/calibration.ini", int height = 240, int width = 320)
        {
            svs_vi_ = svs_vi;
            strcpy(params_file_, params_file);
            height_ = height;
            width_ = width;

            Init();
        }

        ~CameraParameters();

        void Init();
        void Close();

    private:
        svsVideoImages* svs_vi_;
        svsSP* svs_sp_;

        char params_file_[256];

        int height_;
        int width_;

        std::string distortion_model_;

        double left_D_[5];
        double left_K_[3][3];
        double left_R_[3][3];
        double left_P_[3][4];

        double right_D_[5];
        double right_K_[3][3];
        double right_R_[3][3];
        double right_P_[3][4];
};

#endif //CAMERA_PARAMETERS_H
