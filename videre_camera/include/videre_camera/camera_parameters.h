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
#include <string>

// OpenCV library
#include <svsclass.h>

class CameraParameters
{
    public:
        CameraParameters(svsVideoImages* svs_vi, const char* params_file = "cfg/calibration.ini", int width = 320, int height = 240) :
            svs_vi_(svs_vi), params_file_(params_file), height_(height), width_(width)
        {
            Init();
        }

        ~CameraParameters(){};

        inline int height() { return height_; }
        inline int width() { return width_; }

        inline const std::string distortion_model() { return distortion_model_; }

        inline void left_D(double D[5])
        {
            for(int ii = 0; ii < 5; ii++)
                D[ii] = left_D_[ii];
        }
        inline void left_K(double K[3][3])
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 3; jj++)
                    K[ii][jj] = left_K_[ii][jj];
        }
        inline void left_R(double R[3][3])
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 3; jj++)
                    R[ii][jj] = left_R_[ii][jj];
        }
        inline void left_P(double P[3][4])
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 4; jj++)
                    P[ii][jj] = left_P_[ii][jj];
        }
        inline void right_D(double D[5])
        {
            for(int ii = 0; ii < 5; ii++)
                D[ii] = right_D_[ii];
        }
        inline void right_K(double K[3][3])
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 3; jj++)
                    K[ii][jj] = right_K_[ii][jj];
        }
        inline void right_R(double R[3][3])
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 3; jj++)
                    R[ii][jj] = right_R_[ii][jj];
        }
        inline void right_P(double P[3][4])
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 4; jj++)
                    P[ii][jj] = right_P_[ii][jj];
        }

    private:
        svsVideoImages* svs_vi_;
        svsSP* svs_sp_;

        const std::string params_file_;

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

        void Init();
};

#endif //CAMERA_PARAMETERS_H
