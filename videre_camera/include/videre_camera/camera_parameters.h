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
#include <vector>

// OpenCV library
#include <svsclass.h>

// Boost Array
#include <boost/array.hpp>

typedef enum
{
    SIDE_LEFT = 0,
    SIDE_RIGHT
} stereo_side_t;

class CameraParameters
{
    public:
        CameraParameters(svsVideoImages* svs_vi, const char* params_file = "cfg/calibration.opencv", int width = 640, int height = 480) :
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

        // Accessor function versions for ROS
        inline void left_D(std::vector<double>& D)
        {
            D.insert(D.begin(), left_D_, left_D_+5);
        }
        inline void left_K(boost::array<double,9>& K)
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 3; jj++)
                    K[3*ii+jj] = left_K_[ii][jj];
        }
        inline void left_R(boost::array<double,9>& R)
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 3; jj++)
                    R[3*ii+jj] = left_R_[ii][jj];
        }
        inline void left_P(boost::array<double,12>& P)
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 4; jj++)
                    P[4*ii+jj] = left_P_[ii][jj];
        }
        inline void right_D(std::vector<double>& D)
        {
            D.insert(D.begin(), right_D_, right_D_+5);
        }
        inline void right_K(boost::array<double,9>& K)
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 3; jj++)
                    K[3*ii+jj] = right_K_[ii][jj];
        }
        inline void right_R(boost::array<double,9>& R)
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 3; jj++)
                    R[3*ii+jj] = right_R_[ii][jj];
        }
        inline void right_P(boost::array<double,12>& P)
        {
            for(int ii = 0; ii < 3; ii++)
                for(int jj = 0; jj < 4; jj++)
                    P[4*ii+jj] = right_P_[ii][jj];
        }

    private:
        svsVideoImages* svs_vi_;
        svsSP* svs_sp_;

        const std::string params_file_;
        std::string params_string_;

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

        bool get_calibration_string();
        void parseCalibrationSVS(std::string params, stereo_side_t stereo_side);
        void parseCalibrationOST(std::string params, stereo_side_t stereo_side);
        void extractParams(std::string calibration_format);
};

#endif //CAMERA_PARAMETERS_H
