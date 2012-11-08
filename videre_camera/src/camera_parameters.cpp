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
 * @file    camera_parameters.cpp
 * @author  George Andrew Brindeiro
 * @date    08/08/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <videre_camera/camera_parameters.h>
#include <videre_camera/videre_log.h>

// Standard C/C++ libraries
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

// SVS library
#include <svsclass.h>

void CameraParameters::Init()
{
    VC_LOG(INFO, "Initializing Camera Parameters");

    VC_LOG(INFO, "Setting camera parameters");

    svs_vi_->SetColor(40, 40);
    svs_vi_->SetSize(width_, height_);
    svs_vi_->SetExposure(83, 100);
    svs_vi_->SetBrightness(0, 30);
    svs_vi_->SetRate(30);

    VC_LOG(INFO, "Reading camera calibration from %s", params_file_.c_str());

    // Distortion Model
	// Default is "plumb_bob"
	distortion_model_ = "plumb_bob";

    // Get string corresponding to calibration file
    if(get_calibration_string())
    {
    	std::string calibration_format = params_file_.substr(params_file_.find_last_of(".") + 1);

    	extractParams(calibration_format);
    }
    else
    {
    	VC_LOG(ERROR, "Could not get calibration string");
    }
}

bool CameraParameters::get_calibration_string()
{
	std::ifstream t;
	char* buffer;
	int length;

	t.open(params_file_.c_str()); // open input file

	if(t.fail())
	{
		VC_LOG(ERROR, "Could not open file %s", params_file_.c_str());
		return false;
	}
	else
	{
		t.seekg(0, std::ios::end);    // go to the end
		length = t.tellg();           // report location (this is the length)
		t.seekg(0, std::ios::beg);    // go back to the beginning
		buffer = new char[length];    // allocate memory for a buffer of appropriate dimension
		t.read(buffer, length);       // read the whole file into the buffer
		t.close();                    // close file handle
	}

	params_string_.assign(buffer);

	VC_LOG(INFO, "Read parameters string");
	std::cout << params_string_;

	return true;
}

template <class T>
void extract(std::string& data, std::string section, std::string param, T& t)
{
    size_t found = data.find(section);

    if (found != std::string::npos)
    {
        found = data.find(param,found);

        if (found != std::string::npos)
        {
            std::istringstream iss(data.substr(found+param.length()));
            iss >> t;
        }
    }
    else
    	VC_LOG(ERROR, "Extract1 could not find %s", section.c_str());
}

void extract(std::string& data, std::string section,
             std::string param, double *m, int n)
{
    size_t found = data.find(section);

    if (found != std::string::npos)
    {
        found = data.find(param,found);

        if (found != std::string::npos)
        {
            std::istringstream iss(data.substr(found + param.length()));
            double v;

            for (int i = 0; i < n; ++i)
            {
                iss >> v;
                m[i] = v;
            }
        }
    }
    else
    	VC_LOG(ERROR, "Extract2 could not find %s", section.c_str());
}

void CameraParameters::parseCalibrationSVS(std::string params, stereo_side_t stereo_side)
{
	std::string side;

    switch (stereo_side)
    {
        case SIDE_LEFT:
        {
        	side = "left";

            // K - original camera matrix
            extract(params, "[" + side + " camera]", "f ", left_K_[0][0]);
            extract(params, "[" + side + " camera]", "fy", left_K_[1][1]);
            extract(params, "[" + side + " camera]", "Cx", left_K_[0][2]);
            extract(params, "[" + side + " camera]", "Cy", left_K_[1][2]);

			left_K_[0][1] = left_K_[1][0] = left_K_[2][0] = left_K_[2][1] = 0.0;
            left_K_[2][2] = 1.0;

            // D - distortion params
            extract(params, "[" + side + " camera]", "kappa1", left_D_[0]);
            extract(params, "[" + side + " camera]", "kappa2", left_D_[1]);
            extract(params, "[" + side + " camera]", "tau1", left_D_[2]);
            extract(params, "[" + side + " camera]", "tau2", left_D_[3]);
            extract(params, "[" + side + " camera]", "kappa3", left_D_[4]);

            // R - rectification matrix
            extract(params, "[" + side + " camera]", "rect", &left_R_[0][0], 9);

            // P - projection matrix
            extract(params, "[" + side + " camera]", "proj", &left_P_[0][0], 12);
            left_P_[0][3] *= .001;  // convert from mm to m

            break;
        }

        case SIDE_RIGHT:
        {
        	side = "right";

            // K - original camera matrix
            extract(params, "[" + side + " camera]", "f ", right_K_[0][0]);
            extract(params, "[" + side + " camera]", "fy", right_K_[1][1]);
            extract(params, "[" + side + " camera]", "Cx", right_K_[0][2]);
            extract(params, "[" + side + " camera]", "Cy", right_K_[1][2]);

			right_K_[0][1] = right_K_[1][0] = right_K_[2][0] = right_K_[2][1] = 0.0;
            right_K_[2][2] = 1.0;

            // D - distortion params
            extract(params, "[" + side + " camera]", "kappa1", right_D_[0]);
            extract(params, "[" + side + " camera]", "kappa2", right_D_[1]);
            extract(params, "[" + side + " camera]", "tau1", right_D_[2]);
            extract(params, "[" + side + " camera]", "tau2", right_D_[3]);
            extract(params, "[" + side + " camera]", "kappa3", right_D_[4]);

            // R - rectification matrix
            extract(params, "[" + side + " camera]", "rect", &right_R_[0][0], 9);

            // P - projection matrix
            extract(params, "[" + side + " camera]", "proj", &right_P_[0][0], 12);
            right_P_[0][3] *= .001;  // convert from mm to m

            break;
        }
    }
}

void CameraParameters::parseCalibrationOST(std::string params, stereo_side_t stereo_side)
{
    std::string side;

    switch (stereo_side)
    {
        case SIDE_LEFT:
        {
        	side = "left";

            // K - original camera matrix
            extract(params, "[narrow_stereo/" + side + "]", "camera matrix", &left_K_[0][0], 9);

            // D - distortion params
            extract(params, "[narrow_stereo/" + side + "]", "distortion", &left_D_[0], 5);

            // R - rectification matrix
            extract(params, "[narrow_stereo/" + side + "]", "rectification", &left_R_[0][0], 9);

            // P - projection matrix
            extract(params, "[narrow_stereo/" + side + "]", "projection", &left_P_[0][0], 12);

            break;
        }

        case SIDE_RIGHT:
        {
        	side = "right";

            // K - original camera matrix
            extract(params, "[narrow_stereo/" + side + "]", "camera matrix", &right_K_[0][0], 9);

            // D - distortion params
            extract(params, "[narrow_stereo/" + side + "]", "distortion", &right_D_[0], 5);

            // R - rectification matrix
            extract(params, "[narrow_stereo/" + side + "]", "rectification", &right_R_[0][0], 9);

            // P - projection matrix
            extract(params, "[narrow_stereo/" + side + "]", "projection", &right_P_[0][0], 12);

            break;
        }
    }
}

//
// gets params from a string
// "SVS"-type parameter strings use mm for the projection matrices, convert to m
// "OST"-type parameter strings use m for projection matrices
//
void CameraParameters::extractParams(std::string calibration_format)
{
    //printf("\n\n[extractParams] Parameters:\n\n");

    // Initialize Translation parameters
    /*for (int i = 0; i < 3; ++i)
    {
        T[i] = 0.0;
    }*/

    // Initialize Rotation parameters
    /*for (int i = 0; i < 3; ++i)
    {
        Om[i] = 0.0;
    }*/

    if(calibration_format == "svs") // SVS-type parameters
    {
        //printf("[dcam] SVS-type parameters\n");

        // Left camera calibration parameters
        parseCalibrationSVS(params_string_, SIDE_LEFT);

        // Right camera calibration parameters
        parseCalibrationSVS(params_string_, SIDE_RIGHT);

        // external params of undistorted cameras
        //extract(params, "[external]", "Tx", T[0]);
        //extract(params, "[external]", "Ty", T[1]);
        //extract(params, "[external]", "Tz", T[2]);
        //extract(params, "[external]", "Rx", Om[0]);
        //extract(params, "[external]", "Ry", Om[1]);
        //extract(params, "[external]", "Rz", Om[2]);

        //T[0] *= .001;
        //T[1] *= .001;
        //T[2] *= .001;
    }
    else if(calibration_format == "opencv") // OST-type parameters
    {
        //printf("[dcam] OST-type parameters\n");

        // Left camera calibration parameters
        parseCalibrationOST(params_string_, SIDE_LEFT);

        // Right camera calibration parameters
        parseCalibrationOST(params_string_, SIDE_RIGHT);

        // external params of undistorted cameras
        //extract(params, "[externals]", "translation", T, 3);
        //extract(params, "[externals]", "rotation", Om, 3);
    }

    // disparity resolution
    //extract(params, "[stereo]", "dpp", dpp);
    //extract(params, "[stereo]", "corrxsize", corrSize);
    //extract(params, "[stereo]", "convx", filterSize);
    //extract(params, "[stereo]", "ndisp", numDisp);

    // check for left camera matrix
    //if (left_info.K[0] == 0.0) { hasRectification = false; }
    //else { hasRectification = true; }

    // check for right camera matrix
    //if (right_info.K[0] == 0.0) { hasRectification = false; }
}
