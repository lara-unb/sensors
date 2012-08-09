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
#include <cstring>
#include <string>

// SVS library
#include <svsclass.h>

CameraParameters::~CameraParameters()
{
    Close();
}

void CameraParameters::Init()
{
    VC_LOG(INFO, "Initializing Camera Parameters");
    VC_LOG(INFO, "Reading camera parameters from %s", params_file_);

    svs_vi_->ReadParams((char*) params_file_);

    VC_LOG(INFO, "Setting camera parameters");

    svs_vi_->SetColor(40, 40);
    svs_vi_->SetSize(width_, height_);
    svs_vi_->SetExposure(83, 100);
    svs_vi_->SetBrightness(0, 30);
    svs_vi_->SetRate(30);

    VC_LOG(INFO, "Loading camera parameters");

    svs_sp_ = svs_vi_->GetSP();

    // Distortion Model
    // Default is "plumb_bob"
    distortion_model_ = "plumb_bob";

    // Distortion Parameters
    // For "plumb_bob" the 5 parameters are: (k1, k2, t1, t2, k3)
    left_D_[0] = svs_sp_->left.kappa1;
    left_D_[1] = svs_sp_->left.kappa2;
    left_D_[2] = svs_sp_->left.tau1;
    left_D_[3] = svs_sp_->left.tau2;
    left_D_[4] = svs_sp_->left.kappa3;

    right_D_[0] = svs_sp_->right.kappa1;
    right_D_[1] = svs_sp_->right.kappa2;
    right_D_[2] = svs_sp_->right.tau1;
    right_D_[3] = svs_sp_->right.tau2;
    right_D_[4] = svs_sp_->right.kappa3;

    // Intrinsic Parameters (for the raw distorted images)
    left_K_[0][0] = svs_sp_->left.f;
    left_K_[0][1] = 0;
    left_K_[0][2] = svs_sp_->left.Cx;
    left_K_[1][0] = 0;
    left_K_[1][1] = svs_sp_->left.fy;
    left_K_[1][2] = svs_sp_->left.Cy;
    left_K_[2][0] = 0;
    left_K_[2][1] = 0;
    left_K_[2][2] = 1;

    right_K_[0][0] = svs_sp_->right.f;
    right_K_[0][1] = 0;
    right_K_[0][2] = svs_sp_->right.Cx;
    right_K_[1][0] = 0;
    right_K_[1][1] = svs_sp_->right.fy;
    right_K_[1][2] = svs_sp_->right.Cy;
    right_K_[2][0] = 0;
    right_K_[2][1] = 0;
    right_K_[2][2] = 1;

    // Rectification Parameters
    left_R_[0][0] = svs_sp_->left.rect[0][0];
    left_R_[0][1] = svs_sp_->left.rect[0][1];
    left_R_[0][2] = svs_sp_->left.rect[0][2];
    left_R_[1][0] = svs_sp_->left.rect[1][0];
    left_R_[1][1] = svs_sp_->left.rect[1][1];
    left_R_[1][2] = svs_sp_->left.rect[1][2];
    left_R_[2][0] = svs_sp_->left.rect[2][0];
    left_R_[2][1] = svs_sp_->left.rect[2][1];
    left_R_[2][2] = svs_sp_->left.rect[2][2];

    right_R_[0][0] = svs_sp_->right.rect[0][0];
    right_R_[0][1] = svs_sp_->right.rect[0][1];
    right_R_[0][2] = svs_sp_->right.rect[0][2];
    right_R_[1][0] = svs_sp_->right.rect[1][0];
    right_R_[1][1] = svs_sp_->right.rect[1][1];
    right_R_[1][2] = svs_sp_->right.rect[1][2];
    right_R_[2][0] = svs_sp_->right.rect[2][0];
    right_R_[2][1] = svs_sp_->right.rect[2][1];
    right_R_[2][2] = svs_sp_->right.rect[2][2];

    // Projection Parameters (Instrinsics for the rectified images)
    left_P_[0][0] = svs_sp_->left.proj[0][0];
    left_P_[0][1] = svs_sp_->left.proj[0][1];
    left_P_[0][2] = svs_sp_->left.proj[0][2];
    left_P_[0][3] = svs_sp_->left.proj[0][3];
    left_P_[1][0] = svs_sp_->left.proj[1][0];
    left_P_[1][1] = svs_sp_->left.proj[1][1];
    left_P_[1][2] = svs_sp_->left.proj[1][2];
    left_P_[1][3] = svs_sp_->left.proj[1][3];
    left_P_[2][0] = svs_sp_->left.proj[2][0];
    left_P_[2][1] = svs_sp_->left.proj[2][1];
    left_P_[2][2] = svs_sp_->left.proj[2][2];
    left_P_[2][3] = svs_sp_->left.proj[2][3];

    right_P_[0][0] = svs_sp_->right.proj[0][0];
    right_P_[0][1] = svs_sp_->right.proj[0][1];
    right_P_[0][2] = svs_sp_->right.proj[0][2];
    right_P_[0][3] = svs_sp_->right.proj[0][3];
    right_P_[1][0] = svs_sp_->right.proj[1][0];
    right_P_[1][1] = svs_sp_->right.proj[1][1];
    right_P_[1][2] = svs_sp_->right.proj[1][2];
    right_P_[1][3] = svs_sp_->right.proj[1][3];
    right_P_[2][0] = svs_sp_->right.proj[2][0];
    right_P_[2][1] = svs_sp_->right.proj[2][1];
    right_P_[2][2] = svs_sp_->right.proj[2][2];
    right_P_[2][3] = svs_sp_->right.proj[2][3];
}

void CameraParameters::Close()
{

}
