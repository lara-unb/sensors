/**
 * @file    videre_camera.cpp
 * @author  George Andrew Brindeiro
 * @date    25/07/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <videre_camera/videre_camera.h>
#include <videre_camera/videre_log.h>
#include <videre_camera/image_display.h>
#include <videre_camera/stereo_data.h>

// Standard C libraries
#include <cstdio>

// SVS library
#include <svsclass.h>

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

VidereCamera::~VidereCamera()
{
    CloseCapture();

    if(display_)
        id_->Close();
}

void VidereCamera::Init()
{
    VC_LOG(INFO,"Initializing Videre Camera");

    InitCapture();

    sd_ = new StereoData(svs_vi_);

    if(display_)
    {
        id_ = new ImageDisplay();
    }
}

bool VidereCamera::InitCapture()
{
    VC_LOG(INFO,"Initializing Capture");
    VC_LOG(INFO,"Framegrabber is %s", svsVideoIdent);

    svs_vi_ = getVideoObject();

    VC_LOG(INFO,"Opening video capture");

    bool video_open = svs_vi_->Open();
    if(!video_open)
    {
        VC_LOG(ERROR,"svs_vi_->Open() failed");

        CloseCapture();
        return false;
    }

    VC_LOG(INFO,"Found %i cameras", svs_vi_->Enumerate());

    cp_ = new CameraParameters(svs_vi_);

    VC_LOG(INFO,"Setting up image rectification");

    bool video_rectified = svs_vi_->SetRect(true);
    if(!video_rectified)
        VC_LOG(ERROR,"video_object_->SetRect() failed");

    VC_LOG(INFO,"Starting video capture");

    bool video_started = svs_vi_->Start();
    if(!video_started)
    {
        VC_LOG(ERROR,"video_object_->Start() failed");

        CloseCapture();
        return false;
    }

    return true;
}

bool VidereCamera::GetData()
{
    svsStereoImage* svs_si = svs_vi_->GetImage(timeout_);

    bool got_image = sd_->GetData(svs_si);

    if(display_)
        id_->Display(sd_->cv_left(), sd_->cv_right(), sd_->cv_disp());

    //VC_LOG(INFO,"Image Count = %d", count_++);

    return got_image;
}

void VidereCamera::CloseCapture()
{
    VC_LOG(INFO,"Closing Capture");
    svs_vi_->Close();
}

