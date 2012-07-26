/**
 * @file    videre_camera.cpp
 * @author  George Andrew Brindeiro
 * @date    25/07/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#define LEFTWINDOW "Left Camera"
#define RIGHTWINDOW "Right Camera"

#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/io.h>

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>

#include "svsclass.h"

#include <videre_camera/videre_camera.h>

bool VidereCamera::GammaTableIsInitialized_ = false;
unsigned char VidereCamera::GammaTable_[256];

VidereCamera::~VidereCamera()
{
    CloseCapture();

    if(display_)
        CloseDisplay();
}

bool VidereCamera::GetImagePair(IplImage** ppImageLeft, IplImage** ppImageRight)
{
    stereoImage_ = videoObject_->GetImage(timeout_);

    if(stereoImage_ == NULL)
    {
        printf("stereoImage == NULL");
        return false;
    }
    else
    {
        unsigned long* pSVSLeftImage = stereoImage_->Color();
        unsigned long* pSVSRightImage = stereoImage_->Color();

        SVStoCV(pSVSLeftImage, *ppImageLeft);
        SVStoCV(pSVSRightImage, *ppImageRight);

        count_++;

        return true;
    }
}

void VidereCamera::DisplayImagePair()
{
    cvShowImage(LEFTWINDOW, pImageLeft_);
    cvShowImage(RIGHTWINDOW, pImageRight_);
}

void VidereCamera::Init()
{
    if(!GammaTableIsInitialized_)
        InitGammaTable();

    InitCapture();

    if(display_)
        InitDisplay();
}

bool VidereCamera::InitCapture()
{
    printf("\n\n*** Inicialização do módulo camera");
    printf("\n*** Framegrabber: %s", svsVideoIdent);

    // Get the svsVideoImages object from the currently loaded camera interface
    videoObject_ = getVideoObject();
    videoObject_->ReadParams((char*) "cfg/calibration.ini");

    // Open the stereo device
    bool ret;
    ret = videoObject_->Open();
    if(!ret)
    {
        printf("\n*** Erro: o sistema de cameras nao pode ser iniciado.");
        CloseCapture();
        return false;
    }

    printf("\n*** Sistema de cameras iniciado: %i cameras encontradas", videoObject_->Enumerate());

    // Set camera parameters *after* opening the device
    videoObject_->SetColor(40, 40);
    videoObject_->SetSize(width_, height_);
    videoObject_->SetExposure(83, 100);
    videoObject_->SetBrightness(0, 30);
    videoObject_->SetRate(30);

    ret = videoObject_->Start();
    if(!ret)
    {
        printf("\n*** Erro: nao pode ser iniciada captura do sistema de cameras.");
        CloseCapture();
        return false;
    }

    // Set up acquisition to rectify the image
    ret = videoObject_->SetRect(true);
    if(!ret)
    {
        printf("\n*** Aviso: nao pode ser feita retificacao das imagens.");
    }

    return true;
}

void VidereCamera::CloseCapture()
{
    videoObject_->Close();
}

void VidereCamera::InitDisplay()
{
    // Create CV images
    pImageLeft_ = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_8U, 3);
    pImageRight_ = cvCreateImage(cvSize(width_, height_), IPL_DEPTH_8U, 3);

    // Create CV windows
    cvNamedWindow(LEFTWINDOW, CV_WINDOW_AUTOSIZE);
    cvNamedWindow(RIGHTWINDOW, CV_WINDOW_AUTOSIZE);
    cvMoveWindow(LEFTWINDOW, 0, 0);
    cvMoveWindow(RIGHTWINDOW, 330, 0);
}

void VidereCamera::CloseDisplay()
{
    // Release CV images
    cvReleaseImage(&pImageLeft_);
    cvReleaseImage(&pImageRight_);

    // Destroy CV windows
    cvDestroyAllWindows();
}

void VidereCamera::InitGammaTable()
{
    for(int n = 0; n < 256; n++)
    {
        GammaTable_[n] = (unsigned char) (255.0*(pow((double) n / 255.0, gamma_)));
    }
}

void VidereCamera::SVStoCV(unsigned long* pImSVS, IplImage*pImCV)
{
    pImCV->height = height_;
    pImCV->width = width_;

    unsigned long ulong;
    uchar *puchardest;
    uchar *pucharorig;

    pucharorig = (uchar *) (&ulong);
    for(long i = 0; i < pImCV->height; ++i)
    {
        for(long j = 0; j < pImCV->width; ++j)
        {
            ulong = (pImSVS + width_ * i)[j];
            puchardest = (uchar *) (pImCV->imageData + pImCV->widthStep * i);
            (puchardest)[j * 3 + 0] = GammaTable_[(pucharorig)[2]];
            (puchardest)[j * 3 + 1] = GammaTable_[(pucharorig)[1]];
            (puchardest)[j * 3 + 2] = GammaTable_[(pucharorig)[0]];
        }
    }
}

void VidereCamera::PrintSVSInfo(svsImageParams *pIp)
{
    printf("\n SVS Image Info:");
    printf("\n    linelen = %i", pIp->linelen);
    printf("\n    lines = %i", pIp->lines);
    printf("\n    ix = %i", pIp->ix);
    printf("\n    iy = %i", pIp->iy);
    printf("\n    width = %i", pIp->width);
    printf("\n    height = %i", pIp->height);
    printf("\n    vergence = %f", pIp->vergence);
    printf("\n    gamma = %f", pIp->gamma);
}

void VidereCamera::PrintCVInfo(IplImage *pImCV)
{
    printf("\n OpenCV Image Info:");
    printf("\n    nSize = %i", pImCV->nSize);
    printf("\n    nChannels = %i", pImCV->nChannels);
    printf("\n    depth = %i", pImCV->depth);
    printf("\n    dataOrder = %i", pImCV->dataOrder);
    printf("\n    origin = %i", pImCV->origin);
    printf("\n    width = %i", pImCV->width);
    printf("\n    height = %i", pImCV->height);
    printf("\n    imageSize = %i", pImCV->imageSize);
    printf("\n    widthStep = %i", pImCV->widthStep);
}
