/**
 * @file	videre_camera_node.cpp
 * @author	George Andrew Brindeiro
 * @date	17/07/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <videre_camera/videre_camera.h>
#include <videre_camera/videre_camera_node.h>

// Standard C libraries
#include <signal.h>
#include <stdio.h>

// OpenCV library
#include <cv.h>
#include <highgui.h>

VidereCamera* vc;

int main(int argc, char **argv)
{
    // Set up signal handlers
    setup_sig_handler();

    // Initialize CV images
    IplImage* left_image = cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3);
    IplImage* right_image = cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3);

    // Create VidereCamera object
    vc = new VidereCamera(true);

    while(1)
    {
        bool got_image = vc->GetImagePair(&left_image, &right_image);

        if(!got_image)
        {
            printf("Error in camera_getimagepair, exiting program\n");
            break;
        }
    }

    return 0;
}

void setup_sig_handler()
{
    signal(SIGSEGV, &sig_handler);
    signal(SIGINT, &sig_handler);
    signal(SIGTSTP, &sig_handler);
}

void sig_handler(int sig)
{
    switch(sig)
    {
        case SIGSEGV:
            signal(SIGSEGV, SIG_DFL);
            printf("Signal caught: SIGSEGV\n");
            break;
        case SIGINT:
            signal(SIGINT, SIG_IGN);
            printf("Signal caught: SIGINT\n");
            break;
        case SIGTSTP:
            signal(SIGTSTP, SIG_IGN);
            printf("Signal caught: SIGTSTP\n");
            break;
    }

    printf("Closing camera nicely...\n");
    vc->~VidereCamera();

    exit(0);
}
