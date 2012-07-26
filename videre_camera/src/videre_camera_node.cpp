/**
 * @file	videre_camera_node.cpp
 * @author	George Andrew Brindeiro
 * @date	17/07/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <signal.h>

#include <stdio.h>

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>

#include <videre_camera/videre_camera.h>
#include <videre_camera/videre_camera_node.h>

VidereCamera* vc;

IplImage* pImageLeft;
IplImage* pImageRight;

int main(int argc, char **argv)
{
    // Set up signal handlers
    signal(SIGSEGV, &sigsegv_handler);
    signal(SIGINT, &sigint_handler);
    signal(SIGTSTP, &sigtstp_handler);

    // Create VidereCamera object
    vc = new VidereCamera(true);

    while(1)
    {
        bool got_image = vc->GetImagePair(&pImageLeft, &pImageRight);

        if(got_image)
        {
            vc->DisplayImagePair();
            cvWaitKey(100);
        }
        else
        {
            printf("Error in camera_getimagepair, exiting program\n");
            break;
        }
    }

    return 0;
}

void sigsegv_handler(int sig)
{
    signal(SIGSEGV, SIG_DFL);
    printf("System segfaulted, stopping camera nicely\n");

    vc->~VidereCamera();
}

void sigint_handler(int sig)
{
    signal(SIGINT, SIG_DFL);
    printf("System interrupted, stopping camera nicely\n");

    vc->~VidereCamera();
}

void sigtstp_handler(int sig){
    signal(SIGTSTP, SIG_DFL);
    printf("System stopped temporarily, stopping camera nicely\n");

    vc->~VidereCamera();
}
