/**
* @file		videre_camera_node.h
* @author	George Andrew Brindeiro
* @date		17/07/2012
*
* @brief ROS node for the STH-DCSG-VAR-C stereo cameras by Videre Design
*
* This node uses the interface provided by the VidereCamera class to publish
* images, disparity images and point clouds from the STH-DCSG-VAR-C cameras.
* 
* Contact: georgebrindeiro@lara.unb.br
* 
* Revisions:
* [17/07/2012] Created
*/

#ifndef VIDERE_CAMERA_NODE_H
#define VIDERE_CAMERA_NODE_H

void setup_sig_handler();
void sig_handler(int sig);

#endif //VIDERE_CAMERA_NODE_H
