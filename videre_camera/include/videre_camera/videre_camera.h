/**
* @file		videre_camera.h
* @author	George Andrew Brindeiro
* @date		17/07/2012
*
* @brief ROS interface for the STH-DCSG-VAR-C stereo cameras by Videre Design
*
* This driver supports the STH-DCSG-VAR-C stereo cameras by Videre Design, 
* providing a ROS interface to capture regular and stereo images. Unlike the 
* videre_stereo_camera stack, this driver depends on the SVS library.
* 
* Contact: georgebrindeiro@lara.unb.br
* 
* Revisions:
* [17/07/2012] Created
*/




/*****************************************************************************
*** Projeto Pioneer
*** Conteudo: Modulo das cameras.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 01/05/2009: criacao
*****************************************************************************/
/*! \file camera.h 
* \brief Arquivo cabeçalho da biblioteca camera. */
#ifndef VIDERE_CAMERA_H
#define VIDERE_CAMERA_H

#ifdef __cplusplus
// extern "C" {
#endif 

// Definicoes de uso externo:
#define CAMERA_MAXSIZE_IMAGES_QUEUE	10

// Prototipos de uso externo:
int camera_init(int grabperiod_ms, int imagewidth, int imageheight, double imagegamma, int flagverbose);
int camera_close(void);
int camera_getimagepair(IplImage **ppleft, IplImage **ppright);
void camera_printcvimageinfo(IplImage *pImCV);

#ifdef __cplusplus
//}
#endif 

#endif //CAMERA_H

