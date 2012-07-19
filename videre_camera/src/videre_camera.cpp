/**
 * @file	videre_camera.cpp
 * @author	George Andrew Brindeiro
 * @date	17/07/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <image_transport/image_transport.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    //ros::init(argc, argv, "videre_camera");

    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    //image_transport::ImageTransport it(nh);
    //image_transport::Publisher pub = it.advertise("image_raw", 10);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}


/*
****************************************************************************
*** Projeto Pioneer
*** Conteudo: Modulo das cameras.
*** Autor: G. A. Borges.
*** Atualizacoes: 
	- 01/05/2009: criacao
****************************************************************************
! \file camera.cpp
* \brief Arquivo com as funcoes da biblioteca camera.

// Cabecalhos des biblioteca padrao C:
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>  for libc5
#include <sys/io.h>  for glibc
#include <pthread.h>

// Cabecalhos especificos do modulo:
#include <cv.h>				
#include <cvaux.h>			
#include <cxcore.h>			

#include "main.h" 
#include "time.h" 
#include "videre_camera.h" 
#include "svsclass.h"

// Definicoes internas:

// Prototipos internos:
int camera_initcapturesystem(void);
void camera_closecapturesystem(void);
void *camera_grabthread(void *ptr);
void camera_svstocvimage(unsigned long *pImSVS, unsigned long Height, unsigned long Width, IplImage *pImCV);
void camera_printsvsimageinfo(svsImageParams *pIp);

// Variaveis do módulo:
svsVideoImages	*videoObject = NULL; 
svsStereoImage	*stereoImage;					
unsigned char   GammaTable[256];
double		Gamma = 0.850;

IplImage	**ppImageRight, **ppImageLeft;
int			currentimageindex = 0;
int			camera_width = 0, camera_height = 0;
int 		camera_flagverbose = false;

pthread_t       camera_grabthreadstructure;
pthread_mutex_t mutex_grabthreadsharedvariables = PTHREAD_MUTEX_INITIALIZER;
int flaggrabthreadactive = false;
int grabthreadperiod_ms;

****************************************************************************
******************************************************************************
** Funções de inicialização e encerramento
******************************************************************************
****************************************************************************
! \fn int camera_init(int grabperiod_ms, int imagewidth, int imageheight, int flagverbose)
* Funcao de inicializacao das cameras.
* \param 
* \return

int camera_init(int grabperiod_ms, int imagewidth, int imageheight, double imagegamma, int flagverbose)
{
	int n;

	camera_width  = imagewidth;
	camera_height = imageheight;
	camera_flagverbose = flagverbose;
	Gamma = imagegamma;

	// Iniciar sistema de captura
	if(camera_initcapturesystem()==false){
		return false;
	}
	
	// Alocar memoria para as imagens:
	ppImageRight = new IplImage* [CAMERA_MAXSIZE_IMAGES_QUEUE];
	ppImageLeft = new IplImage* [CAMERA_MAXSIZE_IMAGES_QUEUE];
	for(n=0;n<CAMERA_MAXSIZE_IMAGES_QUEUE;++n){
		ppImageRight[n] = cvCreateImage(cvSize(camera_width,camera_height), IPL_DEPTH_8U, 3);
		ppImageLeft[n]  = cvCreateImage(cvSize(camera_width,camera_height), IPL_DEPTH_8U, 3);
	}

	// Iniciar thread de captura
	grabthreadperiod_ms = grabperiod_ms;
	pthread_create( &camera_grabthreadstructure, NULL, camera_grabthread, NULL);
		
	return true;
}

! \fn int camera_close(void)
* Funcao de encerramento do servidor das cameras.
* \param 
* \return

int camera_close(void)
{
	int n;

	// Encerrar thread de captura
	pthread_mutex_lock( &mutex_grabthreadsharedvariables );
	flaggrabthreadactive = false;
	pthread_mutex_unlock( &mutex_grabthreadsharedvariables );
	pthread_join( camera_grabthreadstructure, NULL);
		
	// Encerrar sistema de captura
	camera_closecapturesystem();

	// Liberar memoria das imagens:
	for(n=0;n<CAMERA_MAXSIZE_IMAGES_QUEUE;++n){
		cvReleaseImage(&ppImageLeft[n]);
		cvReleaseImage(&ppImageRight[n]);
	}
	delete [] ppImageLeft;
	delete [] ppImageRight;
	
	return true;
}

****************************************************************************
******************************************************************************
** Funcoes de interface
******************************************************************************
****************************************************************************
! \fn int camera_getimagepair(IplImage **ppleft, IplImage **ppright)
* Funcao que retorna o par de imagens mais atual.
* \param 
* \return

int camera_getimagepair(IplImage **ppleft, IplImage **ppright)
{
	int flaggrabactive, imageindex;

	pthread_mutex_lock( &mutex_grabthreadsharedvariables );
	flaggrabactive = flaggrabthreadactive;
	imageindex = currentimageindex;
	pthread_mutex_unlock( &mutex_grabthreadsharedvariables );

	if(flaggrabactive==true){
		if(ppleft != NULL){
			*ppleft = ppImageLeft[imageindex];
		}
		if(ppright != NULL){
			*ppright = ppImageRight[imageindex];
		}
		return true;
	}
	
	return false;
}


****************************************************************************
******************************************************************************
** Funcoes internas
******************************************************************************
****************************************************************************
int camera_initcapturesystem(void)
{
	int n;

	for (n=0; n<256; n++){
		GammaTable[n] = (unsigned char)(255.0*(pow((double)n/255.0, Gamma)));
    	}

       printf("\n\n*** Inicialização do módulo camera");
       printf("\n*** Framegrabber: %s",svsVideoIdent);
//       printf(svsVideoIdent);

// Get the svsVideoImages object from the currently loaded camera interface
	videoObject = getVideoObject();
	videoObject->ReadParams((char*)"/home/shakey/Desktop/SVS/parametroscalibracaocamerateste.ini");

// Open the stereo device 
	bool ret;
	ret = videoObject->Open();
	if (!ret){
		printf("\n*** Erro: o sistema de cameras nao pode ser iniciado.");
		camera_closecapturesystem();
		return false;
    	}
	
	printf("\n*** Sistema de cameras iniciado: %i cameras encontradas", videoObject->Enumerate());

// Set camera parameters *after* opening the device
	videoObject->SetColor(40,40); 
	videoObject->SetSize(camera_width, camera_height);	
	videoObject->SetExposure(83,100); 
	videoObject->SetBrightness(0,30); 
	videoObject->SetRate(30); 

	ret = videoObject->Start();
	if (!ret){
		printf("\n*** Erro: nao pode ser iniciada captura do sistema de cameras.");
		camera_closecapturesystem();
		return false;
    }

// Set up acquisition to rectify the image
	ret = videoObject->SetRect(true);
	if (!ret){
		printf("\n*** Aviso: nao pode ser feita retificacao das imagens.");
    }

	return true;
}

void camera_closecapturesystem(void)
{
	videoObject->Close();
}

void *camera_grabthread(void *ptr)
{
	int flaggrabactive;
	int nfailures = 0;
	timestruct_t time_t, time_tictoc;
	double executiontime, t;
	double Tusleep;
	
	printf("\n*** Iniciada thread de captura de imagens");
	flaggrabactive = true;
	pthread_mutex_lock( &mutex_grabthreadsharedvariables );
	flaggrabthreadactive = flaggrabactive;
	pthread_mutex_unlock( &mutex_grabthreadsharedvariables );

	time_reset(&time_t);
	while(flaggrabactive==true){
		// Coleta instante inicial
		time_reset(&time_tictoc);
		
		// Faz aquisicao de um par de imagens:
		stereoImage = videoObject->GetImage(5000);
		if (stereoImage == NULL){
			printf("\n*** Aviso: falha na captura de imagens (%i/5)",++nfailures);
			if(nfailures==5){
				printf("\n*** Aviso: sistema de aquisicao de imagens desativado.");
				flaggrabactive=false;
				pthread_mutex_lock( &mutex_grabthreadsharedvariables );
				flaggrabthreadactive = flaggrabactive;
				pthread_mutex_unlock( &mutex_grabthreadsharedvariables );
			}
		}
		else{
			pthread_mutex_lock( &mutex_grabthreadsharedvariables );
			if(++currentimageindex >= CAMERA_MAXSIZE_IMAGES_QUEUE){
				currentimageindex = 0;
			}
			camera_svstocvimage(stereoImage->Color(), camera_height, camera_width, ppImageLeft[currentimageindex]);
			camera_svstocvimage(stereoImage->ColorRight(), camera_height, camera_width, ppImageRight[currentimageindex]);
		
			pthread_mutex_unlock( &mutex_grabthreadsharedvariables );
			if(camera_flagverbose) printf("\n currentimageindex = %i",currentimageindex);
		
		}
		// Armazena solicitacao de parada
		pthread_mutex_lock( &mutex_grabthreadsharedvariables );
		flaggrabactive = flaggrabthreadactive;
		pthread_mutex_unlock( &mutex_grabthreadsharedvariables );

		// Coleta instante final
		executiontime = time_gettime(&time_tictoc);
		t = time_gettime(&time_t);
		if(camera_flagverbose) printf("\n t = %f s, executiontime = %f s",t,executiontime);

		// Coloca a thread para dormir pelo tempo necessario
		Tusleep = ((grabthreadperiod_ms-executiontime*1000) * 1000);
		if(Tusleep < 10000){
			Tusleep = 10000;
		}
		if(camera_flagverbose) printf("\n tempo para usleep = %f us",Tusleep);
		usleep((useconds_t)(Tusleep));
	}

	//stereoImage->SaveToFile("svsimagefiles");
	//camera_printsvsimageinfo(&stereoImage->ip);

	printf("\n*** Encerrada thread de captura de imagens");

	return NULL;
}

void camera_svstocvimage(unsigned long *pImSVS, unsigned long Height, unsigned long Width, IplImage *pImCV)
{
	// Imagem de entrada tem que ser colorida
	pImCV->height = Height;		
	pImCV->width = Width;		
	unsigned long ulong;
	uchar *puchardest;
	uchar *pucharorig;
	
	pucharorig = (uchar *)(&ulong);
	for(long i=0;i<pImCV->height;++i){
		for(long j=0;j<pImCV->width;++j){
			ulong = (pImSVS + Width*i)[j];
			puchardest = (uchar *)(pImCV->imageData + pImCV->widthStep*i);
			(puchardest)[j*3+0] = GammaTable[(pucharorig)[2]];
			(puchardest)[j*3+1] = GammaTable[(pucharorig)[1]];
			(puchardest)[j*3+2] = GammaTable[(pucharorig)[0]];
		}
	}
}


void camera_svstocvimage(unsigned long *pImSVS, unsigned long Height, unsigned long Width, IplImage *pImCV)
{

	// Imagem de entrada tem que ser colorida
	pImCV->height = Height;		
	pImCV->width = Width;		
	unsigned long ulong;

	for(int i=0;i<pImCV->height;++i){
		for(int j=0;j<pImCV->width;++j){
			ulong = (pImSVS + Width*i)[j];
			((uchar*)(pImCV->imageData + pImCV->widthStep*i))[j*3+0] = GammaTable[((unsigned char*)(&ulong))[2]];
			((uchar*)(pImCV->imageData + pImCV->widthStep*i))[j*3+1] = GammaTable[((unsigned char*)(&ulong))[1]];
			((uchar*)(pImCV->imageData + pImCV->widthStep*i))[j*3+2] = GammaTable[((unsigned char*)(&ulong))[0]];
		}
	}
}

void camera_printcvimageinfo(IplImage *pImCV)
{
	printf("\n OpenCV Image Info:");
	printf("\n    nSize = %i",pImCV->nSize);
	printf("\n    nChannels = %i",pImCV->nChannels);
	printf("\n    depth = %i",pImCV->depth);
	printf("\n    dataOrder = %i",pImCV->dataOrder);
	printf("\n    origin = %i",pImCV->origin);
	printf("\n    width = %i",pImCV->width);
	printf("\n    height = %i",pImCV->height);
	printf("\n    imageSize = %i",pImCV->imageSize);
	printf("\n    widthStep = %i",pImCV->widthStep);
}

void camera_printsvsimageinfo(svsImageParams *pIp)
{
	printf("\n SVS Image Info:");
	printf("\n    linelen = %i",pIp->linelen);
	printf("\n    lines = %i",pIp->lines);
	printf("\n    ix = %i",pIp->ix);
	printf("\n    iy = %i",pIp->iy);
	printf("\n    width = %i",pIp->width);
	printf("\n    height = %i",pIp->height);
	printf("\n    vergence = %f",pIp->vergence);
	printf("\n    gamma = %f",pIp->gamma);
}
*/
*/
