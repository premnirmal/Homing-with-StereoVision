/** MODIFIED FOR TEST-STREO -PN 09/05/12 **/

/**************************************************************************
 *
 * Title:	simplestereo
 * Copyright:	(C) 2006,2007,2008 Don Murray donm@ptgrey.com
 *
 * Description:
 *
 *    Get an image set from a Bumblebee or Bumblebee2 via DMA transfer
 *    using libdc1394 and process it with the Triclops stereo
 *    library. Based loosely on 'grab_gray_image' from libdc1394 examples.
 *
 *-------------------------------------------------------------------------
 *     License: LGPL
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *************************************************************************/

//=============================================================================
// Copyright © 2006,2007,2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research Inc.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//
//=============================================================================

//=============================================================================
//
// simplestereo.cpp
// HISTORY -- this version modified 4/10/10 to test Bumblebee and OpenCv
// Modified 10/10 to be called from elsewhere
// dml - Fordham Univ 
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <iostream>
#include <time.h> //PN 09/05/12

#include <dc1394/conversions.h>
#include <dc1394/control.h>
#include <dc1394/utils.h>

using namespace std;

#define D2R(x) ((double(x))*3.14159/180.0)
#define R2D(x) ((double(x))*180.0/3.14159)


#define WRITECLOUD // ifdef'ed will re-write the point clound on each frame
#define OPENCV

#ifdef OPENCV
#include <cv.h>
#include <highgui.h>
#endif

bool gWritePointCloud=false;
bool gWriteImages=false;

//=============================================================================
// PGR Includes
//=============================================================================
#include "pgr_registers.h"
#include "pgr_stereocam.h"

#include "stereoCamera.h"

//=============================================================================
// a simple function to write a .pgm file
int
writePgm( char* 	szFilename,
	  unsigned char* pucBuffer,
	  int		width,
	  int		height )
{
  FILE* stream;
  stream = fopen( szFilename, "wb" );
  if( stream == NULL)
    {
      perror( "Can't open image file" );
      return 1;
    }

  fprintf( stream, "P5\n%u %u 255\n", width, height );
  fwrite( pucBuffer, width, height, stream );
  fclose( stream );
  return 0;
}

//=============================================================================
// a simple function to write a .ppm file
int
writePpm( char* 	szFilename,
	  unsigned char* pucBuffer,
	  int		width,
	  int		height )
{
  FILE* stream;
  stream = fopen( szFilename, "wb" );
  if( stream == NULL)
    {
      perror( "Can't open image file" );
      return 1;
    }

  fprintf( stream, "P6\n%u %u 255\n", width, height );
  fwrite( pucBuffer, 3*width, height, stream );
  fclose( stream );
  return 0;
}


//=============================================================================
// cleanup_and_exit()
// This is called when the program exits and destroys the existing connections
// to the 1394 drivers
#if 0

void
cleanup_and_exit( dc1394camera_t* camera )
{
  dc1394_capture_stop( camera );
  dc1394_video_set_transmission( camera, DC1394_OFF );
  dc1394_camera_free( camera );
  exit( 0 );
}

/* Routine from Triclops manual to print an error message */

void printTriclopsError(TriclopsError e) {
  char eString[1024];
  sprintf(eString,"Triclips(%d): %s\n\n",e,
	  triclopsErrorToString(e));
  //  sleep(5);
}
#endif

/* Macro to test for error and print error message */

#define TRIERR(e) { if ( e != TriclopsErrorOk ) {	\
      printTriclopsError(e);				\
      triclopsDestroyContext( triclops );		\
      cleanup_and_exit( camera ); }}




void stereoCam::printTriclopsError(TriclopsError e) {
  char eString[1024];
  sprintf(eString,"Triclips(%d): %s\n\n",e,
	  triclopsErrorToString(e));
  //    sleep(5);
}

void stereoCam::cleanup_and_exit( dc1394camera_t* camera )
{
  dc1394_capture_stop( camera );
  dc1394_video_set_transmission( camera, DC1394_OFF );
  dc1394_camera_free( camera );
}

/*----------------- BumbleBee P3AT kinematics ----------------*/

#define eX 59   // displ X center eye to center head
#define eY 56.5 // displ Y center eye to center PT axis
#define pY 93   // displ Y center PT axis to topplate robot
#define pZ 60   // displ Z center PT axis to front center robot

void stereoCam::transformBumbleBee2World(double from[3], double to[3], double pan, 
					 double tilt, double rx, double ry, double rth) {
  double tx, ty, tz;
  //  printf("BB transform: BB(%5.1lf,%5.1lf,%5.1lf)->", from[0],from[1],from[2]);

  // move the point to the center of the PT unit
  to[0] = 1000*from[0] - eX; //Reye to Hcenter front
  to[1] = 1000*from[1] + eY; //Hcenter down Y to center PT front
  to[2] = 1000*from[2]; //PT center front to PT center center
  // do pan around Y
  double s = sin(D2R(pan)), c = cos(D2R(pan));
  tx = c*to[0]-s*to[2];
  tz = s*to[0]+c*to[2];
  to[0]=tx; to[2]=tz;
  // do tilt around X
  s = sin(D2R(tilt)); c = cos(D2R(tilt));
  ty = c*to[1]-s*to[2];
  tz = s*to[1]+c*to[2];
  to[1]=ty; to[2]=tz;
  // move the point to the center of the robot proj on ground
  to[1] += pY + 279.0; // PT center to topplate center to ground proj PT center center
  to[2] += 255-pZ; // ground proj PT center center to ground proj robot center center
  // rotate the axis from BB to robot
  ty = -to[0]; tx = to[2]; tz = to[1];
  to[0]=tx; to[1]=ty; to[2]=-tz;

  //printf("R(%5.1lf,%5.1lf,%5.1lf)->",to[0],to[1],to[2]);
  // apply the robot transformation
  // rotate the XY axis by -theta
  // s = sin(D2R(rth)); c = cos(D2R(rth));
  // tx = c*to[0]-s*to[1];
  // ty = s*to[0]+c*to[1];
  // to[0]=tx; to[1]=ty;
  // // transform by origin
  // to[0] += rx;
  // to[1] += ry;

  //printf("W(%5.1lf,%5.1lf,%5.1lf).\n",to[0], to[1], to[2]);
  return;

}

void stereoCam::transformWorld2BumbleBee(double from[3], double to[3], double pan, 
					 double tilt, double rx, double ry, double rth) {
  double tx, ty, tz;
  //printf("Inv BB transform: WW(%5.1lf,%5.1lf,%5.1lf)->", from[0],from[1],from[2]);

  // transform out robot origin
  to[0] = from[0] - rx;
  to[1] = from[1] - ry;
  to[2] = from[2];
  // transform out robot rotation
  double s = sin(D2R(-rth)), c = cos(D2R(-rth));
  tx = c*to[0]-s*to[1];
  ty = s*to[0]+c*to[1];
  to[0]=tx; to[1]=ty;
  // rotate from robot to BB
  tx=to[0]; ty=to[1]; tz = -to[2];
  to[1]=tz; to[2]=tx; to[0]=-ty;
  // move back up from the robot to the camera
  to[2] -= (255-pZ);
  to[1] -= (pY+279.0);
  // reverse the tilt
  s = sin(D2R(-tilt)); c = cos(D2R(-tilt));
  ty = c*to[1]-s*to[2];
  tz = s*to[1]+c*to[2];
  to[1]=ty; to[2]=tz;
  // reverse the pan
  s = sin(D2R(-pan)); c = cos(D2R(-pan));
  tx = c*to[0]-s*to[2];
  tz = s*to[0]+c*to[2];
  to[0]=tx; to[2]=tz;
  // move to the center of the Reye
  to[0] = (to[0]-eX)/1000.0;
  to[1] = (to[1]-eY)/1000.0;
  to[2] /= 1000.0;

  //printf("BB(%5.1lf,%5.1lf,%5.1lf).\n",to[0], to[1], to[2]);
}




stereoCam::stereoCam() {

   
  // Find cameras on the 1394 buses
  d = dc1394_new ();

  // Enumerate cameras connected to the PC
  err = dc1394_camera_enumerate (d, &list);
  if ( err != DC1394_SUCCESS )
    {
      fprintf( stderr, "Unable to look for cameras\n\n"
	       "Please check \n"
	       "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' "
	       "are loaded \n"
	       "  - if you have read/write access to /dev/raw1394\n\n");
      nThisCam=-1;
      //return 1;
    }

  if (list->num == 0)
    {
      fprintf( stderr, "No cameras found!\n");
      nThisCam=-1;
      //return 1;
    }

  //  printf( "There were %d camera(s) found attached to your PC\n", list->num  );

  // Identify cameras. Use the first stereo camera that is found
  for ( nThisCam = 0; nThisCam < list->num; nThisCam++ )
    {
      camera = dc1394_camera_new(d, list->ids[nThisCam].guid);

      if(!camera)
        {
	  printf("Failed to initialize camera with guid %llx", 
		 list->ids[nThisCam].guid);
	  continue;
        }

      //  printf( "Camera %d model = '%s'\n", nThisCam, camera->model );

      if ( isStereoCamera(camera))
        {
	  //	  printf( "Using this camera\n" );
	  break;
        }
      dc1394_camera_free(camera);
    }

  if ( nThisCam == list->num )
    {
      printf( "No stereo cameras were detected\n" );
      nThisCam=-1;
      //return 0;
    }
  // Free memory used by the camera list
  dc1394_camera_free_list (list);


  // query information about this stereo camera
  err = queryStereoCamera( camera, &stereoCamera );
  if ( err != DC1394_SUCCESS )
    {
      fprintf( stderr, "Cannot query all information from camera\n" );
      cleanup_and_exit( camera );
    }

  if ( stereoCamera.nBytesPerPixel != 2 )
    {
      // can't handle XB3 3 bytes per pixel
      fprintf( stderr,
	       "Example not updated to work with XB3 in 3 camera mode yet!\n" );
      cleanup_and_exit( stereoCamera.camera );
    }

  // set the capture mode
  //  printf( "Setting stereo video capture mode\n" );
  err = setStereoVideoCapture( &stereoCamera );
  if ( err != DC1394_SUCCESS )
    {
      fprintf( stderr, "Could not set up video capture mode\n" );
      cleanup_and_exit( stereoCamera.camera );
    }

  // have the camera start sending us data
  //  printf( "Start transmission\n" );
  err = startTransmission( &stereoCamera );
  if ( err != DC1394_SUCCESS )
    {
      fprintf( stderr, "Unable to start camera iso transmission\n" );
      cleanup_and_exit( stereoCamera.camera );
    }

  // give the auto-gain algorithms a chance to catch up
  //   printf( "Giving auto-gain algorithm a chance to stabilize\n" );
  //   sleep( 5 );

  // Allocate all the buffers.
  // Unfortunately color processing is a bit inefficient 
  // because of the number of
  // data copies.  Color data needs to be
  // - de-interleaved into separate bayer tile images
  // - color processed into RGB images
  // - de-interleaved to extract the green channel 
  //   for stereo (or other mono conversion)

  // size of buffer for all images at mono8
  // nBufferSize=480*640*stereoCamera.nBytesPerPixel; // PN 08/30/12
  nBufferSize = stereoCamera.nRows *
    stereoCamera.nCols *
    stereoCamera.nBytesPerPixel;
  
  //  printf("Stereo Camera is %d rows and %d columns.\n", stereoCamera.nRows, stereoCamera.nCols);

  // allocate a buffer to hold the de-interleaved images
  pucDeInterlacedBuffer = new unsigned char[ nBufferSize ];
  pucRGBBuffer 	= new unsigned char[ 3 * nBufferSize ];
  pucGreenBuffer 	= new unsigned char[ nBufferSize ];

  // do stereo processing

  //  printf( "Getting TriclopsContext from camera (slowly)... \n" );
  e = getTriclopsContextFromCamera( &stereoCamera, &triclops );
  TRIERR(e);
  // printf( "...got it\n" );

  e=triclopsSetResolution(triclops,768, 1024); // Full Resolution of the stereo camera
  TRIERR(e);
  
  // make sure we are in subpixel mode
  e = triclopsSetSubpixelInterpolation( triclops, 1 );
  TRIERR(e);
  e = triclopsSetDisparityMapping(triclops,0,60);
  TRIERR(e);
  e = triclopsSetDisparityMappingOn(triclops,1);
  TRIERR(e);
  e = triclopsSetUniquenessValidationMapping(triclops,0);
  TRIERR(e);
  e = triclopsSetTextureValidationMapping(triclops,0);
  TRIERR(e);


}


stereoCam::~stereoCam(){
  //  printf( "Stop transmission\n" );

  triclopsDestroyContext( triclops );	 

  //  Stop data transmission
  if ( dc1394_video_set_transmission( stereoCamera.camera, DC1394_OFF ) 
       != DC1394_SUCCESS )
    {
      fprintf( stderr, "Couldn't stop the camera?\n" );
    }


  delete[] pucDeInterlacedBuffer;
  if ( pucRGBBuffer )
    delete[] pucRGBBuffer;
  if ( pucGreenBuffer )
    delete[] pucGreenBuffer;

  // close camera
  cleanup_and_exit( camera );

}

/* version that takes no robot information */
IplImage* stereoCam::doStereoFrame(
#ifdef OPENCV
			      IplImage *frame, 
			      IplImage *disparity, 
			      IplImage *depth
			      /*FILE *depthData*/
			      /* FILE *out*/)
#endif
{
  doStereoFrame(frame, disparity, depth,/* depthData, out,*/ 0.0,0.0, 0.0,0.0,0.0);
}


/* version that takes the DPPU pan and tilt, and the robot pose */
IplImage* stereoCam::doStereoFrame(
#ifdef OPENCV
			      IplImage *frame, 
			      IplImage *disparity, 
			      IplImage *depth,
			      /*FILE *depthData,*/
			      /*FILE *out,*/
			      double pan, double tilt, double rx, double ry, double rth)
#endif
{
  //  if ( inRoutine() ) return; // protected routine
  
  for (int i=0; i<1; i++) {
    pucRightRGB	= NULL;
    pucLeftRGB	= NULL;
    pucCenterRGB	= NULL;

    extractImagesColor( &stereoCamera,
			DC1394_BAYER_METHOD_NEAREST,
			pucDeInterlacedBuffer,
			pucRGBBuffer,
			pucGreenBuffer,
			&pucRightRGB,
			&pucLeftRGB,
			&pucCenterRGB,
			&input );
  }

     
  //#if 0 // write out the raw images from the camera
  // if ( !writePpm( "output/yright.ppm", pucRightRGB, 
  // 		  stereoCamera.nCols, stereoCamera.nRows ) )
  //   printf( "wrote right.ppm\n" );
  // if ( !writePpm( "output/yleft.ppm", pucLeftRGB, 
  // 		  stereoCamera.nCols, stereoCamera.nRows ) )
  //   printf( "wrote left.ppm\n" );
  //#endif


  e = triclopsRectify( triclops, &input );
  TRIERR(e);
  //     e = triclopsPreprocess(triclops, &input);
  //     TRIERR(e);

  /************************************/
  e = triclopsStereo( triclops );
  TRIERR(e);
  /************************************/

  //#if 0
  
  // get and save the rectified and disparity images
  e=triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, 
		      &image );
  TRIERR(e);
  // e=triclopsSaveImage( &image, "output/yrectified.pgm" );
  // TRIERR(e);
  // printf( "wrote 'rectified.pgm'\n" );
  
  //#endif


  e=triclopsGetImage16( triclops, TriImg16_DISPARITY, 
			TriCam_REFERENCE, &image16 );
  TRIERR(e);

  //#if 0 // save disparity image -- warning its 16bit not 8bit!
  //  if (gWriteImages) {
  //e=triclopsSaveImage16( &image16, "output/ydisparity.pgm" );
  // TRIERR(e);
  // printf( "wrote 'disparity.pgm'\n" );
  // }
  //#endif
  outRoutine(); // allow other access to camera  now

  int nPoints=0;
  int iPixelInc = image16.rowinc/2;
  int rPixelInc = 3*stereoCamera.nCols;

  // /** HEADER FOR DEPTHDATA FILE **/
  // struct tm *local;
  // time_t t;
  // t = time(NULL);
  // local = localtime(&t);
  // fprintf(depthData,"Depth Data for Stereo Camera on %sWith pan=%.1f and tilt=%.1f.\n\n",asctime(local),pan,tilt);
  // /** END -- HEADER FOR DEPTHDATA FILE **/

  for ( int iRow = 0; iRow < image16.nrows; iRow++ )    {
    unsigned short* pusRow = image16.data + iRow * iPixelInc;
    unsigned char * rgbRow = pucRightRGB +  iRow * rPixelInc;

    for ( int iCol= 0; iCol < image16.ncols; iCol++ )  {
      unsigned short usDisp = pusRow[ iCol ];
      float x,y,z;            
      double posBB[3], posW[3]; // x,y,z in bumblebee and in world frames
	
#ifdef OPENCV
      cvSet2D(frame,iRow,iCol, // populate RGB image
	      cvScalar(rgbRow[ 3*iCol+2], 
		       rgbRow[ 3*iCol+1], 
		       rgbRow[ 3*iCol  ],0));
      
#endif
      if ( usDisp < 0xFF00 ) { // valid disparity only
	// convert the 16 bit disparity value to floating point x,y,z
	triclopsRCD16ToXYZ( 
			   triclops, 
			   iRow, 
			   iCol, 
			   usDisp, 
			   &x,
			   &y,
			   &z // x,y,z, in bumblebee frame
			    );
	nPoints++; posBB[0]=x; posBB[1]=y; posBB[2]=z;

	  /* make robot position = 0,0,0; so that the function
	     transforms points to ROBOT-FRAME */
	  transformBumbleBee2World(posBB, posW, pan, tilt, 0, 0, 0); 
	  //if (out!=0) 
	    // fprintf(out,
	    // 	    "%f, %f, %f, %d, %d, %d, %d, %d, %d\n",
	    // 	    posW[0], posW[1], posW[2], // write out world frame locations
	    // 	    iRow, iCol, usDisp, 
	    // 	    rgbRow[ 3*iCol], 
	    // 	    rgbRow[ 3*iCol+1], 
	    // 	    rgbRow[ 3*iCol+2]);
#ifdef OPENCV
	    //if (depth!=0) 
	    cvSet2D(depth,iRow,iCol, 
		    // populate depth image with ROBOT-FRAME info
		    cvScalar(double(posW[0]), double(posW[1]), double(posW[2]), 0.0));
	    //fprintf(depthData,"%d, %d, %f, %f, %f\n", iRow, iCol, x,y,z);
#endif

	   
#ifdef OPENCV
      //if (disparity!=0) 
	cvSet2D(disparity,iRow,iCol, 
		// populate disparity image
		cvScalar(usDisp/255,0,0,0));
#endif
      }// end-if valid disparity
      
    }// for iCol
  }// for iRow
  
  return depth;
}
