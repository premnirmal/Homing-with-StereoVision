#include <dc1394/conversions.h>
#include <dc1394/control.h>
#include <dc1394/utils.h>

#include <cv.h>
#include <highgui.h>

#include "pgr_registers.h"
#include "pgr_stereocam.h"

#define OPENCV

class stereoCam {

public:

  stereoCam();

  ~stereoCam();

#ifdef OPENCV
  IplImage* doStereoFrame(IplImage *frame, IplImage *disparity,
			  IplImage *depth/*, FILE *depthData, FILE *out*/); // takes no camera pose into account

  IplImage* doStereoFrame(IplImage *frame, IplImage *disparity,// takes camera and robot pose into acct
			  IplImage *depth,/* FILE *depthData, FILE *out,*/ double p, double t, double x, double y, double th);
#endif
  /* do geometry of camera as mounted on P3ATs */

void transformBumbleBee2World(double from[3], double to[3], double pan, 
			      double tilt, double rx, double ry, double rth);

void transformWorld2BumbleBee(double from[3], double to[3], double pan, 
			      double tilt, double rx, double ry, double rth);
private:

  dc1394camera_t* 	camera;
  dc1394error_t 	err;
  dc1394_t * d;
  dc1394camera_list_t * list;
  unsigned int nThisCam;
  PGRStereoCamera_t stereoCamera;
   // Allocate all the buffers.
   // Unfortunately color processing is a bit inefficient 
   // because of the number of
   // data copies.  Color data needs to be
   // - de-interleaved into separate bayer tile images
   // - color processed into RGB images
   // - de-interleaved to extract the green channel 
   //   for stereo (or other mono conversion)

   // size of buffer for all images at mono8
  unsigned int   nBufferSize;
   // allocate a buffer to hold the de-interleaved images
  unsigned char* pucDeInterlacedBuffer;
  unsigned char* pucRGBBuffer;
  unsigned char* pucGreenBuffer;

  TriclopsInput input;
  TriclopsError e;
  TriclopsContext triclops;


  unsigned char* pucRightRGB;
  unsigned char* pucLeftRGB;
  unsigned char* pucCenterRGB;

  TriclopsImage image;
  TriclopsImage16 image16;

  bool isInRoutine; // for multithreading

  bool inRoutine() { // check if routine is being called now
    if (isInRoutine) return true;
    isInRoutine=true;
    return false;
  }

  void outRoutine() { // set out of the routine to allow others in
    isInRoutine=false;
  }

  void  cleanup_and_exit( dc1394camera_t* camera );


/* Routine from Triclops manual to print an error message */

  void printTriclopsError(TriclopsError e);


};

