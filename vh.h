/* PREM NIRMAL
   Fordham RCV Lab
   Fordham University
   Bronx NY 10458
*/

#include <cstdio>
#include <iostream>
#include <cv.h>
#include <cstdlib>
#include <highgui.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Aria.h"

#include "stereoCamera.h"

#include "defs.h"

using namespace cv;
using namespace std;
using namespace Eigen;

/*----------------------------- Defs --------------------------------------*/
IplImage * getHomeDepth(ArDPPTU *myPTZ);
Vector2d FindMatches(Image im1, Keypoint keys1, IplImage *homeDepth, 
		   Image im2, Keypoint keys2, IplImage *currentDepth,
		   int imageCount);
Keypoint CheckForMatch(Keypoint key, Keypoint klist);
int DistSquared(Keypoint k1, Keypoint k2);
double calculateDistance(CvScalar G, CvScalar C);
double calculateTheta(CvScalar G, CvScalar C);
Image CombineImagesHorizontally(Image im1, Image im2);
Image CombineImagesVertically(Image im1, Image im2);
Vector2d determineMotion(int imageCount,IplImage *homeDepth, ArDPPTU *myPTZ);
float performRANSAC(float P, int row, int col, IplImage *depthImage, bool xORz);
IplImage * cocatenateImages(IplImage* pan,vector<IplImage*> images);
double ransacOperation(vector<double> distance);
/*----------------------------- Routines ----------------------------------*/
