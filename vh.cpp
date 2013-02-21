/* PREM NIRMAL
   Fordham RCV Lab
   Fordham University
   Bronx NY 10458
*/

#include "vh.h"

#define IMG_WIDTH 1024
#define IMG_HEIGHT 768

#define HOMEIMAGE "stereo-data/home.pgm"
#define HOMEKEY "stereo-data/home.key"

#define MIN_MATCHES 1

/***** RANSAC *****/
#define POINTS 16 // number of points around feature
#define ERR 400 // max error value allowed (mm)
#define L 5 // max number of iterations
/***** RANSAC *****/

/***** RANSAC2 *****/
#define ERR2 500 // max error value allowed (mm)
#define L2 50 // max number of iterations
/***** RANSAC2 *****/

/*** PAN-TILT ***/
#define panStart -135
#define panEnd 135
#define panInc 67
#define defTilt 5
/*** PAN-TILT ***/

#define PERCENT 40


IplImage * getHomeDepth(ArDPPTU *myPTZ)
{
  cout<<"Initializing stereo camera.."<<endl;
  /*** "WARM UP STEREOCAMERA", if the robot has just been powered on, then the first
       picture will have less illumination. ***/
  stereoCam *myStereoCamera = new stereoCam();
  IplImage *raw,*disparity,*homeDepth,*frame;
  sleep(0.5);
  raw=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_8U,3);
  disparity=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_8U,1);
  homeDepth=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_64F,3);
  frame=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_8U,3);
  homeDepth=myStereoCamera->doStereoFrame(frame,disparity,homeDepth,0,0,0,0,0);
  sleep(0.5);
  delete myStereoCamera;
  cvReleaseImage(&raw);
  cvReleaseImage(&disparity);
  cvReleaseImage(&homeDepth);
  cvReleaseImage(&frame);
  /**********************************************************************************/

  cout<<"Capturing home image, depth data, and SIFT features.."<<endl;
  
  vector<IplImage*> depthImages,frames;

  IplImage *homeDepthPan, *framePan;
  cout<<"Doing Home scan.."<<endl;

  time_t rawtime;
  struct tm *timeinfo;
  time(&rawtime);
  timeinfo=localtime(&rawtime);

  char imageName[100];
  int count=0;
  myPTZ->panTilt(panStart,defTilt);
  sleep(2);

  // take images using pan-tilt
  for(int i=panStart;i<=panEnd;i+=panInc)
    { 
      IplImage *raw=new IplImage();
      IplImage *disparity=new IplImage();
      IplImage *homeDepth=new IplImage();
      IplImage *frame=new IplImage();
      raw=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_8U,3);
      disparity=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_8U,1);
      homeDepth=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_64F,3);
      frame=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_8U,3);

      count++;
      sprintf(imageName,"stereo-data/singlehomeimage%d%d%d%d%d-#%d.pgm",
	      timeinfo->tm_year+2000, timeinfo->tm_mon+1, timeinfo->tm_mday,
	      timeinfo->tm_hour+1, timeinfo->tm_min+1,count);
      cout<<"Panning to "<<i<<", tilting "<<defTilt<<endl;
      myPTZ->panTilt(i,defTilt);
      ArUtil::sleep(2000);
      stereoCam *myStereoCamera = new stereoCam();
      homeDepth=myStereoCamera->doStereoFrame(frame,disparity,homeDepth,i,defTilt,0,0,0);
      //      ArUtil::sleep(1500);
      depthImages.push_back(homeDepth);
      frames.push_back(frame);
      cvSaveImage(imageName,frame);
      delete myStereoCamera;
    }
  cout<<"Resetting PTZ.."<<endl;
  myPTZ->reset();

  framePan=cvCreateImage(cvSize(IMG_WIDTH*frames.size(),IMG_HEIGHT),IPL_DEPTH_8U,3);
  homeDepthPan=cvCreateImage(cvSize(IMG_WIDTH*depthImages.size(),IMG_HEIGHT),IPL_DEPTH_64F,3);
  framePan=cocatenateImages(framePan,frames);
  homeDepthPan=cocatenateImages(homeDepthPan,depthImages);

  IplImage *homeDepthPan2,*framePan2;
  homeDepthPan2=cvCreateImage(cvSize((int)homeDepthPan->width*PERCENT/100,
				     (int)homeDepthPan->height*PERCENT/100),IPL_DEPTH_64F,3);
  framePan2=cvCreateImage(cvSize((int)framePan->width*PERCENT/100,
				     (int)framePan->height*PERCENT/100),IPL_DEPTH_8U,3);
  cvResize(homeDepthPan,homeDepthPan2);
  cvResize(framePan,framePan2);
  IplImage *gray=new IplImage();
  gray=cvCreateImage(cvGetSize(framePan2),IPL_DEPTH_8U,1);
  cvCvtColor(framePan2,gray,CV_BGR2GRAY);
  
  cvSaveImage("stereo-data/home.pgm",gray);
  cout<<"Saved 'home.pgm'.."<<endl;
  cout<<"Getting sift features from 'home.pgm' and storing them"
    " as 'home.key'.."<<endl;
  system("./sift <stereo-data/home.pgm > stereo-data/home.key");

  cvReleaseImage(&gray);
  cvReleaseImage(&framePan);
  cvReleaseImage(&framePan2);
  cvReleaseImage(&homeDepthPan);
  frames.clear();
  depthImages.clear();

  return homeDepthPan2;
}

/* Cocatenate a vector of images to make one landscape panorama
   NOTE: No stitching, seam finding, blending, warping involved.
   Just a simple cocatenation.
 */
IplImage * cocatenateImages(IplImage *pan, vector<IplImage*> images)
{
  cout<<"Cocatenating images.."<<endl;
  vector<IplImage*>::reverse_iterator imageit;
  int count=0;
  CvScalar p;
  for(imageit=images.rbegin();imageit!=images.rend();imageit++)
    {
      IplImage *img=*imageit;
      for(int u=1;u<img->width;u++)
	for(int v=1;v<img->height;v++)
	  {
	    p=cvGet2D(img,v,u);
	    cvSet2D(pan,v,u+(count*IMG_WIDTH),p);
	  }
      count++;
      cvReleaseImage(&img);
    }
  return pan;
}

/* Given a pair of images and their keypoints, pick the first keypoint
   from one image and find its closest match in the second set of
   keypoints.
   Also given is the depthImage of the currentImage and snapshotImage. Using
   the x,y,z parameters of the matched SIFT features from the depthImages,
   calculate the change in orientation from current image to goal image.
   Return theta in degrees.
*/
Vector2d FindMatches(Image im1, Keypoint keys1, IplImage *homeDepth, Image im2, Keypoint keys2, IplImage *currentDepth, int imageCount)
{
  Vector2d zero,motion;
  zero << 0.0,0.0;
  
  Image result;
  /* Create a new image that joins the two images vertically/ */
  result=CombineImagesVertically(im1,im2);
  Keypoint k,match;
  int count=0, numberOfMatches=0;
  vector<double> theta,distance;
  double thetaAverage=0,distanceAverage=0;
  
  /* Match the keys in list keys1 to their best matches in keys2 */
  for(k=keys1;k!=NULL;k=k->next)
    {
      match=CheckForMatch(k,keys2);  
      if(match!=NULL) 
	{
	  int gRow=k->row; //location of keypoint in goal image
	  int gCol=k->col;
	  int cRow=match->row; //location of keypoint in current image
	  int cCol=match->col;
	  
	  CvScalar G,C; // store stereodata (depth and x value) as a CvScalar
	  G=cvGet2D(homeDepth,gRow,gCol);
	  C=cvGet2D(currentDepth,cRow,cCol);
	  // /**** RANSAC filtering on matched SIFT points ****/
	  // G.val[1]=performRANSAC(G.val[1],gRow,gCol,homeDepth,0);
	  // G.val[0]=performRANSAC(G.val[0],gRow,gCol,homeDepth,1);
	  // C.val[1]=performRANSAC(C.val[1],cRow,cCol,currentDepth,0);
	  // C.val[0]=performRANSAC(C.val[0],cRow,cCol,currentDepth,1);
	  
	  // check whether values are not nan, and not zero (valid values in depthimage)
	  if(G.val[0]!=0 && G.val[0]==G.val[0] && G.val[1]!=0 && G.val[1]==G.val[1])
	    if(C.val[0]!=0 && C.val[0]==C.val[0] && C.val[1]!=0 && C.val[1]==C.val[1])
	      {
		theta.push_back(calculateTheta(G,C));
		distance.push_back(calculateDistance(G,C));
		DrawLine(result, (int) k->row, (int) k->col,
			 (int) (match->row + im1->rows), (int) match->col);
		count++;
	      }

	  numberOfMatches++;

	}//end if(match..
    }//end for(k=keys1..

  /** store result **/
  FILE *matched;
  char matchfileName[100];
  sprintf(matchfileName,"stereo-data/matched%d.pgm",imageCount);
  matched=fopen(matchfileName,"w");
  WritePGM(matched,result);
  fclose(matched);
  cout<<"Wrote '"<<matchfileName<<"'.\n";
  /**--------------**/

  /** calculate average theta **/
  int thetaCount=0;
  vector<double>::iterator t;
  for(t=theta.begin();t!=theta.end();t++)
    if(fabs(*t)<1000 && *t!=0)
      if(*t==*t) // prevent nan
	{
	  thetaCount++;
	  thetaAverage+=*t;
	}
  if(thetaCount!=0)
    thetaAverage/=thetaCount;
  else
    thetaAverage=0;
  /** ----------------------- **/

  /** calculate average distance **/
  distanceAverage=0;
  vector<double>::iterator d;
  for (d=distance.begin();d!=distance.end();d++)
    distanceAverage+=*d;
  distanceAverage/=distance.size();

  cout<<"DistanceAverage:"<<distanceAverage<<endl;
  /** ----------------------- **/

  printf("Found a total of %d matches.\n",numberOfMatches);
  printf("Out of which, %d were valid points in depthImage.\n",count);
  
  if(numberOfMatches==0)
    {
      cout<<"\tFindMatches: No matches found.\n";
      return zero;
    }
  
  if(count<MIN_MATCHES)
    {
      cout<<"\tFindMatches: Not enough valid points in depthImage.\n";
      return zero;
    }

  //convert to degrees
  thetaAverage=thetaAverage*180/M_PI;

  motion << distanceAverage,thetaAverage;

  theta.clear(); distance.clear();

  //  free(result->pixels);
  //free(result);   
  free_img(result);
  //  free(k); free(match);
  
  return motion;
  
}//end FindMatches..


/* This searches through the keypoints in klist for the two closest
   matches to key.  If the closest is less than 0.6 times distance to
   second closest, then return the closest match.  Otherwise, return
   NULL.
*/
Keypoint CheckForMatch(Keypoint key, Keypoint klist)
{
  int dsq, distsq1 = 100000000, distsq2 = 100000000;
  Keypoint k, minkey = NULL;

  /* Find the two closest matches, and put their squared distances in
     distsq1 and distsq2.
  */
  for (k = klist; k != NULL; k = k->next) {
    dsq = DistSquared(key, k);

    if (dsq < distsq1) {
      distsq2 = distsq1;
      distsq1 = dsq;
      minkey = k;
    } else if (dsq < distsq2) {
      distsq2 = dsq;
    }
  }

  /* Check whether closest distance is less than 0.6 of second. */
  if (10 * 10 * distsq1 < 6 * 6 * distsq2)
    return minkey;
  else return NULL;
}


/* Return squared distance between two keypoint descriptors.
 */
int DistSquared(Keypoint k1, Keypoint k2)
{
  int i, dif, distsq = 0;
  unsigned char *pk1, *pk2;

  pk1 = k1->descrip;
  pk2 = k2->descrip;

  for (i = 0; i < 128; i++) {
    dif = (int) *pk1++ - (int) *pk2++;
    distsq += dif * dif;
  }
  return distsq;
}

double calculateDistance(CvScalar G, CvScalar C)
{
  double zG=G.val[0];
  double zC=C.val[0];
    
  return (zC-zG);
}

/* Calculate angle between keypoint in goal image
   and home image.
   This is the partial movement angle. */
double calculateTheta(CvScalar G, CvScalar C)
{
  double thetaG=atan2(G.val[1],G.val[0]);
  double thetaC=atan2(C.val[1],C.val[0]);
  return thetaC-thetaG;
}

/* Return a new image that contains the two images with im1 above im2.
 */
Image CombineImagesVertically(Image im1, Image im2)
{
  int rows, cols, r, c;
  Image result;

  rows = im1->rows + im2->rows;
  cols = MAX(im1->cols, im2->cols);
  result = CreateImage(rows, cols);

  /* Set all pixels to 0,5, so that blank regions are grey. */
  for (r = 0; r < rows; r++)
    for (c = 0; c < cols; c++)
      result->pixels[r][c] = 0.5;

  /* Copy images into result. */
  for (r = 0; r < im1->rows; r++)
    for (c = 0; c < im1->cols; c++)
      result->pixels[r][c] = im1->pixels[r][c];
  for (r = 0; r < im2->rows; r++)
    for (c = 0; c < im2->cols; c++)
      result->pixels[r + im1->rows][c] = im2->pixels[r][c];

  return result;
}

/* Return a new image that contains the two images with im1 beside im2.  
 */
Image CombineImagesHorizontally(Image im1, Image im2)
{
  int rows, cols, r, c;
  Image result;
  rows = MAX(im1->rows,im2->rows);
  cols = im1->cols+im2->cols;
  result = CreateImage(rows,cols);
  /* Set all pixels to 0,5, so that blank regions are grey. */
  for (r = 0; r < rows; r++)
    for (c = 0; c < cols; c++)
      result->pixels[r][c] = 0.5;
  /* Copy images into result. */
  for (r = 0; r < im1->rows; r++)
    for (c = 0; c < im1->cols; c++)
      result->pixels[r][c] = im1->pixels[r][c];
  for (r = 0; r < im2->rows; r++)
    for (c = 0; c < im2->cols; c++)
      result->pixels[r][c+im1->cols] = im2->pixels[r][c];

  return result;
}


/* Captures current image, converts it to .pgm and extracts SIFT features.
   Then, it compares the current image to the home image, from which it
   determines the desired angle of orientation to rotate the robot.
 */
Vector2d determineMotion(int imageCount,IplImage *homeDepth, ArDPPTU *myPTZ)
{
  Vector2d motion;
  char keypointName[100];
  char grayFileName[100], command[256], imageName[100];
  Image im1 = NULL, im2 = NULL;
  Keypoint k1 = NULL, k2 = NULL;
  time_t rawtime;
  struct tm *timeinfo;
  time(&rawtime);
  timeinfo=localtime(&rawtime);

  //  sprintf(grayFileName,"stereo-data/image%d.pgm",imageCount);
  sprintf(grayFileName,"stereo-data/image%d%d%d%d%d-%d.pgm",
	  timeinfo->tm_year+2000, timeinfo->tm_mon+1, timeinfo->tm_mday,
	  timeinfo->tm_hour+1, timeinfo->tm_min+1,imageCount);

  sprintf(keypointName,"stereo-data/keypoint%d.key",imageCount);
  vector<IplImage*> depthImages,frames;
  IplImage *DepthPan, *framePan;
  int count=0;
  
  cout<<"Doing scan.."<<endl;
  myPTZ->panTilt(panStart,defTilt);
  sleep(2);

  // take images using pan-tilt
  for(int i=panStart;i<=panEnd;i+=panInc)
    { 
      IplImage *raw=new IplImage();
      IplImage *disparity=new IplImage();
      IplImage *Depth=new IplImage();
      IplImage *frame=new IplImage();
      raw=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_8U,3);
      disparity=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_8U,1);
      Depth=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_64F,3);
      frame=cvCreateImage(cvSize(IMG_WIDTH,IMG_HEIGHT),IPL_DEPTH_8U,3);

      count++;
      sprintf(imageName,"stereo-data/singleimage%d%d%d%d%d-#%d.pgm",
	      timeinfo->tm_year+2000, timeinfo->tm_mon+1, timeinfo->tm_mday,
	      timeinfo->tm_hour+1, timeinfo->tm_min+1,count);
      cout<<"Panning to "<<i<<", tilting "<<defTilt<<endl;
      myPTZ->panTilt(i,defTilt);
      ArUtil::sleep(2000);
      stereoCam *myStereoCamera = new stereoCam();
      Depth=myStereoCamera->doStereoFrame(frame,disparity,Depth,i,defTilt,0,0,0);
      //      ArUtil::sleep(1500);
      depthImages.push_back(Depth);
      frames.push_back(frame);
      cvSaveImage(imageName,frame);
      delete myStereoCamera;
    }
  cout<<"Resetting PTZ.."<<endl;
  myPTZ->reset();

  DepthPan=cvCreateImage(cvSize(IMG_WIDTH*depthImages.size(),IMG_HEIGHT),IPL_DEPTH_64F,3);
  framePan=cvCreateImage(cvSize(IMG_WIDTH*frames.size(),IMG_HEIGHT),IPL_DEPTH_8U,3);
  DepthPan=cocatenateImages(DepthPan,depthImages);
  framePan=cocatenateImages(framePan,frames);

  IplImage *DepthPan2,*framePan2;
  DepthPan2=cvCreateImage(cvSize((int)DepthPan->width*PERCENT/100,
				 (int)DepthPan->height*PERCENT/100),IPL_DEPTH_64F,3);
  framePan2=cvCreateImage(cvSize((int)framePan->width*PERCENT/100,
				 (int)framePan->height*PERCENT/100),IPL_DEPTH_8U,3);
  cvResize(DepthPan,DepthPan2);
  cvResize(framePan,framePan2);

  /*** CONVERT TO GRAYSCALE ***/
  IplImage *gray=new IplImage();
  gray=cvCreateImage(cvGetSize(framePan2),IPL_DEPTH_8U,1);
  cvCvtColor(framePan2,gray,CV_BGR2GRAY);
  cvSaveImage(grayFileName,gray);
  cout<<"Wrote '"<<grayFileName<<"'"<<endl;

  /*** OBTAIN SIFT FEATURES ***/
  cout<<"Getting sift features from "<<grayFileName<<" and storing them as "
      <<keypointName<<endl;
  snprintf(command,256,"./sift <%s >%s",grayFileName,keypointName);
  system(command);
  /*** -------------------- ***/

  im1=ReadPGMFile(HOMEIMAGE);
  k1=ReadKeyFile(HOMEKEY);
  im2=ReadPGMFile(grayFileName);
  k2=ReadKeyFile(keypointName);
  
  cout<<"Finding matches...";
  motion=FindMatches(im1,k1,homeDepth,im2,k2,DepthPan2,imageCount);

  //  remove(grayFileName);
  remove(keypointName);
  cvReleaseImage(&DepthPan);
  cvReleaseImage(&DepthPan2);
  cvReleaseImage(&framePan);
  cvReleaseImage(&framePan2);
  cvReleaseImage(&gray);
  frames.clear();
  depthImages.clear();
  //free(im1); free(im2);  
  free_img(im1); free_img(im2);
  //  free(k1); free(k2);

  return motion;
}

/* perform RANSAC operation on depth point in depthImage
   to get depth as an average of surrounding 
   NOTE: this only performs RANSAC on depth or val[0] and val[1]
   (x and y in robot coordinates)
   Change to val[0] and val[2] if using x and z in BumbleBee coordinates
   -PN 10/12 */
float performRANSAC(float P, int row, int col, IplImage *depthImage, bool xORz)
{
  //  cout<<"Original value: "<<P<<endl;
 
  vector<float> M, N, K;
  float x=0;
  float r,consensus;
  int random_number;
  vector<int> random_numbers;
  vector<float>::iterator it;
  int count=0;
  int a,b,c,d,n,S;
  a=b=c=d=sqrt(POINTS)/2;

  //  cout<<"ERR="<<ERR<<endl;

  /* prevent use of points outside image size */
  if(row-a<0)
    a=row-1;
  if(row+b>depthImage->height)
    b=fabs(row-depthImage->height)-1;
  if(col-c<0)
    c=col-1;
  if(col+d>depthImage->width)
    d=fabs(col-depthImage->width)-1;
  /* *************************************** */

  // accumulate points
  for(int i=row-a;i<row+b;i++)
    for(int j=col-c;j<col+d;j++)
      { 
        //cout<<"i,j: "<<i<<","<<j<<endl;
        if(xORz==1)
          M.push_back(cvGet2D(depthImage,i,j).val[0]);
        else
          M.push_back(cvGet2D(depthImage,i,j).val[1]);
      }
  
  n=floor(M.size()/2); // number of random points to collect
  S=floor(n/2); // minimum consensus sample size

  srand(time(0));
  do
    {
      consensus=0; x=0;
      K.clear(); N.clear(); random_numbers.clear();
      
      // accumulate random points and mean from M
      for(int k=0;k<n;k++)
	{
	  random_number=rand()%M.size();
	  random_numbers.push_back(random_number);
	  r=M.at(random_number);
	  N.push_back(r);
	  x+=r;
	}
      x/=n;

      //populate K
      for(it=M.begin();it!=M.end();it++)
	if(fabs(*it-x)<ERR)
	  K.push_back(*it);
    
      // calculate mean of K
      for(it=K.begin();it!=K.end();it++)
	consensus+=*it;
      consensus/=K.size();
      count++;
    }
  while(K.size()<S && count<L);

  //  cout<<"Value after RANSAC:"<<consensus<<endl;
  
  K.clear(); M.clear(); N.clear(); random_numbers.clear();

  if(count>=L)
    return P;
  else if(consensus==0 || fabs(consensus)<5 || consensus!=consensus)
    return P;
  else
    P=consensus;

  return P;
}
/* Perform RANSAC on an input vector<double> and return mean */
double ransacOperation(vector<double> distance)
{
  double average=0;
//   if (distance.size() < 5)
//     {
      vector<double>::iterator d;
      for (d=distance.begin();d!=distance.end();d++)
	average+=*d;
      average/=distance.size();
      return average;
//     }

//   else
//     {
//       vector<double> N, K;
//       float x=0;
//       float r,consensus;
//       int random_number;
//       vector<int> random_numbers;
//       vector<double>::iterator it;
//       int count=0;
//       int n,S;
//       double initialMean=0;
      
//       /* for(it=distance.begin();it!=distance.end();it++) */
//       /*   initialMean+=*it; */
//       /* initialMean/=distance.size(); */
//       /*   cout<<"Initial mean: "<<initialMean<<endl; */
      
//       n=floor(distance.size()/2); // number of random points to collect
//       S=floor(n/2); // minimum consensus sample size
      
//       srand(time(0));
//       do
// 	{
// 	  consensus=0; x=0;
// 	  K.clear(); N.clear(); random_numbers.clear();
	  
// 	  // accumulate random points and mean from M
// 	  for(int k=0;k<n;k++)
// 	    {
// 	      random_number=rand()%distance.size();
// 	      random_numbers.push_back(random_number);
// 	      r=distance.at(random_number);
// 	      N.push_back(r);
// 	      x+=r;
// 	    }
// 	  x/=n;
	  
// 	  //populate K
// 	  for(it=distance.begin();it!=distance.end();it++)
// 	    if(fabs(*it-x)<ERR2)
// 	      K.push_back(*it);
	  
// 	  // calculate mean of K
// 	  for(it=K.begin();it!=K.end();it++)
// 	    consensus+=*it;
// 	  consensus/=K.size();
// 	  count++;
// 	}
//       while(K.size()<S && count<L2);
      
//       N.clear(); K.clear(); random_numbers.clear();
      
//       return consensus;
//     }// end else
}
