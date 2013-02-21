/* Prem NIRMAL
   Fordham RCV Lab
   Fordham University
   Bronx NY 10458

   ransac.cpp
   IMPLEMENTATION OF RANSAC ALGORITHM TO OBTAIN DEPTH FROM DEPTHIMAGE.
   OUTLIER REMOVAL WILL GIVE A RELIABLE DEPTH VALUE.

   SEPT 2012

*/

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <cmath>

using namespace cv;
using namespace std;

#define ROW 500
#define COL 800

#define POINTS 100 // number of points around feature
#define ERR 0.1  // max error value allowed
#define n 70     // number of random points
#define S 50     // minimum size of data-set required to exit
#define MAX_IT 50 // max number of iterations

/*** DEFS ******************************************************************/
CvScalar performRANSAC(CvScalar P, int row, int col, IplImage *depthImage);
bool isMember(int a, vector<int> vec);
/***************************************************************************/

int main (int argc, char **argv)
{
  IplImage *Depth=0;
  Depth=cvLoadImage("depth.jpg");

  CvScalar point;
  point=cvGet2D(Depth,ROW,COL);

  cout<<"Original depth of point: "<<point.val[2]<<endl;
  point=performRANSAC(point,ROW,COL,Depth);
  cout<<"Depth of point after RANSAC op: "<<point.val[2]<<endl;

  return 0;
}

/***************************************************************************/

/* perform RANSAC operation on depth point in depthImage
   to get depth as an average of surrounding 
   NOTE: this only performs RANSAC on depth i.e. Z (not X,Y)
   i.e. P.val[2] 
   -PN 09/12 */
CvScalar performRANSAC(CvScalar P, int row, int col, IplImage *depthImage)
{
  vector<double> M, N, K;
  double x=0;
  double r,consensus;
  int random_number;
  vector<int> random_numbers;
  vector<double>::iterator it;
  int count=0;
  
  cout<<"#POINTS="<<POINTS<<", ERR="<<ERR<<", n="<<n
      <<", S="<<S<<endl;

  do
    {
      // collect depth points
      for(int i=row-(sqrt(POINTS)/2);i<row+(sqrt(POINTS)/2);i++)
	for(int j=col-(sqrt(POINTS)/2);j<col+(sqrt(POINTS)/2);j++)
	  M.push_back(cvGet2D(depthImage,i,j).val[2]);

      // calculate random points and mean from that set

      srand(time(0));
      for(int k=0;k<n;k++)
	{
	  random_number=rand()%100;
	  random_numbers.push_back(random_number);
	  r=M.at(random_number);
	  N.push_back(r);
	  x+=r;
	}
      x/=n;
      cout<<"n="<<n<<endl;
      cout<<"x="<<x<<endl;

      //populate K
      for(it=M.begin();it!=M.end();it++)
	if(fabs(*it-x)<ERR)
	  K.push_back(*it);
    
      cout<<"K.size()="<<K.size()<<endl;
      
      // calculate mean of K
      for(it=K.begin();it!=K.end();it++)
	consensus+=*it;
      consensus/=K.size();
      cout<<"consensus="<<consensus<<endl;
      count++;
    }
  while(K.size()<S || count>=50);

  if(count>=50)
    cout<<"RANSAC FAIL."<<endl;
  else
    cout<<"RANSAC COMPLETE. Took "<<count<<" iterations."<<endl;

  P.val[2]=consensus;

  return P;
}

bool isMember(int a, vector<int> vec)
{
  vector<int>::iterator it;
  for(it=vec.begin();it!=vec.end();it++)
    if(a==*it)
      return 1;
  return 0;
}
