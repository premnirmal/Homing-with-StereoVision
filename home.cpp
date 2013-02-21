/* PREM NIRMAL
   Fordham RCV Lab
   Fordham University
   Bronx NY 10458
*/
/*
  09/2012
  Visual Homing using Stereo-Vision.
*/

#include <cstdio>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "Aria.h"

#include "vh.h"

using namespace cv;
using namespace std;
using namespace Eigen;

ArDPPTU *myPTZ=0;
ArRobot *myRobot=0;

#define HOMEIMAGE "stereo-data/home.pgm"
#define HOMEKEY "stereo-data/home.key"
#define IMG_WIDTH 1024
#define IMG_HEIGHT 768
#define MOVEMENT_UNIT 100 // Move backwards by this unit if no features found
#define MAXVEL 100
#define MAXROTVEL 25
#define EPSILON 2 // If angle to orient < EPSILON, goal reached.
#define TOLERANCE 50 // If distance to goal < TOLERANCE, goal reached.
#define MAX_DISTANCE -50000

/****************************************************************************/

int main (int argc, char **argv)
{
  int imageCount=1,homingSteps=0;
  double theta=5;

  /* ROBOT DECLARATIONS */
  ArArgumentParser parser(&argc, argv); // set up our parser
  ArSimpleConnector simpleConnector(&parser); // set up our simple connector
  ArRobot robot; // declare robot object
  //  ArSonarDevice sonar;
 
  /* INITIALIZATION OF CONNECTION TO ROBOT */
  Aria::init(); // mandatory init
  parser.loadDefaultArguments(); // load the default arguments 
  if (!simpleConnector.parseArgs() // check command line args
      || !parser.checkHelpAndWarnUnparsed(1))
    {    
      simpleConnector.logOptions();
      Aria::shutdown();
      return 1;
    }
  if (!simpleConnector.connectRobot(&robot)) // ask for connection to robot
    {
      printf("Could not connect to robot... exiting\n");
      Aria::shutdown();
      return 1;
    }

  ArDPPTU thePTZ(&robot); // pan-tilt base, mode DPPU
  myPTZ = &thePTZ;
  myPTZ->reset(); 
  ArUtil::sleep(1000);
  
  /* INITIALIZATION OF ROBOT*/
  robot.runAsync(true); // commands processed in separate thread
  robot.enableMotors(); // turn the power to the motors on
  //  robot.addRangeDevice(&sonar); // add sonar (THIS IS UNNECCESARY FOR VH)
  ArUtil::sleep(1000); // sleep time allows robot to initialise sonar, motors, etc

  if (myPTZ==0) {
    printf("PTZ unit not initialized\n");
    return 1;
  }

  ArActionJoydrive *joyDrive = new ArActionJoydrive();
  robot.addAction(joyDrive,100);
  joyDrive->activate();

  /*** CAPTURE HOME IMAGE ***/
  IplImage *homeDepth=0;
  cout<<endl<<"Align robot towards goal location, to capture home image.."<<endl
      <<"Then press ENTER.";
  cin.get();
  homeDepth=getHomeDepth(myPTZ);
  myPTZ->reset(); 
  /*** ------------------ ***/
  
  /**** BEGIN VISUAL HOMING *******************************************/

  /*** SET MAX VELOCITY ***/
  robot.setTransVelMax(MAXVEL);
  robot.setRotVelMax(MAXROTVEL);

  cout<<endl<<"Now position robot away from goal location, "
      <<"and press ENTER to home";
  cin.get();

  double thetaHistory=0;
  Vector2d motion;
  double distance=200;
  bool homingComplete=false;

  while(!homingComplete)
    {
      ArUtil::sleep(1000);
      robot.stop();
      cout<<"Determining motion.."<<endl;
      motion=determineMotion(imageCount,homeDepth,myPTZ);
      sleep(1);
      myPTZ->reset(); 
      distance=motion(0);///2;
      theta=motion(1);///1.5;
      cout<<"Theta = "<<theta<<", d = "<<distance<<endl;
      sleep(1);
      
      if( (fabs(theta)>EPSILON || fabs(distance)>=TOLERANCE)
	  && distance>MAX_DISTANCE )
	{
	  cout<<"Orienting robot by "
	      <<theta<<" degrees... "
	      <<"And moving "<<distance<<" mm..."<<endl;
	  robot.setDeltaHeading(theta);
	  ArUtil::sleep(1000);
	  robot.move(distance);
	  ArUtil::sleep(5000+fabs(distance)*3);
	}
      else if(fabs(theta)<=EPSILON && fabs(distance)<TOLERANCE && theta!=0)
	{
	  cout<<"Theta = "<<theta
	      <<" and distance = "<<distance<<endl
	      <<"Homing complete."<<endl;
	  homingComplete=true;
	}
      
      else
	cout<<"Invalid values of theta and d. Continuing.."<<endl;
      imageCount++;
      thetaHistory=theta;
      homingSteps++;      
    }
  
  cout<<"Number of steps = "<<homingSteps<<endl;

  /**** END VISUAL HOMING *********************************************/

  //  remove(HOMEIMAGE);
  //  remove(HOMEKEY);
  
  cvReleaseImage(&homeDepth);

  Aria::shutdown();
  return 0;
}

/****************************************************************************/
