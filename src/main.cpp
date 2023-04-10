#include <iostream>
#include <random>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include "particle.h"
#include <opencv2/imgproc/types_c.h>
using namespace std;
using namespace cv;

void onMouse(int Event,int x,int y,int flags,void* param)
{

    if(Event == EVENT_LBUTTONDOWN)
    {
        cout << "x: " << x << "; y: " << y << endl;
    }
}


int main()
{
    VideoCapture video("/home/yrsn/Devcpp/ParticleFilter/Particle-Filter/video/bumping.mp4");

    if(!video.isOpened()) return -1;
    Size videoSize = Size((int) video.get(CAP_PROP_FRAME_WIDTH), (int) video.get(CAP_PROP_FRAME_HEIGHT));

    Mat videoFrame;
    Mat refFrame; //the image include the target patch for future tracking
    Mat refFrameShow; //the image include the target patch for display

    //Size objectSize(70,70); //the size of the object to be tracked in the refFrame

    Size objectSize(60,60); //the size of the object to be tracked in the refFrame

    //Point2f roiPoint = Point2f(288,232); //The center coordinate of the object to be tracked.

    Point2f roiPoint = Point2f(356,232); //The center coordinate of the object to be tracked.


    refFrame = imread("/home/yrsn/Devcpp/ParticleFilter/Particle-Filter/video/output2.jpg", COLOR_BGR2RGB);
    refFrameShow = refFrame.clone();

    rectangle(refFrameShow,
              Point2f(roiPoint.x-objectSize.width/2,roiPoint.y-objectSize.height/2),
              Point2f(roiPoint.x+objectSize.width/2,roiPoint.y+objectSize.height/2),
              Scalar(255,0,0),4); //draw the ROI.
    video >> videoFrame;

    namedWindow("Patch", WINDOW_NORMAL);
    imshow("Patch", refFrameShow);


    int nParticles = 2000;
    Mat Resampling;


    Particle particleAlgo(refFrame, videoFrame, nParticles, objectSize, roiPoint);
    int KeyEvent;
    setMouseCallback("Patch",onMouse, NULL);

    while(!videoFrame.empty())
    {
        Resampling = videoFrame.clone();

        //To initialize the particles
        if(video.get(CAP_PROP_POS_FRAMES)==1)
        {
            particleAlgo.setInputImage(videoFrame);
            particleAlgo.generateParticlesInit();
            Resampling = particleAlgo.getParticleImage().clone();
            namedWindow("Particles",WINDOW_NORMAL);
            imshow("Particles", Resampling);
            videoFrame = particleAlgo.getTarget().clone();
        }

        //Frame by frame tracking
        if(video.get(CAP_PROP_POS_FRAMES)>1)
        {
            particleAlgo.setInputImage(videoFrame);
            particleAlgo.particleAlgo();
            videoFrame = particleAlgo.getTarget().clone();
            Resampling = particleAlgo.getParticleImage().clone();
            imshow("Particles", Resampling);
        }

        namedWindow("Tracking",WINDOW_NORMAL);
        imshow("Tracking",videoFrame);

        KeyEvent = waitKey(33);
        if(KeyEvent == 27)
        {
            break;
        }
        else if(KeyEvent == 32)
        {
            waitKey(0);
        }

        video >> videoFrame;
    }
    waitKey(0);
    return 0;

}
