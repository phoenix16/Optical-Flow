/*
 * Developer : Prakriti Chintalapoodi - c.prakriti@gmail.com 
*/

#ifndef MotionEstimator_H
#define MotionEstimator_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <string>
#include <iostream>

#define MAX_FEATURES 400

using namespace cv;
using namespace std;

inline static double square(int a)
{
    return a * a;
}

class MotionEstimator
{
private:
    Mat gray;  // current grayscale frame
    Mat grayPrev; // previous grayscale frame

    vector<Point2f> points[2];  // Tracked features from 0->1
    vector<Point2f> features;   // detected features
    vector <uchar> status;      // status of tracked features
    vector <float> err;         // error in tracking

    int maxCorners;         // max number of corners in goodFeaturesToTrack function
    double qualityLevel;    // Parameters for Shi-Tomasi algorithm
    double minDistance;     // Parameters for Shi-Tomasi algorithm

    bool addNewPoints();
    void detectFeaturePoints();

public:
    MotionEstimator();
    void performFeatureDetection(Mat& frame);
};

#endif // MotionEstimator_H
