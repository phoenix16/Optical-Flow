#include "MotionEstimator.h"

MotionEstimator::MotionEstimator()
{
    // Parameters for Shi-Tomasi algorithm
    qualityLevel = 0.1;
    minDistance = 2;
}

// Determine if new points should be added
bool MotionEstimator::addNewPoints()
{
    // if too few points
    cout << "\tNumber of accumulated features from previous frame = " << points[0].size() << endl;
    return points[0].size() <= 10;
}

// Feature point detection
void MotionEstimator::detectFeaturePoints()
{
    cout << "Detecting Features..." << endl;
    // Detect the features
    cv::goodFeaturesToTrack(grayPrev,       // input image
                            features,       // output detected features
                            MAX_FEATURES,   // max number of features
                            qualityLevel,   // quality level
                            minDistance);   // min distance between 2 features
    cout << "\tNumber of features detected = " << features.size() << endl;
}

// Perform entire Feature Detection pipeline on given frame
void MotionEstimator::performFeatureDetection(Mat& frame)
{
    cvtColor(frame, gray, CV_BGR2GRAY);

    // For first frame of sequence
    if (grayPrev.empty())
        gray.copyTo(grayPrev);

    // If number of feature points is insufficient, detect more
    if (addNewPoints())
    {
        // Detect feature points
        detectFeaturePoints();

        // Add detected features to currently tracked features
        points[0].insert(points[0].end(), features.begin(), features.end());
    }

    // Track features using Lucas-Kanade method
    cout << "Calculating Optical Flow..." << endl;
    calcOpticalFlowPyrLK(grayPrev, gray,  // 2 consecutive frames
                         points[0],       // input point positions in prev frame
                         points[1],       // output point positions in current frame
                         status,          // tracking success
                         err);            // tracking error


    // Loop over the tracked points to reject some
    int numValid = 0;
    namedWindow("Feature Tracking");
    //moveWindow("Feature Tracking", 1000, 300);
	moveWindow("Feature Tracking", 2500, 300);

    for (size_t i = 0; i < points[1].size(); i++)  // check every feature point
    {
        if ( status[i] == 0 )
            continue; // Don't consider points rejected by optical flow
        //            (status[i] &&
        //            (abs(points[0][i].x - points[1][i].x) +
        //            (abs(points[0][i].y - points[1][i].y)) > 2));
        numValid++;

        Point p,q;
        p.x = (int) points[0][i].x;
        p.y = (int) points[0][i].y;
        q.x = (int) points[1][i].x;
        q.y = (int) points[1][i].y;

        double angle;
        angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
        double hypotenuse;
        hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );

        // Lengthen the line by a factor of 3 (else arrows are short because of barely any motion between frames)
        q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
        q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

        // Draw a circle on feature points, and line for motion vectors
        // line thickness = 1, CV_AA = AntiAliased drawing, 0 = no fractional bits in center cooridinate or radius
        cv::line(frame, p, q, cv::Scalar(255,0,0), 1, CV_AA, 0);
        cv::circle(frame, points[1][i], 3, cv::Scalar(0,0,255), -1, 8); // radius = 3

    }
    imshow("Feature Tracking", frame );
    cout << "\tNumber of features points rejected : " << points[1].size()-numValid << endl;

    // Set the current (points,image) = previous (points,image) for next iteration
    std::swap(points[1], points[0]);
    cv::swap(grayPrev, gray);
}
