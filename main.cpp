#include "MotionEstimator.h"

#define PI 3.1415926535;
#define NUM_FRAMES_TO_PROCESS  600

// Global Variables
Mat frame1, img_object;
Point pt1, pt2;
bool roi_capture = false;
bool got_roi = false;

// Function Headers
void mouse_click(int event, int x, int y, int flags, void *param);
void findObject(Mat& frame, Mat& img_object);

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        cout << "Usage: ./ObjectTracker <input video file>" << endl;
        return -1;
    }

    VideoCapture inputVideo(argv[1]);
    if (!inputVideo.isOpened())
    {
        cout << "Error opening input video " << argv[1] << endl;
        return -1;
    }
    double fps = inputVideo.get(CV_CAP_PROP_FPS);
    int delay = 1000/fps;    // time in ms between successive frames = 1000/fps
    cout << "FPS = " << fps << endl;
    cout << "Number of frames = " << static_cast<int>(inputVideo.get(CV_CAP_PROP_FRAME_COUNT)) << endl;

    inputVideo >> frame1;     // read first frame only
    if(!frame1.empty())
    {
        // Collect the ROI of the object, store it in img_object
        cout << "Click and drag to select the object to detect" << endl;
        namedWindow("Frame 1: Capture ROI", 1);
        moveWindow("Frame 1: Capture ROI", 600, 300);
        imshow("Frame 1: Capture ROI", frame1);
        setMouseCallback("Frame 1: Capture ROI", mouse_click, 0);

        waitKey(0);

        // close the window after ROI is saved
        destroyWindow("Frame 1: Capture ROI");
        destroyWindow("ROI");
    }

    // Create Trackbar
    const char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
    createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );

    MatchingMethod(0, 0);


    //    // Create instance of Motion Estimator
    //    MotionEstimator m;

    //    Mat frame;
    //    bool stop(false);
    //    int frameNum = 0;

    //    // Set position in video back to beginning
    //    inputVideo.set(CV_CAP_PROP_POS_FRAMES, 0);

    //    // Process every frame of video only when object ROI has been selected
    //    if (!img_object.empty())
    //    {
    //        namedWindow("input video");
    //        moveWindow("input video", 600, 300);

    //        while(!stop)
    //        {
    //            inputVideo >> frame;            // read current frame
    //            if( frame.empty()) break;       // check if at end
    //            frameNum++;
    //            imshow("input video", frame);
    //            cout << "\nFrame Number : " << frameNum << endl;

    findObject(frame, img_object);
    //            m.performFeatureDetection(frame);

    //            if (inputVideo.get(CV_CAP_PROP_POS_FRAMES) == NUM_FRAMES_TO_PROCESS)
    //            {
    //                cout << "\nDone" << endl;
    //                break;
    //            }
    //            // introduce delay or press key to stop
    //            if (waitKey(delay) >= 0)
    //                stop = true;
    //        }
    //    }

    waitKey(0);
    return 0;
}


//Callback for mousclick event, the x-y coordinate of mouse button-up and button-down
//are stored in two points pt1, pt2.
void mouse_click(int event, int x, int y, int flags, void *param)
{
    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
    {
        cout << "Mouse Left Button Pressed" << endl;
        if (!roi_capture)
        {
            pt1.x = x;
            pt1.y = y;
        }
        else
        {
            cout << "ROI already acquired" << endl;
        }
        break;
    }
    case CV_EVENT_LBUTTONUP:
    {
        if (!got_roi)
        {
            cout << "Mouse Left Button released" << endl;
            pt2.x = x;
            pt2.y = y;

            Mat roi(frame1, Rect(pt1, pt2));
            roi.copyTo(img_object);
            cout << "ROI acquired" << endl;
            namedWindow("ROI", 1);
            moveWindow("ROI", 1000, 300);
            imshow("ROI", roi);
            got_roi = true;
            cout << "Press any key to close ROI windows and start detecting object in video" << endl;
        }
        else
        {
            cout << "ROI already acquired" << endl;
        }
        break;
    }
    }
}


void findObject(Mat& frame, Mat& img_object)
{
    // Source image to display
    Mat img_display;
    frame.copyTo(img_display);

    // Create the result matrix
    Mat result;
    int result_cols =  frame.cols - img_object.cols + 1;
    int result_rows = frame.rows - img_object.rows + 1;
    result.create(result_cols, result_rows, CV_32FC1);

    // Do the Matching and Normalize
    // Matching method choices are:
    // CV_TM_SQDIFF, CV_TM_SQDIFF_NORMED, CV_TM_CCORR, CV_TM_CCORR_NORMED, CV_TM_CCOEFF, CV_TM_CCOEFF_NORMED
    matchTemplate( frame, img_object, result, CV_TM_SQDIFF_NORMED );
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

    // Localizing the best match with minMaxLoc
    double minVal; double maxVal;
    Point minLoc; Point maxLoc; Point matchLoc;
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
    else
    { matchLoc = maxLoc; }

    // Draw a rectangle around the found object
    rectangle(img_display, matchLoc, Point( matchLoc.x + img_object.cols , matchLoc.y + img_object.rows ), Scalar(255,0,0), 2, 8, 0 );
    rectangle(result, matchLoc, Point( matchLoc.x + img_object.cols , matchLoc.y + img_object.rows ), Scalar(255,0,255), 2, 8, 0 );

    imshow("Object Tracking", img_display);
//    imshow(result_window, result);
}
