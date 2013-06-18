#include "MotionEstimator.h"

#define PI 3.1415926535;
#define NUM_FRAMES_TO_PROCESS  600

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        cout << "Usage: ./OpticalFlow <Input Video file>" << endl;
        return -1;
    }

    VideoCapture inputVideo(argv[1]);
    if (!inputVideo.isOpened())
    {
        cout << "Error opening Input Video " << argv[1] << endl;
        return -1;
    }
    double fps = inputVideo.get(CV_CAP_PROP_FPS);
    int delay = 1000/fps;    // time in ms between successive frames = 1000/fps
    cout << "FPS = " << fps << endl;
    cout << "Number of frames = " << static_cast<int>(inputVideo.get(CV_CAP_PROP_FRAME_COUNT)) << endl;

    // Create instance of Motion Estimator
    MotionEstimator m;

    Mat frame;
    bool stop(false);
    int frameNum = 0;

    namedWindow("Input Video");
    moveWindow("Input Video", 600, 300);

    while(!stop)
    {
        inputVideo >> frame;            // read current frame
        if( frame.empty()) break;       // check if at end
        frameNum++;
        imshow("Input Video", frame);

        // Perform Optical Flow and track the features
        m.performFeatureDetection(frame);

        if (inputVideo.get(CV_CAP_PROP_POS_FRAMES) == NUM_FRAMES_TO_PROCESS)
        {
            cout << "\nDone" << endl;
            break;
        }
        // introduce delay or press key to stop
        if (waitKey(delay) >= 0)
            stop = true;
    }

    waitKey(0);
    return 0;
}
