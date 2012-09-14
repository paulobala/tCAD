#ifndef tCAD_KalmanPixel_h
#define tCAD_KalmanPixel_h

#include "ofMain.h"
#include "ofxCvMain.h"
#include "ofxOpenCv.h"

/*
 Kalman filter for pixel/point. Based on source code on http://forum.openframeworks.cc/index.php?topic=9218.0
 */
class KalmanPixel {
public:
    KalmanPixel(cv::Point point);
    virtual ~KalmanPixel();
    cv::Point correct(cv::Point point);
    CvKalman * kalman;
    CvMat* state;
    CvMat* process_noise;
    CvMat* measurement;
    CvRandState rng;
};

#endif
