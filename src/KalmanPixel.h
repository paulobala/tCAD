//
//  KalmanPixel.h
//  TeseAppExample
//
//  Created by paulobala on 18/03/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef TeseAppExample_KalmanPixel_h
#define TeseAppExample_KalmanPixel_h

#include "ofMain.h"
#include "ofxCvMain.h"
#include "ofxOpenCv.h"

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
