

#include <iostream>
#include "KalmanPixel.h"
// Transition matrix ‘A’ describes relationship between
// model parameters at step k and at step k+1 (this is
// the “dynamics” in our model)
//

KalmanPixel::KalmanPixel(cv::Point point) {
    
    kalman = cvCreateKalman( 2, 2, 0 );
    measurement = cvCreateMat( 2, 1, CV_32FC1 );
    cvZero( measurement );
    const float A[] = { 1,0,0,1 };
    memcpy( kalman->transition_matrix->data.fl, A, sizeof(A));
    cvSetIdentity( kalman->measurement_matrix, cvRealScalar(1) );
    cvSetIdentity( kalman->process_noise_cov, cvRealScalar(1e-8) );
    cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1e-8) );
    cvSetIdentity( kalman->error_cov_post, cvRealScalar(1e-5));
    
    kalman->state_post->data.fl[0]=point.x;
    kalman->state_post->data.fl[1]=point.y;
    
}

KalmanPixel::~KalmanPixel() {
}


cv::Point KalmanPixel::correct(cv::Point point){
    
    
    /* predict point position */
    const CvMat* prediction = cvKalmanPredict( kalman, 0 );
    //float predict = prediction->data.fl[0];
    
    
    /* generate measurement */
    measurement->data.fl[0]=point.x;
    measurement->data.fl[1]=point.y;
    
    /* adjust Kalman filter state */
    const CvMat* correction = cvKalmanCorrect( kalman, measurement );
    
    return cv::Point(correction->data.fl[0], correction->data.fl[1]);
    
}
