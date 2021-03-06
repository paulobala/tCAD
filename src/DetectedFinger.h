#ifndef tCAD_Finger_h
#define tCAD_Finger_h
#include "Limb.h"

/*
 Represents detected finger in the finger detection process
 */
class DetectedFinger{
    cv::Point2i extremity;//fingertip
    Limb * limb;
    cv::Point2i fingerBase;
    float fingerDepthForSurfacePLane;
    float fingerDepthForObjectPlane;
public:
        
    DetectedFinger(){
        limb = new Limb();
        extremity =  cv::Point2i ();
        fingerDepthForSurfacePLane = 0;
        fingerDepthForObjectPlane = 0;
        fingerBase = cv::Point2i ();
    }
    
    cv::Point2i getExtremity(){return extremity;}
    void setExtremeity(cv::Point2i value){ extremity = value;}
    Limb * getLimb(){return limb;}
    void setLimb(Limb * value){ limb =value;}
    cv::Point2i getFingerBase(){return fingerBase;}
    void setFingerBase( cv::Point2i value){fingerBase=value;}
    float getFingerDepthForSurfacePLane(){return fingerDepthForSurfacePLane;}
    void setFingerDepthForSurfacePLane(float value){fingerDepthForSurfacePLane =value;}
    float getFingerDepthForObjectPlane(){return fingerDepthForObjectPlane;}
    void setFingerDepthForObjectPlane(float value){fingerDepthForObjectPlane =value;}
    
};

#endif
