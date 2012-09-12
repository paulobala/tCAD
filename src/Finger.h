#ifndef Carver_kinectTouch_h
#define Carver_kinectTouch_h
#include "KalmanPixel.h"

#include "ExponentialMovingAverage.h"
#include "Shape3D.h"
#include "ColorScheme.h"

class Finger{
    ExponentialMovingAverage * xavg;
    ExponentialMovingAverage * yavg;
    ExponentialMovingAverage * zForSurfacePlaneAvg;
    ExponentialMovingAverage * zForObjectPlaneAvg;
    unsigned long startTime;
    unsigned long lastUpdateTime;
    ofImage touchSurface, touchAbove, touchCamera, touchObject, touchAboveObject;
    float x;
    float y; 
    float zForSurfacePlane;
    float zForObjectPlane; 
    float startX;
    float startY; 
    float fingerBaseX;
    float fingerBaseY; 
    float limbCenterX;
    float limbCenterY;
    int fingerID;
public:
    
    enum TypeFinger{
        SHAPESELECTION,
        CONTOUR, 
        SCALECONTAINERSOUTH,
        SCALECONTAINEREAST, 
        CAMERA,
        NOTYPE, 
        ONTABLEOPTION,
        PICKER, 
        CALIBRATION,
        UNUSABLE
    };
    
    enum TypeLevel{
        SURFACE,
        ABOVESURFACE,
        CEILING, 
        OBJECT,
        ABOVEOBJECT,
        NOLEVEL
    };
    
    
    int getFingerID(){return fingerID;}
    void setFingerID(int value){fingerID = value;}
    
    
    float getX(){return x;}
    void setX(float value){x = value;}
    float getY(){return y;}
    void setY(float value){y = value;}
    float getZForSurfacePlane(){return zForSurfacePlane;}
    void setZForSurfacePlane(float value){ zForSurfacePlane=value;}
    float getZForObjectPlane(){return zForObjectPlane;}
    void setZForObjectPlane(float value){ zForObjectPlane=value;}
    float getStartX(){return startX;}
    void setStartX(float value){startX = value;}
    float getStartY(){return startY;}
    void setStartY(float value){startY = value;}
    float getFingerBaseX(){return fingerBaseX;}
    void setFingerBaseX(float value){fingerBaseX =value;}
    float getFingerBaseY(){return fingerBaseY;}
    void setFingerBaseY(float value){fingerBaseY =value;}
    float getLimbCenterX(){return limbCenterX;}
    void setLimbCenterX(float value){limbCenterX =value;}
    float getLimbCenterY(){return limbCenterY;}
    void setLimbCenterY(float value){limbCenterY =value;}
  
    bool markForRemoval, markForMoved, markForAdd;
    bool hasBeenAdded, hasBeenRemoved;
    
    TypeFinger typeFinger;
    TypeLevel typeLevel;
    TypeLevel previousTypeLevel;
    
    vector<cv::Point> limbApproxCountour;
    std::vector<Shape3D* > pickedShapes3D;
    
    Finger(float x_, float y_, float zForSurfacePlane_,float zForObjectPlane_, int fingerID_, float limbCenterX_, float limbCenterY_, float fingerBaseX_, float fingerBaseY_, vector<cv::Point> limbApproxCountour_){
        startTime = ofGetElapsedTimeMillis();
        fingerID = fingerID_;
        xavg = new ExponentialMovingAverage(5);
        xavg->initializeSamples(x_);
        x = x_;
        startX =  x_;
        yavg = new ExponentialMovingAverage(5);
        yavg->initializeSamples(y_);
        y =  y_;
        startY = y_;
        zForSurfacePlaneAvg = new ExponentialMovingAverage(5);
        zForSurfacePlaneAvg->initializeSamples(zForSurfacePlane_);
        zForSurfacePlane = zForSurfacePlane_;
        zForObjectPlaneAvg = new ExponentialMovingAverage(5);
        zForObjectPlaneAvg->initializeSamples(zForObjectPlane_);
        zForObjectPlane = zForObjectPlane_;
        
        limbCenterX = limbCenterX_;
        limbCenterY =  limbCenterY_;
        fingerBaseX= fingerBaseX_;
        fingerBaseY= fingerBaseY_;
        limbApproxCountour = limbApproxCountour_;
        markForRemoval = false;
        markForMoved = false;
        markForAdd = false;
        hasBeenAdded = false;
        hasBeenRemoved = false;
        typeFinger = NOTYPE;
        typeLevel = NOLEVEL;
        previousTypeLevel = NOLEVEL;
        
        pickedShapes3D = std::vector<Shape3D*>();
        
        touchSurface.loadImage("images/fingerSurface.png");
        touchAbove.loadImage("images/fingerAboveSurface.png");
        touchCamera.loadImage("images/camera.png");
        touchObject.loadImage("images/fingerObject.png");
        touchAboveObject.loadImage("images/fingerAboveObject.png");
    }
    
    bool checkFinger(float x_, float y_, float zForSurfacePlane_,float zForObjectPlane_, float limbCenterX_, float limbCenterY_, float fingerBaseX_, float fingerBaseY_, vector<cv::Point> limbApproxCountour_){
        xavg->addSample(x_);
        yavg->addSample(y_);
        zForSurfacePlaneAvg->addSample(zForSurfacePlane_);
        zForObjectPlaneAvg->addSample(zForObjectPlane_);
        x = xavg->ExponentialAverage(); 
        y = yavg->ExponentialAverage();
        zForSurfacePlane = zForSurfacePlaneAvg->ExponentialAverage();
        zForObjectPlane = zForObjectPlaneAvg->ExponentialAverage();
        limbCenterX =limbCenterX_;
        limbCenterY = limbCenterY_;
        fingerBaseX=fingerBaseX_;
        fingerBaseY=fingerBaseY_;
        limbApproxCountour = limbApproxCountour_;
        if(hasBeenRemoved){
            hasBeenRemoved = false;
        }
        return true;
        
    }
    
    
    void updateMoved(){
        if(hasBeenAdded){
            unsigned long currenttime = ofGetElapsedTimeMillis();
            markForMoved = true;
            lastUpdateTime = currenttime;
        }
    }
    
    void update(){
        unsigned long currenttime =  ofGetElapsedTimeMillis();
        if(hasBeenAdded){
            if(hasBeenRemoved){
                if(currenttime - lastUpdateTime > 500){
                    markForRemoval = true;
                }
            }
        }
        else{
            if(currenttime - startTime > 500){
                markForAdd = true;
                hasBeenAdded = true;
                lastUpdateTime = currenttime;
            }
        } 
        
    }
    
    
    void draw(){
        string str = "ID  "+ofToString((int)fingerID);
        
        ofSetColor(ofColor::white);
        ofCircle(x*ofGetWidth(), y*ofGetHeight(), 20);
        
        if(typeFinger==PICKER){
            ofSetColor(ofColor::white);
            ofCircle(x*ofGetWidth(), y*ofGetHeight(), 15);
        }
        if(typeFinger==SHAPESELECTION){
            ofSetColor(COLORSCHEME_LIGHTGREY);
            ofCircle(x*ofGetWidth(), y*ofGetHeight(), 15);
        }
        
        
        switch(typeLevel){
            case SURFACE:{
                touchSurface.setAnchorPercent(0.5, 0.5); 
                touchSurface.draw(x*ofGetWidth(), y*ofGetHeight(),50,50);
                break;}
            case ABOVESURFACE: 
            {
                touchAbove.setAnchorPercent(0.5, 0.5); 
                touchAbove.draw(x*ofGetWidth(), y*ofGetHeight(),50,50);               
                break;}
            case CEILING:
            {
                touchCamera.setAnchorPercent(0.5, 0.5); 
                touchCamera.draw(x*ofGetWidth(), y*ofGetHeight(),50,50);
                
                break;}
            case  OBJECT:
            {
                touchObject.setAnchorPercent(0.5, 0.5); 
                touchObject.draw(x*ofGetWidth(), y*ofGetHeight(),50,50);
                
                break;}
            case ABOVEOBJECT:
            {
                touchAboveObject.setAnchorPercent(0.5, 0.5); 
                touchAboveObject.draw(x*ofGetWidth(), y*ofGetHeight(),50,50);
                
                break;}
            case NOLEVEL:
            {
                break;}
            default:
                break;
        }
    }
    
};

#endif
