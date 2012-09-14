#ifndef tCAD_markertCAD_h
#define tCAD_markertCAD_h

#include "TuioObject.h"

/*
 Represents a physical object
 */
class Token{
    float x;
    float y;
    float angle;
    float xSpeed;//speed of x change
    float ySpeed;//speed of y change
    float rotationspeed;//speed of angle change  
    int tokenID;
    unsigned long startTime;
    unsigned long lastUpdateTime;
public:
    
    enum TYPETOKEN{
        CONTAINER, 
        SHREDDER1,
        SHREDDER2,
        ONTABLECC,
        ONTOKENCC,
        INAIRCC, 
        MARKERXZ,
        MARKERXY,
        MARKERYZ,
        CALIBRATION,
        ORBITER,
        SAVE,
        NOTYPE
    };//Function
    
    
    TUIO::TuioObject * tuioObject;
    
    bool markForRemoval, markForMoved, markForAdd;
    bool hasBeenAdded, hasBeenRemoved;
    
    TYPETOKEN typeobject;
    
    float getY(){return y;}
    float getX(){return x;}
    float getAngle(){return angle;}
    float getXSpeed(){return xSpeed;}
    float getYSpeed(){return ySpeed;}
    float getRotationSpeed(){return rotationspeed;}
    float getSymbolID(){return tokenID;}
    
    /*
     Constructor for uncalibrated tokens
     */
    Token( TUIO::TuioObject * tuioObject_){
        startTime = ofGetElapsedTimeMillis();
        tuioObject = tuioObject_;
        tokenID = tuioObject->getSymbolID();
        x = tuioObject->getX();
        y = tuioObject->getY();
        angle = tuioObject->getAngle();
        xSpeed = tuioObject->getXSpeed();
        ySpeed = tuioObject->getYSpeed();
        rotationspeed = tuioObject->getRotationSpeed();
        markForRemoval = false;
        markForMoved = false;
        markForAdd = false;
        hasBeenAdded= false;
        hasBeenRemoved = false;
        typeobject = NOTYPE;
    }
    /*
     Constructor for calibrated tokens
     */
    Token( TUIO::TuioObject * tuioObject_, float x_, float y_){
        startTime = ofGetElapsedTimeMillis();
        tuioObject = tuioObject_;
        tokenID = tuioObject->getSymbolID();
        x = x_;
        y = y_;
        angle = tuioObject->getAngle();
        xSpeed = tuioObject->getXSpeed();
        ySpeed = tuioObject->getYSpeed();
        rotationspeed = tuioObject->getRotationSpeed();
        markForRemoval = false;
        markForMoved = false;
        markForAdd = false;
        hasBeenAdded= false;
        hasBeenRemoved = false;
        typeobject = NOTYPE;
    }
    
    /*
     Check to see if given tuioobject is the same as the one in token; without calibration
     */
    bool checkToken( TUIO::TuioObject * tuioObject_ ){
        
        if(hasBeenRemoved)//has old object been removed from table
        {
            //is in new object in a square of 2 pixels around old object
            if(tuioObject_->getX() *ofGetWidth() > x* ofGetWidth() -2 && tuioObject_->getY() *ofGetWidth() < x*ofGetWidth() +2 ){
                if(tuioObject_->getY() *ofGetHeight() > y *ofGetHeight() -2 && tuioObject_->getY() *ofGetHeight()  < y *ofGetHeight() +2){
                    tuioObject = tuioObject_;
                    tokenID = tuioObject->getSymbolID();
                    x = tuioObject->getX();
                    y = tuioObject->getY();
                    angle = tuioObject->getAngle();
                    xSpeed = tuioObject->getXSpeed();
                    ySpeed = tuioObject->getYSpeed();
                    rotationspeed = tuioObject->getRotationSpeed();
                    hasBeenRemoved = false;
                    return true;
                }
            }
            
        } 
        else if(tokenID == tuioObject_->getSymbolID())//same ID
        {
            tuioObject = tuioObject_;
            tokenID = tuioObject->getSymbolID();
            x = tuioObject->getX(); 
            y = tuioObject->getY();
            angle = tuioObject->getAngle();
            xSpeed = tuioObject->getXSpeed();
            ySpeed = tuioObject->getYSpeed();
            rotationspeed = tuioObject->getRotationSpeed();
            return true;}
        else{
            //tuio object is not the same
            return false;}
    }
    
    /*
     Check to see if given tuioobject is the same as the one in token; with calibration
     */
    bool checkToken( TUIO::TuioObject * tuioObject_ , float x_, float y_){
        
        if(hasBeenRemoved)//has old object been removed from table
        {
            //is in new object in a square of 2 pixels around old object
            if(x_ *ofGetWidth() > x* ofGetWidth() -2 && x_ *ofGetWidth() < x*ofGetWidth() +2 ){
                if(y_ *ofGetHeight() > y *ofGetHeight() -2 && y_ *ofGetHeight()  < y *ofGetHeight() +2){
                    tuioObject = tuioObject_;
                    tokenID = tuioObject->getSymbolID();
                    x = x_;
                    y = y_;
                    angle = tuioObject->getAngle();
                    xSpeed = tuioObject->getXSpeed();
                    ySpeed = tuioObject->getYSpeed();
                    rotationspeed = tuioObject->getRotationSpeed();
                    hasBeenRemoved = false;
                    return true;
                }
            }
        } 
        else if(tokenID == tuioObject_->getSymbolID())//same ID
        {
            tuioObject = tuioObject_;
            tokenID = tuioObject->getSymbolID();
            x = x_;
            y = y_;
            angle = tuioObject->getAngle();
            xSpeed = tuioObject->getXSpeed();
            ySpeed = tuioObject->getYSpeed();
            rotationspeed = tuioObject->getRotationSpeed();
            return true;}
        else{
            //tuio object is not the same
            return false;}
    }
    
    /*
     Update movement; with calibration
     */
    void updateMoved(float x_, float y_){
        if(hasBeenAdded){
            unsigned long currenttime = ofGetElapsedTimeMillis();
            if(tuioObject != NULL){
                tokenID = tuioObject->getSymbolID();
                x = x_;
                y = y_;
                angle = tuioObject->getAngle();
                xSpeed = tuioObject->getXSpeed();
                ySpeed = tuioObject->getYSpeed();
                rotationspeed = tuioObject->getRotationSpeed();
            }
            
            markForMoved = true;
            lastUpdateTime = currenttime;
        }
    }
    
    
    /*
     Update movement
     */
    void updateMoved(){
        if(hasBeenAdded){
            unsigned long currenttime = ofGetElapsedTimeMillis();
            
            if(tuioObject != NULL){
                tokenID = tuioObject->getSymbolID();
                x = tuioObject->getX();
                y = tuioObject->getY();
                angle = tuioObject->getAngle();
                xSpeed = tuioObject->getXSpeed();
                ySpeed = tuioObject->getYSpeed();
                rotationspeed = tuioObject->getRotationSpeed();
            }
            
            markForMoved = true;
            lastUpdateTime = currenttime;
        }
    }
    
    /*
     Update timers
     */
    void update(){
        unsigned long currenttime =  ofGetElapsedTimeMillis();
        if(hasBeenAdded){
            if(hasBeenRemoved){
                if(currenttime - lastUpdateTime > 500)//token remains after 0.5 seconds after removal
                {
                    markForRemoval = true;
                }
            }
        }else
        {
            if(currenttime - startTime > 500)//token is only valid after 0.5 seconds
            {
                markForAdd = true;
                hasBeenAdded= true;
                lastUpdateTime = currenttime;
            }
        } 
    }
    
    
    
};


#endif
