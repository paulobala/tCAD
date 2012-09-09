#ifndef Carver_markerCarver_h
#define Carver_markerCarver_h

#include "TuioObject.h"

class Token{
    float x;
    float y;
    float angle;
    float xSpeed;
    float ySpeed;
    float rotationspeed;  
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
    };
 
    
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
    
    
    bool checkToken( TUIO::TuioObject * tuioObject_ ){
        
        if(hasBeenRemoved){
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
        else if(tokenID == tuioObject_->getSymbolID()){
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
            return false;}
    }
    
    
    bool checkToken( TUIO::TuioObject * tuioObject_ , float x_, float y_){
        
        if(hasBeenRemoved){
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
        else if(tokenID == tuioObject_->getSymbolID()){
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
            return false;}
    }
    
    
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
    void update(){
        unsigned long currenttime =  ofGetElapsedTimeMillis();
        if(hasBeenAdded){
            if(hasBeenRemoved){
                if(currenttime - lastUpdateTime > 500){
                    markForRemoval = true;
                }
            }
        }else
        {
        if(currenttime - startTime > 500){
                markForAdd = true;
                hasBeenAdded= true;
                lastUpdateTime = currenttime;
            }} 
        
    }
    
    
    
};


#endif
