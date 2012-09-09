//
//  Rotater.h
//  Carver
//
//  Created by paulobala on 23/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_Rotater_h
#define Carver_Rotater_h

#define TICKS 36
#include "ExponentialMovingAverageAngle.h"

class LineSegmentRotate{
public:
    ofVec2f start, end;
    LineSegmentRotate(ofVec2f astart, ofVec2f aend){
        start = astart; end = aend;
    }
    
}
;

class OrbitToken{
    LockableVector<ContainerToken*> * containers;
    ExponentialMovingAverageAngle * avg;
    AxisPlane * selectedPlane;
    unsigned long lastInside;
    float lastTick;
    bool clean;
    Token * token;
    float getAngle(ofVec2f center, ofVec2f p1){
        ofVec2f p0 = ofVec2f(center.x, center.y - 200);
        return 2*atan2(p1.y - p0.y, p1.x - p0.x);
        
    } 
   
public:
     void store(ofVec2f a, ofVec2f b){
        avg->addSample(getAngle(b,a));
        
    }
    
    OrbitToken(Token * token_, LockableVector<ContainerToken*> * containers_,AxisPlane * selectedPlane_){
        token = token_;
        containers = containers_;
        avg = new ExponentialMovingAverageAngle(3);
        lastInside = ofGetElapsedTimeMillis();
        lastTick = 0;
        clean = true;
        selectedPlane = selectedPlane_;
    }
    
    bool inside(float x, float y){
        if(token == NULL){return false;}
        
        ofVec2f center = ofVec2f(token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
        
        if(center.distance(ofVec2f(x,y)) < 100){
            lastInside = ofGetElapsedTimeMillis();
            
            return true;
        }
        else{
            return false;
        }
    }
 
    void update(){
        if(ofGetElapsedTimeMillis() - lastInside > 1000){
            avg->clearSamples(); 
            clean = true;
        } 
    }
    
    void action(){
        
        float angledeg = ofRadToDeg(avg->getAngle());
        
        float anglestart = 0;
        float angleend = 360/TICKS;
        float currenttick = lastTick;
        for(int i = 0; i < TICKS; i++){
            if(angledeg >= anglestart && angledeg <= angleend){
                currenttick = i;
                break;
            }
            anglestart = anglestart + 360/TICKS;
            angleend = angleend + 360/TICKS;
        }
        
        if(!clean){
            
            if(currenttick != lastTick){
                bool clockwise = true;
                if(currenttick == 0 && lastTick == TICKS-1){
                    clockwise = false;
                }
                else if(currenttick == TICKS -1 && lastTick == 0){
                    clockwise = true;
                }
                else if(currenttick > lastTick){//clockwise
                    clockwise = true;
                }
                else if(currenttick < lastTick){//counterclockwise
                    clockwise = false;
                }
                
                vector<Shape3D*> shapestemp;
                vector<ContainerToken *> conttemp = containers->getObjects();
                for(int i = 0;i < conttemp.size(); i++){
                    if(conttemp.at(i)->getIsOnTable()){
                        for(int j = 0; j < conttemp.at(i)->links.size(); j++){
                            shapestemp.push_back(conttemp.at(i)->links.at(j));
                        }
                    }
                }
                
                
                switch(selectedPlane->axis){
                    case AxisPlane::NOAXIS:
                       
                        break;
                    case AxisPlane::X_Z:{
                        for(int i = 0; i < shapestemp.size(); i++){
                            if(clockwise){
                                shapestemp.at(i)->mesh.rotate(20, ofVec3f(0,1,0));
                            }
                            else{
                                shapestemp.at(i)->mesh.rotate(-20, ofVec3f(0,1,0));
                            }
                        }
                        break;
                    }    
                    case AxisPlane::X_Y:{
                        for(int i = 0; i < shapestemp.size(); i++){
                            if(clockwise){
                                shapestemp.at(i)->mesh.rotate(20, ofVec3f(0,0,1));
                            }
                            else{
                                shapestemp.at(i)->mesh.rotate(-20, ofVec3f(0,0,1));
                            }
                        }
                        break;}    
                    case AxisPlane::Y_Z:{
                        for(int i = 0; i < shapestemp.size(); i++){
                            if(clockwise){
                                shapestemp.at(i)->mesh.rotate(20, ofVec3f(1,0,0));
                            }
                            else{
                                shapestemp.at(i)->mesh.rotate(-20, ofVec3f(1,0,0));
                            }
                        }
                        break;}    
                    default:break;
                }   
            }
        }
        else{
            
            clean = false;
        }
        
        lastTick = currenttick;
    }
    
    void draw(){
        if(!clean){
            ofVec2f point1 = ofVec2f(token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
            ofVec2f point2 = ofVec2f(token->getX()*ofGetWidth(), token->getY()*ofGetHeight()-200);
            point2.rotate(ofRadToDeg(avg->getAngle()),point1);
            ofPushStyle();
            ofSetColor(ofColor::black);
            ofSetLineWidth(20);
            ofLine(point1, point2);
            ofPopStyle();
        }
    }
    
    
};

#endif
