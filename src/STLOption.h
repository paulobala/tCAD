//
//  STLCIRCLE.h
//  Carver
//
//  Created by paulobala on 22/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_STLCIRCLE_h
#define Carver_STLCIRCLE_h
#include "OnScreenOption.h"



class STLOption: public OnScreenOption
{
    

public:
    STLOption(ofVec2f center_, float radius_, string name_, ofVec3f * point_, LockableVector<Shape3D* > * shapes_,ofColor color_){
        center = center_; 
        radius = radius_; 
        name = name_;
        entryPoint = point_;
        color =  color_;
        shapes = shapes_;
        ofTrueTypeFont::setGlobalDpi(72);
        font.loadFont("fonts/helveticaNeue.ttf", 14, true, true);
        img.loadImage("images/box.png");
    }
        
    void draw(){
        ofPushStyle();
        ofSetColor(color);
        ofCircle(center, radius);
        ofSetColor(ofColor::white);
        img.setAnchorPercent(0.5, 0.5);
        img.draw(center.x,center.y,40,40);
        ofSetColor(ofColor::white);
        font.drawString(name, center.x -10,center.y-10);
        ofPopStyle();
    };
    
    void action(){
        
    };
    
    bool checkHit(float x, float y){
        if( ofVec2f(x,y).distance(center) < radius){
        action();
        return true;
        };
        return false;
    }
};


#endif
