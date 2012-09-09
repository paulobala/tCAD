//
//  SaveKinectOption.h
//  Carver
//
//  Created by paulobala on 09/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_SaveKinectOption_h
#define Carver_SaveKinectOption_h

#include "OnScreenOption.h"

class SaveKinectOption: public OnScreenOption
{
    
    ofTrueTypeFont font14;
    // ofImage img;
public:
     
    SaveKinectOption(ofVec2f center_, float radius_, string name_, ofVec3f * point_, ofColor color_){
        center = center_; 
        radius = radius_; 
        name = name_;
        entryPoint = point_;
        color = color_;
        ofTrueTypeFont::setGlobalDpi(72);
        font14.loadFont("fonts/helveticaNeue.ttf", 14, true, true);
        img.loadImage("images/okay.png");
    }
    
    void draw(){
        ofPushStyle();
        ofSetColor(color);
        ofCircle(center, radius);
        ofSetColor(ofColor::white);
        img.setAnchorPercent(0.5, 0.5);
        img.draw(center.x,center.y,55,55);
        ofSetColor(ofColor::white);
        font14.drawString(name, center.x -20,center.y);
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
