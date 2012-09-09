//
//  STLCircleSubOptions.h
//  Carver
//
//  Created by paulobala on 24/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_STLCircleSubOptions_h
#define Carver_STLCircleSubOptions_h

#include "OnScreenOption.h"
#include "Basic3DObjectFromSTL.h"

class STLSubOptions: public OnScreenOption
{
    
	string path;
 
public: 
  
    STLSubOptions(ofVec2f center_, float radius_, string name_, ofVec3f * point_, LockableVector<Shape3D* > * shapes_ ,string path_, ofColor color_, ofImage img_){
        center = center_; 
        radius = radius_;
        name = name_;
        path = path_;
        entryPoint = point_;
        color = color_;
        shapes = shapes_;
        img = img_;
        ofTrueTypeFont::setGlobalDpi(72);
        font.loadFont("fonts/helveticaNeue.ttf", 14, true, true);
    }

    void draw(){
        ofPushStyle();
        ofSetColor(color);
        ofCircle(center, radius);
        ofSetColor(ofColor::black);
        img.setAnchorPercent(0.5, 0.5);
        img.draw(center.x,center.y,55,55);
        ofSetColor(ofColor::white);
        font.drawString(name, center.x -20,center.y);
        ofPopStyle();
    };
    
    void action(){
        Basic3DObjectFromSTL * temp = new Basic3DObjectFromSTL(path,*entryPoint);
        temp->changeColor(new ofColor(color.r, color.g, color.b));
        shapes->lockVector();
        shapes->addElement(temp); 
        shapes->unlockVector();
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
