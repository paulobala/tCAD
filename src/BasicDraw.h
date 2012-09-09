//
//  BasicDraw.h
//  Modeller
//
//  Created by paulobala on 17/04/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Modeller_BasicDraw_h
#define Modeller_BasicDraw_h

#include "Shape3D.h"
#include "ParentDraw.h"
#include "ColorScheme.h"

class BasicDraw: public ParentDraw{
public:
    
    BasicDraw(Shape3D* shape_):ParentDraw(shape_, new ofColor(100,100,100)){
    };
    BasicDraw(Shape3D* shape_, ofColor * color_):ParentDraw(shape_, color_){
    };
    void draw(){
        ofPushStyle();
        int limitTime = 3000;
        if(ofGetElapsedTimeMillis() - shape3D->creationTime > limitTime){
            ofSetColor(*color);}
        else{
            float difr = 255-color->r;
            float difg = 255-color->g;
            float difb = 255-color->b;
            float newr = ofMap(ofGetElapsedTimeMillis() - shape3D->creationTime, 0, limitTime, 0, difr);
            float newg = ofMap(ofGetElapsedTimeMillis() - shape3D->creationTime, 0, limitTime, 0, difg);
            float newb = ofMap(ofGetElapsedTimeMillis() - shape3D->creationTime, 0, limitTime, 0, difb);
            
            ofSetColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255)));//white to color
        }
        glEnable(GL_DEPTH_TEST);
        ofEnableLighting();
        ParentDraw::draw();
        ofDisableLighting(); 
        glDisable(GL_DEPTH_TEST);
        ofPopStyle();
    }
    
    
};

#endif
