//
//  ScalePlusContainer.h
//  Carver
//
//  Created by paulobala on 02/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_ScalePlusContainer_h
#define Carver_ScalePlusContainer_h
#include "ofMain.h"

class EastContainer {
    ofVec2f nw, ne, sw, se;
    ofPath path;
    unsigned long clickTime;    
public:
    float limitTime;
    ofColor color;
 
    EastContainer( ofVec2f nw_, ofVec2f ne_, ofVec2f sw_, ofVec2f se_ );
    
    void update(ofVec2f nw_, ofVec2f ne_, ofVec2f se_, ofVec2f sw_);
    
    void draw(ofImage img_);
    void draw();
    bool inside(float x, float y);
    void action();

    
};

#endif
