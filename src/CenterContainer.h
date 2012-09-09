//
//  CenterContainer.h
//  Carver
//
//  Created by paulobala on 03/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_CenterContainer_h
#define Carver_CenterContainer_h
#include "ofMain.h"

class CenterContainer {
    ofVec2f nw, ne, sw, se;
    ofPath path;
    unsigned long clickTime;
public:
    enum OperationType{
        SCALE, TRANSLATE, ROTATE, BOOLEANS, NOOPERATION
    };
    
    ofColor color;
    
    CenterContainer( ofVec2f nw_, ofVec2f ne_, ofVec2f sw_, ofVec2f se_ );
    
    void update(ofVec2f nw_, ofVec2f ne_, ofVec2f se_, ofVec2f sw_);    
    void draw(ofImage img_);    
    void draw();
    bool inside(float x, float y);    
    void action();
    bool isWaiting();
};

#endif
