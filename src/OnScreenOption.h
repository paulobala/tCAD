//
//  CircleOption.h
//  Carver
//
//  Created by paulobala on 24/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_CircleOption_h
#define Carver_CircleOption_h

class OnScreenOption{
protected:
    LockableVector<Shape3D*> * shapes;
    ofVec3f * entryPoint;
    ofTrueTypeFont font;
  
public:
    float radius;
    ofVec2f center;
    string name;
    ofColor color;
    ofImage img;
    virtual void draw()=0;
    virtual void action() = 0;
    virtual bool checkHit(float x, float y) = 0;
};

#endif
