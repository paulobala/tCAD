#ifndef tCAD_MinusContainer_h
#define tCAD_MinusContainer_h
#include "ofMain.h"

class WestContainer
{
    ofVec2f nw, ne, sw, se;
    ofPath path;
    unsigned long clickTime;
public:
    
    ofColor color;
    float limitTime;
    WestContainer(  ofVec2f nw_, ofVec2f ne_, ofVec2f sw_, ofVec2f se_ );
    void update( ofVec2f anw, ofVec2f ane, ofVec2f ase, ofVec2f asw);
    void draw(ofImage img_);
    void draw();
    bool inside(float x, float y);
    void action();

};

#endif
