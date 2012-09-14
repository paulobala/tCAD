#ifndef tCAD_ScalePlusContainer_h
#define tCAD_ScalePlusContainer_h
#include "ofMain.h"

class EastContainer {
    ofVec2f nw, ne, sw, se;//edges of area
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
