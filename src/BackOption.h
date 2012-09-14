#ifndef tCAD_BackOption_h
#define tCAD_BackOption_h
#include "OnScreenOption.h"

/*
 Option to go back to main menu
 */
class BackOption: public OnScreenOption
{
    ofTrueTypeFont font14;
public:
    /*
     Constructor
     */
    BackOption(ofVec2f center_, float radius_, string name_, ofVec3f * point_, ofColor color_){
        center = center_; 
        radius = radius_; 
        name = name_;
        entryPoint = point_;
        color =  color_;
        ofTrueTypeFont::setGlobalDpi(72);
        font14.loadFont("fonts/helveticaNeue.ttf", 14, true, true);
        
    }
    /*
     Draw option
     */
    void draw(){
        ofPushStyle();
        ofSetColor(color);
        ofCircle(center, radius);
        ofSetColor(ofColor::white);
        font14.drawString(name, center.x - 10,center.y);
        ofPopStyle();
    };
    /*
     Pressed
     */
    void action(){
        
    };
    /*
     Option pressed?
     */
    bool checkHit(float x, float y){
        if( ofVec2f(x,y).distance(center) < radius){
            action();
            return true;
        };
        return false;
    }
};
#endif
