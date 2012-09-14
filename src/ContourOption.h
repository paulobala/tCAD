#ifndef tCAD_ContourOption_h
#define tCAD_ContourOption_h

#include "OnScreenOption.h"

/*
 Option was Contour mode
 */
class ContourOption: public OnScreenOption
{
   
   
public:
    /*
     Constructor
     */
    ContourOption(ofVec2f center_, float radius_, string name_, ofVec3f * point_, ofColor color_){
        center = center_; 
        radius = radius_; 
        name = name_;
        entryPoint = point_;
        color =  color_;
        ofTrueTypeFont::setGlobalDpi(72);
        font.loadFont("fonts/helveticaNeue.ttf", 14, true, true);
        img.loadImage("images/contour.png");
    }
    /*
     Draw option
     */
    void draw(){
        ofPushStyle();
        ofSetColor(color);
        ofCircle(center, radius);
        ofSetColor(ofColor::white);
        img.setAnchorPercent(0.5, 0.5);
        img.draw(center.x,center.y-5,40,40);
        ofSetColor(ofColor::black);
        font.drawString(name, center.x - 20,center.y-10);
        ofPopStyle();
    };
    /*
     Pressed option
     */
    void action(){
        
    };
    /*
     Pressed?
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
