

#ifndef Carver_BasicDrawWithAxis_h
#define Carver_BasicDrawWithAxis_h

#include "Shape3D.h"
#include "ParentDraw.h"

class BasicDrawWithAxis: public ParentDraw{
public:
    
    BasicDrawWithAxis(Shape3D* shape_):ParentDraw(shape_, new ofColor(200,100,100)){
    };
    BasicDrawWithAxis(Shape3D* shape_, ofColor * color_):ParentDraw(shape_, color_){
    };
    void draw(){
        ofPushStyle();   
        glEnable(GL_DEPTH_TEST);
        ofEnableLighting();
        ofSetColor(*color);
     
        ParentDraw::draw(); 
        ofDisableLighting();  
        vector<float> limits = shape3D->mesh.ofLimits();
        if(limits.size()==6){
            float offset = 1;
            ofVec3f centeraxis = ofVec3f(limits.at(0)-offset,limits.at(3)+offset,limits.at(5)+offset);
            ofVec3f redline= ofVec3f(limits.at(1),limits.at(3)+offset,limits.at(5)+offset);
            ofVec3f blueline= ofVec3f(limits.at(0)-offset,limits.at(3)+offset,limits.at(4));
            ofVec3f greenline= ofVec3f(limits.at(0)-offset,limits.at(2),limits.at(5)+offset);
            ofSetLineWidth(0.4);
            ofSetColor(COLORSCHEME_RED);
            ofLine(centeraxis,redline);
            ofSetColor(COLORSCHEME_BLUE);
            ofLine(centeraxis, blueline);
            ofSetColor(COLORSCHEME_GREEN);
            ofLine(centeraxis, greenline);
            ofSetDrawBitmapMode(OF_BITMAPMODE_MODEL_BILLBOARD);
            ofSetColor(COLORSCHEME_TEXT_BLACK);
            ofDrawBitmapString(ofToString(limits.at(1)-limits.at(0), 0),(redline+centeraxis)/2);
            ofDrawBitmapString(ofToString(limits.at(3)-limits.at(2), 0),(greenline+centeraxis)/2);
            ofDrawBitmapString(ofToString(limits.at(5)-limits.at(4), 0),(blueline+centeraxis)/2);
        }
        glDisable(GL_DEPTH_TEST);
        ofPopStyle();
    }    
    
};


#endif
