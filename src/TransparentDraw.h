#ifndef tCAD_CompositeDraw_h
#define tCAD_CompositeDraw_h

#include "Shape3D.h"
#include "ParentDraw.h"


class TransparentDraw: public ParentDraw{
public:
    
    TransparentDraw(Shape3D* shape_, ofColor * color_):ParentDraw(shape_, color_){
    };
    TransparentDraw(Shape3D* shape_):ParentDraw(shape_, new ofColor(0,0,255)){
    };
    void draw(){
        ofPushStyle();
        ofEnableLighting();
        ofSetColor(*color,70);
        glEnable(GL_BLEND); //Enable alpha blending
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //Set the blend function
        ParentDraw::draw();
        glDisable(GL_BLEND);
        ofDisableLighting();  
        ofPopStyle();
        
    }
    
};


#endif
