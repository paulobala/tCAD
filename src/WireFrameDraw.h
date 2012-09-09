//
//  WireFrame.h
//  Carver
//
//  Created by paulobala on 12/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_WireFrame_h
#define Carver_WireFrame_h


class WireFrameDraw: public ParentDraw{
public:
    
    WireFrameDraw(Shape3D* shape_):ParentDraw(shape_, new ofColor(200,100,100)){
    };
    WireFrameDraw(Shape3D* shape_, ofColor * color_):ParentDraw(shape_, color_){
    };
    void draw(){
        ofPushStyle();
        ofEnableLighting();
        ofSetColor(*color);
        glEnable(GL_DEPTH_TEST);
        shape3D->mesh.drawWireframe();
        ofDisableLighting();  
        glDisable(GL_DEPTH_TEST);
        ofPopStyle();
    }
    
    
};


#endif
