//
//  PinkDraw.h
//  Modeller
//
//  Created by paulobala on 17/04/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Modeller_PinkDraw_h
#define Modeller_PinkDraw_h
#include "IDrawable.h"

class ParentDraw : public IDrawable{
public:
    ParentDraw(Shape3D* shape_, ofColor * color_){
        color = color_;
        shape3D = shape_;
    };
    
    void draw(){
        shape3D->mesh.draw(); 
    }
    
    void changeColor(ofColor * color_){
        color = color_;
    }
};

#endif
