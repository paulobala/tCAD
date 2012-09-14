
#ifndef tCAD_PinkDraw_h
#define tCAD_PinkDraw_h
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
