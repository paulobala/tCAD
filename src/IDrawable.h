#ifndef Modeller_IDrawable_h
#define Modeller_IDrawable_h
#include "Shape3D.h"

class IDrawable{
protected:
    Shape3D * shape3D;
    ofColor * color;
public:
    virtual void draw() = 0;
    virtual void changeColor(ofColor * color_)=0;    
};


#endif
