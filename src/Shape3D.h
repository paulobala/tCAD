
#ifndef Modeller__Shape3D_h
#define Modeller__Shape3D_h
#include "ofxSTLFacet.h"

#include "ofMeshtCAD.h"
#include "ShapeVariables.h"
#include "Subject.h"

class Shape3D: public Subject{
protected:
    ShapeVariables shapeVariables;
public:
    
    enum STYLE{
    BASIC, TRANSPARENT, WIREFRAME, BASICWITHAXIS
    };
    enum INCRTYPE{
    INCR, DECR
    };
    enum AXIS{
    X,Y,Z
    };
    enum MANIPULATIONTYPE{
    TRANSLATE,SCALE,ROTATE
    };
    
    ofColor * color;
    STYLE style;
    ofMeshtCAD mesh;
    
    unsigned long creationTime;
    
    
    virtual void add(Shape3D *sp) = 0;
    virtual void draw() = 0;
    virtual void drawChildren()= 0;
    
    virtual void getFacets(vector<ofxSTLFacet>* facets)= 0;
    
    virtual void scale(float scalex, float scaley, float scalez) = 0;
    virtual void scale(float scalexyz) = 0;
    virtual void translate(float coordx, float coordy, float coordz) = 0 ;
    virtual void rotate (float anglex, float angley,float anglez) = 0;
    
    virtual void changeStyle(STYLE st) = 0;
    virtual void changeColor(float r, float g, float b) = 0;
    virtual void changeColor(ofColor * color_) = 0;

    virtual ofVec3f getCentroid() = 0;
    
    virtual void update() = 0;
 
    virtual void changeScaleXYZ(INCRTYPE it, float qt) = 0;
    virtual void change(MANIPULATIONTYPE ct, AXIS a, INCRTYPE it, float qt) = 0;
    
    virtual void deleteShape() = 0;
};


#endif
