#ifndef tCAD_CircleOption_h
#define tCAD_CircleOption_h

class OnScreenOption{
protected:
    LockableVector<Shape3D*> * shapes;//shapes on 3D scene
    ofVec3f * entryPoint;//entry point for new shapes
    ofTrueTypeFont font;
  
public:
    float radius;//radius of circle option
    ofVec2f center;//center of circle option
    string name;
    ofColor color;
    ofImage img;//image representing option
    virtual void draw()=0;//draw option
    virtual void action() = 0;//Pressed
    virtual bool checkHit(float x, float y) = 0;//Option was pressed?
};

#endif
