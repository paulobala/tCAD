#ifndef tCAD__DObject_h
#define tCAD__DObject_h
#include "Shape3D.h"
#include "DrawStyles.h"
#include "ofEasyFingerCam.h"
/*
 Base class for Basic3DObject and Composite3DObject.
 */
class Base3DObject : public Shape3D
{
public:
    IDrawable * drawStyle;//style of draw
    
    Base3DObject(ofColor * color_):Shape3D()
    {
        color = color_;
        drawStyle = new BasicDraw(this,color);
        style = Shape3D::BASIC;
        shapeVariables = ShapeVariables();
        creationTime = ofGetElapsedTimeMillis();
    }
    ~Base3DObject() {};
    
    void add(Shape3D * shape_) {};
    void draw()
    {
        drawStyle->draw();
    };
    void drawChildren() {};
    
    void getFacets(vector<ofxSTLFacet>* facets) {};
    void scale(float scalex, float scaley, float scalez) {};
    void translate(float coordx, float coordy, float coordz) {};
    void rotate (float anglex, float angley,float anglez) {};
    /*
     Change draw style
     */
    void changeStyle(STYLE st)
    {
        switch (st)
        {
            case Shape3D::TRANSPARENT:
                drawStyle = new TransparentDraw(this,color);
                style = Shape3D::TRANSPARENT;
                break;
            case Shape3D::BASIC:
                drawStyle = new BasicDraw(this,color);
                style = Shape3D::BASIC;
                break;
            case Shape3D::BASICWITHAXIS:
                drawStyle = new BasicDrawWithAxis(this,color);
                style = Shape3D::BASICWITHAXIS;
                break;
            case Shape3D::WIREFRAME:
                drawStyle = new WireFrameDraw(this,color);
                style = Shape3D::WIREFRAME;
                break;
            default:
                break;
        }
    }
    
    void update()
    {
    };
    /*
     Change color of mesh
     */
    void changeColor(float r, float g, float b)
    {
        if(r!=color->r && g != color->g && b != color->b)
        {
            color->set(r,g,b);
            switch (style)
            {
                case Shape3D::BASIC:
                case Shape3D::TRANSPARENT:
                case Shape3D::WIREFRAME:
                case Shape3D::BASICWITHAXIS:
                    drawStyle->changeColor(color);
                    break;
                default:
                    break;
            }
        }
    }
    /*
     Change color of mesh
     */
    void changeColor(ofColor * color_)
    {
        if(color_->r!=color->r && color_->g != color->g && color_->b != color->b)
        {
            color=color_;
            switch (style)
            {
                case Shape3D::BASIC:
                case Shape3D::TRANSPARENT:
                case Shape3D::WIREFRAME:
                case Shape3D::BASICWITHAXIS:
                    drawStyle->changeColor(color);
                    break;
                default:
                    break;
            }
        }
    }
    /*
     get center of mesh
     */
    ofVec3f getCentroid()
    {
        return mesh.getCentroid();
    }
    /*
     change scale
     */
    void changeScaleXYZ(INCRTYPE it, float qt)
    {
        float qttemp = qt;
        mesh.scale(1/shapeVariables.scaleX, 1/shapeVariables.scaleY,1/shapeVariables.scaleZ);
        if(shapeVariables.scaleX + qttemp > 0.1 && shapeVariables.scaleX + qttemp < 10)
        {
            shapeVariables.scaleX = shapeVariables.scaleX + qttemp;
        }
        if(shapeVariables.scaleY + qttemp >0.1&& shapeVariables.scaleY + qttemp < 10)
        {
            shapeVariables.scaleY = shapeVariables.scaleY + qttemp;
        }
        if(shapeVariables.scaleZ + qttemp >0.1&& shapeVariables.scaleZ + qttemp < 10)
        {
            shapeVariables.scaleZ = shapeVariables.scaleZ + qttemp;
        }
        shapeVariables.scaleXYZ = 1;
        mesh.scale(shapeVariables.scaleX ,shapeVariables.scaleY ,shapeVariables.scaleZ);
    }
    /*
     change scale, translation and rotation
     */
    void change(MANIPULATIONTYPE ct, AXIS a, INCRTYPE it, float qt)
    {
        switch (ct)
        {
            case Shape3D::SCALE:
            {
                switch(a)
                {
                    case Shape3D::X:
                    {
                        mesh.scale(1/shapeVariables.scaleX, 1/shapeVariables.scaleY,1/shapeVariables.scaleZ);
                        if(it == Shape3D::INCR)
                        {
                            float qttemp = qt;
                            if(shapeVariables.scaleX + qttemp > 0.1 && shapeVariables.scaleX + qttemp < 10)
                            {
                                shapeVariables.scaleX = shapeVariables.scaleX + qttemp;
                            }
                            shapeVariables.scaleXYZ = 1;
                        }
                        else
                        {
                            float qttemp = qt;
                            if(shapeVariables.scaleX - qttemp > 0.1 && shapeVariables.scaleX - qttemp < 10)
                            {
                                shapeVariables.scaleX = shapeVariables.scaleX - qttemp;
                            }
                            shapeVariables.scaleXYZ = 1;
                        }
                        mesh.scale(shapeVariables.scaleX, shapeVariables.scaleY,shapeVariables.scaleZ);
                        break;
                    }
                        break;
                    case Shape3D::Y:
                    {
                        mesh.scale(1/shapeVariables.scaleX, 1/shapeVariables.scaleY,1/shapeVariables.scaleZ);
                        if(it == Shape3D::INCR)
                        {
                            float qttemp = qt;
                            if(shapeVariables.scaleY + qttemp > 0.1 && shapeVariables.scaleY + qttemp < 10)
                            {
                                shapeVariables.scaleY = shapeVariables.scaleY+ qttemp;
                            }
                            shapeVariables.scaleXYZ = 1;
                        }
                        else
                        {
                            float qttemp = qt;
                            if(shapeVariables.scaleY - qttemp > 0.1 && shapeVariables.scaleY - qttemp < 10)
                            {
                                shapeVariables.scaleY = shapeVariables.scaleY - qttemp;
                            }
                            shapeVariables.scaleXYZ = 1;
                        }
                        mesh.scale(shapeVariables.scaleX, shapeVariables.scaleY,shapeVariables.scaleZ);
                        break;
                    }
                    case Shape3D::Z:
                    {
                        mesh.scale(1/shapeVariables.scaleX, 1/shapeVariables.scaleY,1/shapeVariables.scaleZ);
                        if(it == Shape3D::INCR)
                        {
                            float qttemp = qt;
                            if(shapeVariables.scaleZ + qttemp > 0.1 && shapeVariables.scaleZ + qttemp < 10)
                            {
                                shapeVariables.scaleZ = shapeVariables.scaleZ+ qttemp;
                            }
                            shapeVariables.scaleXYZ = 1;
                        }
                        else
                        {
                            float qttemp = qt;
                            if(shapeVariables.scaleZ - qttemp > 0.1 && shapeVariables.scaleZ - qttemp < 10)
                            {
                                shapeVariables.scaleZ = shapeVariables.scaleZ - qttemp;
                            }
                            shapeVariables.scaleXYZ = 1;
                        }
                        mesh.scale(shapeVariables.scaleX, shapeVariables.scaleY,shapeVariables.scaleZ);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case Shape3D::TRANSLATE:
            {
                switch(a)
                {
                    case Shape3D::X:
                    {
                        mesh.translate(-shapeVariables.translateX, -shapeVariables.translateY, -shapeVariables.translateZ);
                        if(it == Shape3D::INCR)
                        {
                            shapeVariables.translateX = shapeVariables.translateX+ qt;
                        }
                        else
                        {
                            shapeVariables.translateX = shapeVariables.translateX - qt;
                        }
                        mesh.translate(shapeVariables.translateX, shapeVariables.translateY, shapeVariables.translateZ);
                        break;
                    }
                    case Shape3D::Y:
                    {
                        mesh.translate(-shapeVariables.translateX, -shapeVariables.translateY, -shapeVariables.translateZ);
                        if(it == Shape3D::INCR)
                        {
                            shapeVariables.translateY = shapeVariables.translateY + qt;
                        }
                        else
                        {
                            shapeVariables.translateY = shapeVariables.translateY - qt;
                        }
                        mesh.translate(shapeVariables.translateX, shapeVariables.translateY, shapeVariables.translateZ);
                        break;
                    }
                    case Shape3D::Z:
                    {
                        mesh.translate(-shapeVariables.translateX, -shapeVariables.translateY, -shapeVariables.translateZ);
                        if(it == Shape3D::INCR)
                        {
                            shapeVariables.translateZ = shapeVariables.translateZ + qt;
                        }
                        else
                        {
                            shapeVariables.translateZ = shapeVariables.translateZ - qt;
                        }
                        mesh.translate(shapeVariables.translateX, shapeVariables.translateY, shapeVariables.translateZ);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case Shape3D::ROTATE:
            {
                switch(a)
                {
                    case Shape3D::X:
                    {
                        ofVec3f centroid = mesh.getCentroid();
                        ofVec3f dist = ofVec3f(0,0,0)-centroid;
                        mesh.translate(dist);
                        mesh.rotate(-shapeVariables.rotateX, ofVec3f(1,0,0));
                        if(it == Shape3D::INCR)
                        {
                            shapeVariables.rotateX = shapeVariables.rotateX+qt;
                        }
                        else
                        {
                            shapeVariables.rotateX = shapeVariables.rotateX-qt;
                        }
                        mesh.rotate(shapeVariables.rotateX, ofVec3f(1,0,0));
                        mesh.translate(-dist);
                        break;
                    }
                    case Shape3D::Y:
                    {
                        ofVec3f centroid = mesh.getCentroid();
                        ofVec3f dist = ofVec3f(0,0,0)-centroid;
                        mesh.translate(dist);
                        mesh.rotate(-shapeVariables.rotateY, ofVec3f(0,1,0));
                        if(it == Shape3D::INCR)
                        {
                            shapeVariables.rotateY = shapeVariables.rotateY+qt;
                        }
                        else
                        {
                            shapeVariables.rotateY = shapeVariables.rotateY-qt;
                        }
                        mesh.rotate(shapeVariables.rotateY, ofVec3f(0,1,0));
                        mesh.translate(-dist);
                        break;
                    }
                    case Shape3D::Z:
                    {
                        ofVec3f centroid = mesh.getCentroid();
                        ofVec3f dist = ofVec3f(0,0,0)-centroid;
                        mesh.translate(dist);
                        mesh.rotate(-shapeVariables.rotateZ, ofVec3f(0,0,1));
                        if(it == Shape3D::INCR)
                        {
                            shapeVariables.rotateZ = shapeVariables.rotateZ+qt;
                        }
                        else
                        {
                            shapeVariables.rotateZ = shapeVariables.rotateZ-qt;
                        }
                        mesh.rotate(shapeVariables.rotateZ, ofVec3f(0,0,1));
                        mesh.translate(-dist);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            default:
                break;
        }
    }
    
    void deleteShape()
    {
        notifyObservers(new ObserverMessage(ObserverMessage::REMOVED));
    }
};

#endif
