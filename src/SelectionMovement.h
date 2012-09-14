#ifndef tCAD_SelectionMovementsTouch_h
#define tCAD_SelectionMovementsTouch_h
#include "Finger.h"
#include "Dash.h"

/*
 Represents linking of 3D shape to container token
 */
class SelectionMovement
{
    ofEasyFingerCam * cam;//camera object
    ofRectangle view;//viewport for camera
public:
    Finger * finger;//finger that initiated the selection movement
    std::vector<Shape3D*> selectedShapes;//shapes in 3D scene
    
    /*
     Constructor
     */
    SelectionMovement( std::vector<Shape3D*> selectedShapes_, Finger * finger_, ofEasyFingerCam * cam_,ofRectangle view_)
    {
        selectedShapes = selectedShapes_;
        finger = finger_;
        cam = cam_;
        view = view_;
    }
    
    /*
     Cancel Movement
     */
    void cancel()
    {
        for(int i = 0; i< selectedShapes.size(); i++)
        {
            if( selectedShapes.at(i)->style == Shape3D::TRANSPARENT)
            {
                selectedShapes.at(i)->changeStyle(Shape3D::BASIC);
            }
        }
    }
    
    /*
     Draw Movement
     */
    void draw()
    {
        ofPushStyle();
        for(int i = 0; i< selectedShapes.size(); i++)
        {
            if( selectedShapes.at(i)->style != Shape3D::TRANSPARENT)//shapes that are selected are shown as being transparent
            {
                selectedShapes.at(i)->changeStyle(Shape3D::TRANSPARENT);
            }
            ofSetColor(*selectedShapes[i]->color);
            ofSetLineWidth(4);
            ofVec3f centroid = cam->worldToScreen(selectedShapes[i]->mesh.getCentroid(),view);
            Dash dashline;
            dashline.begin();
            ofLine(centroid.x, centroid.y, finger->getX()*ofGetWidth(),finger->getY()*ofGetHeight());
            dashline.end();
            Dash dashing;
            dashing.begin();
            ofNoFill();
            ofCircle(centroid.x, centroid.y,80);
            dashing.end();
        }
        ofPopStyle();
    }
};

#endif
