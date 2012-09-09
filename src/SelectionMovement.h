//
//  SelectionMovementsTouch.h
//  Carver
//
//  Created by paulobala on 21/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_SelectionMovementsTouch_h
#define Carver_SelectionMovementsTouch_h
#include "Finger.h"
#include "Dash.h"

class SelectionMovement
{
    ofEasyFingerCam * cam;
    ofRectangle view;
public:
    Finger * finger;
    std::vector<Shape3D*> selectedShapes;
    
    SelectionMovement( std::vector<Shape3D*> selectedShapes_, Finger * finger_, ofEasyFingerCam * cam_,ofRectangle view_)
    {
        selectedShapes = selectedShapes_;
        finger = finger_;
        cam = cam_;
        view = view_;
    }
    
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
    
    void draw()
    {
        ofPushStyle();
        for(int i = 0; i< selectedShapes.size(); i++)
        {
            if( selectedShapes.at(i)->style != Shape3D::TRANSPARENT)
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
