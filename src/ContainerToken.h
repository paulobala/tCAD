//
//  Container.h
//  Carver
//
//  Created by paulobala on 20/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_Container_h
#define Carver_Container_h
#include "ofMeshtCAD.h"
#include "AxisPlane.h"
#include "lineIntersection.h"
#include "Token.h"
#include "Observer.h"
#include "OperationContainer.h"
#include "OperationContainerAlternative.h"
#include "DashedLine.h"

class ContainerToken:public Observer
{
    
    ofVec2f nw, ne, sw, se;
    ofVec2f southScale;
    ofVec2f eastScale;
    ofImage southScaleImg, eastScaleImg, scissorsImg;
    ofEasyFingerCam * cam;
    ofRectangle view;
    float prevDistanceX;
    float prevDistanceY;
    ofVec2f previousPointX;
    ofVec2f previousPointY;
    AxisPlane * selectedPlane;
    LockableVector<Shape3D* > * shapes;
    bool hasEastScale;
    bool hasSouthScale; 
    bool hover;
    bool onTable;
    Token * token;
    int idToken;
    ofVec2f calculate_line_point(int x1, int y1, int x2, int y2, int distance)
    {
        float vx = x2 - x1;
        float vy = y2 - y1;
        float mag = sqrt(vx*vx + vy*vy);
        vx /= mag;
        vy /= mag;
        float px = (int)((float)x1 + vx * (distance));
        float py = (int)((float)y1 + vy * (distance));
        return ofVec2f(px,py);
    }
   
public:
    std::vector<Shape3D*> links;

    int getID(){return idToken;}
    
    void setToken(Token * value){token = value;}
    
    Token * getToken(){return token;}
    
    bool getIsOnTable(){return onTable;}
    void setIsOnTable(bool value){onTable = value;}
    
    Finger * fingerOnEastScale;
    Finger * fingerOnSouthScale;
    void setHasEastScale(bool value)
    {
        hasEastScale = value;
    }

    void setHasSouthScale(bool value)
    {
        hasSouthScale = value;
    }
    
    OperationContainer * operation;
    OperationContainerAlternative * operationAlt;
    
    bool alternative;
    
    ContainerToken(Token * token_, ofRectangle view_, AxisPlane * selectedPlane_, ofEasyFingerCam * cam_, bool alternative_, LockableVector<Shape3D* > * shapes_)
    {
        links = std::vector<Shape3D*>();
        token = token_;
        view = view_;
        onTable = true;
        cam = cam_;
        idToken = token_->getSymbolID();
        selectedPlane = selectedPlane_;
        shapes = shapes_;
        southScale = ofVec2f(token->getX()*ofGetWidth() + 200, token->getY()*ofGetHeight() );
        eastScale = ofVec2f(token->getX()*ofGetWidth(), token->getY()*ofGetHeight() + 200);//should be named down
        hasEastScale = false;
        hasSouthScale = false;
        prevDistanceX = 0;
        previousPointX = ofVec2f(0,0);
        previousPointY = ofVec2f(0,0);
        prevDistanceY = 0;
        operation = new OperationContainer(&links, selectedPlane, shapes);
        operationAlt = new OperationContainerAlternative(&links, selectedPlane, shapes);
        alternative = alternative_;
        eastScaleImg.loadImage("images/leftscale.png");
        southScaleImg.loadImage("images/rightscale.png");
        scissorsImg.loadImage("images/scissors.png");
    }
    
    void addLink(Shape3D * shape_)
    {
        bool found = false;
        for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end()&& !found; it++ )
        {
            if((*it)==shape_)
            {
                found = true;
            }
        }
        if(!found)
        {
            links.push_back(shape_);
            shape_->addObserver(this);
            //change operation containers
            if(alternative)
            {
                operationAlt->changedLinks();
            }
            else
            {
                operation->changedLinks();
            }
        }
    }
    
    void removeLink(Shape3D *shape_)
    {
        for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
        {
            if((*it)==shape_)
            {
                links.erase(it);
            }
        }
        //change operation containers
        if(alternative)
        {
            operationAlt->changedLinks();
        }
        else
        {
            operation->changedLinks();
        }
    }
    
    void clearLinks()
    {
        links.clear();
        if(alternative)
        {
            operationAlt->changedLinks();
        }
        else
        {
            operation->changedLinks();
        }
    }
    
    void draw()
    {
        if(onTable)
        {
            if(token->getX()!=0 && token->getY()!=0)
            {
                ofPushStyle();
                if(hover)
                {
                    ofVec2f centroidToken = ofVec2f(token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                    Dash dashing;
                    dashing.begin();
                    ofNoFill();
                    ofCircle(centroidToken.x, centroidToken.y, 100);
                    ofCircle(centroidToken.x, centroidToken.y, 110);
                    dashing.end();
                }
                else
                {
                }
                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                {
                    ofSetColor(*(*it)->color);
                    ofVec2f centroidToken = ofVec2f(token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                    ofVec3f centroidCam = cam->worldToScreen((*it)->mesh.getCentroid(),view);
                    ofVec2f centroid = ofVec2f(centroidCam.x, centroidCam.y);
                    if(centroid.distance(centroidToken) >320)
                    {
                        ofVec2f pointA = calculate_line_point(centroidToken.x, centroidToken.y, centroid.x, centroid.y, 200);
                        ofVec2f pointB = calculate_line_point( centroid.x, centroid.y, centroidToken.x, centroidToken.y, 100);
                        ofSetLineWidth(5);
                        ofLine(centroidToken, pointA);
                        ofLine(centroid, pointB);
                        ofSetLineWidth(3);
                        DashedLine dash = DashedLine(pointA, pointB);
                        dash.draw();
                        scissorsImg.setAnchorPercent(0.5, 0.5);
                        scissorsImg.draw((pointA+pointB)/2, 30, 30);
                    }
                    else
                    {
                        ofSetLineWidth(5);
                        ofLine(centroidToken, centroid);
                    }
                }
                ofPopStyle();
                ofPushStyle();
                ofFill();
                if(alternative)
                {
                    operationAlt->draw();
                }
                else
                {
                    operation->draw();
                }
                ofPopStyle();
                ofPushStyle();
                if(links.size() >= 1 )
                {
                    if(alternative)
                    {
                        if(operationAlt->opType == OperationContainerAlternative::NOOPERATION)
                        {
                            //SCALE XYZ
                            if(hasSouthScale)
                            {
                                ofSetColor(COLORSCHEME_TEXT_WHITE);
                                ofSetLineWidth(3);
                                ofLine(fingerOnSouthScale->getX()*ofGetWidth(), token->getY()*ofGetHeight(),token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofLine(fingerOnSouthScale->getX()*ofGetWidth(), token->getY()*ofGetHeight(),fingerOnSouthScale->getX()*ofGetWidth(), fingerOnSouthScale->getY()*ofGetHeight());
                                ofCircle(fingerOnSouthScale->getX()*ofGetWidth(), fingerOnSouthScale->getY()*ofGetHeight(), 30);
                                //scaleImg.setAnchorPercent(0.5, 0.5);
                                //scaleImg(fingerOnSouthScale->getX()*ofGetWidth(), fingerOnSouthScale->getY()*ofGetHeight(),40,40);
                            }
                            else
                            {
                                ofSetColor(COLORSCHEME_TEXT_WHITE);
                                ofSetLineWidth(3);
                                ofLine(southScale.x, southScale.y,token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofCircle(southScale.x, southScale.y, 30);
                                southScaleImg.setAnchorPercent(0.5, 0.5);
                                southScaleImg.draw(southScale.x, southScale.y,40,40);
                            }
                        }
                        else if(operationAlt->opType == OperationContainerAlternative::SCALE)
                        {
                            //SCALE according to plane
                            switch(selectedPlane->axis)
                            {
                                case AxisPlane::NOAXIS:
                                    ofSetColor(COLORSCHEME_TEXT_WHITE);
                                    break;
                                case AxisPlane::X_Z:
                                {
                                    ofSetColor(COLORSCHEME_BLUE);
                                    break;
                                }
                                case AxisPlane::X_Y:
                                {
                                    ofSetColor(COLORSCHEME_GREEN);
                                    break;
                                }
                                case AxisPlane::Y_Z:
                                {
                                    ofSetColor(COLORSCHEME_GREEN);
                                    break;
                                }
                                default:
                                    break;
                            }
                            if(hasEastScale)
                            {
                                //ofSetColor(255, 0, 0);
                                ofSetLineWidth(3);
                                ofLine(token->getX()*ofGetWidth(), fingerOnEastScale->getY()*ofGetHeight(),token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofLine(token->getX()*ofGetWidth(), fingerOnEastScale->getY()*ofGetHeight(),fingerOnEastScale->getX()*ofGetWidth(), fingerOnEastScale->getY()*ofGetHeight());
                                ofCircle(fingerOnEastScale->getX()*ofGetWidth(), fingerOnEastScale->getY()*ofGetHeight(), 30);
                            }
                            else
                            {
                                //ofSetColor(0, 255, 0);
                                ofSetLineWidth(3);
                                ofLine(eastScale.x, eastScale.y,token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofCircle(eastScale.x, eastScale.y, 30);
                                eastScaleImg.setAnchorPercent(0.5, 0.5);
                                eastScaleImg.draw(eastScale.x, eastScale.y,40,40);
                            }
                            switch(selectedPlane->axis)
                            {
                                case AxisPlane::NOAXIS:
                                    ofSetColor(COLORSCHEME_TEXT_WHITE);
                                    break;
                                case AxisPlane::X_Z:
                                {
                                    ofSetColor(COLORSCHEME_RED);
                                    break;
                                }
                                case AxisPlane::X_Y:
                                {
                                    ofSetColor(COLORSCHEME_RED);
                                    break;
                                }
                                case AxisPlane::Y_Z:
                                {
                                    ofSetColor(COLORSCHEME_BLUE);
                                    break;
                                }
                                default:
                                    break;
                            }
                            if(hasSouthScale)
                            {
                                //ofSetColor(255, 0, 0);
                                ofSetLineWidth(3);
                                ofLine(fingerOnSouthScale->getX()*ofGetWidth(), token->getY()*ofGetHeight(),token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofLine(fingerOnSouthScale->getX()*ofGetWidth(), token->getY()*ofGetHeight(),fingerOnSouthScale->getX()*ofGetWidth(), fingerOnSouthScale->getY()*ofGetHeight());
                                ofCircle(fingerOnSouthScale->getX()*ofGetWidth(), fingerOnSouthScale->getY()*ofGetHeight(), 30);
                            }
                            else
                            {
                                //ofSetColor(0, 255, 0);
                                ofSetLineWidth(3);
                                ofLine(southScale.x, southScale.y,token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofCircle(southScale.x, southScale.y, 30);
                                southScaleImg.setAnchorPercent(0.5, 0.5);
                                southScaleImg.draw(southScale.x, southScale.y,40,40);
                            }
                        }
                        else if(operationAlt->opType == OperationContainerAlternative::BOOLEANS)
                        {
                            hasSouthScale = false;
                            hasEastScale = false;
                            fingerOnEastScale = NULL;
                            fingerOnSouthScale = NULL;
                            switch(operationAlt->chosen)
                            {
                                case OperationContainerAlternative::BOOLEAN_UNION:
                                {
                                    cam->begin(view);
                                    operationAlt->newComposite->chooseOperation(Composite3DObject::BOOLEAN_UNION);
                                    operationAlt->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainerAlternative::BOOLEAN_INTERSECTION:
                                {
                                    cam->begin(view);
                                    operationAlt->newComposite->chooseOperation(Composite3DObject::BOOLEAN_INTERSECTION);
                                    operationAlt->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainerAlternative::BOOLEAN_DIFFERENCE_A_MINUS_B:
                                {
                                    cam->begin(view);
                                    operationAlt->newComposite->chooseOperation(Composite3DObject::BOOLEAN_DIFFERENCE_A_MINUS_B);
                                    operationAlt->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainerAlternative::BOOLEAN_DIFFERENCE_B_MINUS_A:
                                {
                                    cam->begin(view);
                                    operationAlt->newComposite->chooseOperation(Composite3DObject::BOOLEAN_DIFFERENCE_B_MINUS_A);
                                    operationAlt->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainerAlternative::BOOLEAN_SYMMETRIC_DIFFERENCE:
                                {
                                    cam->begin(view);
                                    operationAlt->newComposite->chooseOperation(Composite3DObject::BOOLEAN_SYMMETRIC_DIFFERENCE);
                                    operationAlt->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainerAlternative::NOBOOLEAN:
                                {
                                    for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                    {
                                        if((*it)->style != Shape3D::BASIC)
                                        {
                                            (*it)->changeStyle(Shape3D::BASIC);
                                        }
                                    }
                                    break;
                                }
                                default:
                                    break;
                            }
                        }
                    }
                    else
                    {
                        if(operation->opType == OperationContainer::NOOPERATION)
                        {
                            //SCALE XYZ
                            if(hasSouthScale)
                            {
                                ofSetColor(COLORSCHEME_TEXT_WHITE);
                                ofSetLineWidth(3);
                                ofLine(fingerOnSouthScale->getX()*ofGetWidth(), token->getY()*ofGetHeight(),token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofLine(fingerOnSouthScale->getX()*ofGetWidth(), token->getY()*ofGetHeight(),fingerOnSouthScale->getX()*ofGetWidth(), fingerOnSouthScale->getY()*ofGetHeight());
                                ofCircle(fingerOnSouthScale->getX()*ofGetWidth(), fingerOnSouthScale->getY()*ofGetHeight(), 30);
                            }
                            else
                            {
                                ofSetColor(COLORSCHEME_TEXT_WHITE);
                                ofSetLineWidth(3);
                                ofLine(southScale.x, southScale.y,token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofCircle(southScale.x, southScale.y, 30);
                                southScaleImg.setAnchorPercent(0.5, 0.5);
                                southScaleImg.draw(southScale.x, southScale.y,40,40);
                            }
                        }
                        else if(operation->opType == OperationContainer::SCALE)
                        {
                            //SCALE according to plane
                            switch(selectedPlane->axis)
                            {
                                case AxisPlane::NOAXIS:
                                    ofSetColor(COLORSCHEME_TEXT_WHITE);
                                    break;
                                case AxisPlane::X_Z:
                                {
                                    ofSetColor(COLORSCHEME_BLUE);
                                    break;
                                }
                                case AxisPlane::X_Y:
                                {
                                    ofSetColor(COLORSCHEME_GREEN);
                                    break;
                                }
                                case AxisPlane::Y_Z:
                                {
                                    ofSetColor(COLORSCHEME_GREEN);
                                    break;
                                }
                                default:
                                    break;
                            }
                            if(hasEastScale)
                            {
                                //ofSetColor(255, 0, 0);
                                ofSetLineWidth(3);
                                ofLine(token->getX()*ofGetWidth(), fingerOnEastScale->getY()*ofGetHeight(),token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofLine(token->getX()*ofGetWidth(), fingerOnEastScale->getY()*ofGetHeight(),fingerOnEastScale->getX()*ofGetWidth(), fingerOnEastScale->getY()*ofGetHeight());
                                ofCircle(fingerOnEastScale->getX()*ofGetWidth(), fingerOnEastScale->getY()*ofGetHeight(), 30);
                            }
                            else
                            {
                                //ofSetColor(0, 255, 0);
                                ofSetLineWidth(3);
                                ofLine(eastScale.x, eastScale.y,token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofCircle(eastScale.x, eastScale.y, 30);
                                eastScaleImg.setAnchorPercent(0.5, 0.5);
                                eastScaleImg.draw(eastScale.x, eastScale.y,40,40);
                            }
                            switch(selectedPlane->axis)
                            {
                                case AxisPlane::NOAXIS:
                                    ofSetColor(COLORSCHEME_TEXT_WHITE);
                                    break;
                                case AxisPlane::X_Z:
                                {
                                    ofSetColor(COLORSCHEME_RED);
                                    break;
                                }
                                case AxisPlane::X_Y:
                                {
                                    ofSetColor(COLORSCHEME_RED);
                                    break;
                                }
                                case AxisPlane::Y_Z:
                                {
                                    ofSetColor(COLORSCHEME_BLUE);
                                    break;
                                }
                                default:
                                    break;
                            }
                            if(hasSouthScale)
                            {
                                //ofSetColor(255, 0, 0);
                                ofSetLineWidth(2);
                                ofLine(fingerOnSouthScale->getX()*ofGetWidth(), token->getY()*ofGetHeight(),token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofLine(fingerOnSouthScale->getX()*ofGetWidth(), token->getY()*ofGetHeight(),fingerOnSouthScale->getX()*ofGetWidth(), fingerOnSouthScale->getY()*ofGetHeight());
                                ofCircle(fingerOnSouthScale->getX()*ofGetWidth(), fingerOnSouthScale->getY()*ofGetHeight(), 30);
                            }
                            else
                            {
                                
                                ofSetLineWidth(2);
                                ofLine(southScale.x, southScale.y,token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                                ofCircle(southScale.x, southScale.y, 30);
                                southScaleImg.setAnchorPercent(0.5, 0.5);
                                southScaleImg.draw(southScale.x, southScale.y,40,40);
                            }
                        }
                        else if(operation->opType == OperationContainer::BOOLEANS)
                        {
                            hasSouthScale = false;
                            hasEastScale = false;
                            fingerOnEastScale = NULL;
                            fingerOnSouthScale = NULL;
                            switch(operation->chosen)
                            {
                                case OperationContainer::BOOLEAN_UNION:
                                {
                                    cam->begin(view);
                                    operation->newComposite->chooseOperation(Composite3DObject::BOOLEAN_UNION);
                                    operation->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainer::BOOLEAN_INTERSECTION:
                                {
                                    cam->begin(view);
                                    operation->newComposite->chooseOperation(Composite3DObject::BOOLEAN_INTERSECTION);
                                    operation->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainer::BOOLEAN_DIFFERENCE_A_MINUS_B:
                                {
                                    cam->begin(view);
                                    operation->newComposite->chooseOperation(Composite3DObject::BOOLEAN_DIFFERENCE_A_MINUS_B);
                                    operation->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainer::BOOLEAN_DIFFERENCE_B_MINUS_A:
                                {
                                    cam->begin(view);
                                    operation->newComposite->chooseOperation(Composite3DObject::BOOLEAN_DIFFERENCE_B_MINUS_A);
                                    operation->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainer::BOOLEAN_SYMMETRIC_DIFFERENCE:
                                {
                                    cam->begin(view);
                                    operation->newComposite->chooseOperation(Composite3DObject::BOOLEAN_SYMMETRIC_DIFFERENCE);
                                    operation->newComposite->draw();
                                    cam->end();
                                    break;
                                }
                                case OperationContainer::NOBOOLEAN:
                                {
                                    for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                    {
                                        if((*it)->style != Shape3D::BASIC)
                                        {
                                            (*it)->changeStyle(Shape3D::BASIC);
                                        }
                                    }
                                    break;
                                }
                                default:
                                    break;
                            }
                        }
                    }
                }
                ofPopStyle();
                if(links.size() == 2)
                {
                    //boolLock->draw();
                }
            }
        }
    }
    
    bool checkSouthScale(float x, float y)
    {
        if(onTable)
        {
            if(token->getX()!=0 && token->getY()!=0)
            {
                if(links.size() >= 1 )
                {
                    if(alternative)
                    {
                        if(operationAlt->opType == OperationContainerAlternative::NOOPERATION)
                        {
                            if(ofVec2f(x,y).distance(ofVec2f(southScale.x, southScale.y)) < 30)
                            {
                                return true;
                            }
                        }
                        else if(operationAlt->opType == OperationContainerAlternative::SCALE)
                        {
                            if(ofVec2f(x,y).distance(ofVec2f(southScale.x, southScale.y)) < 30)
                            {
                                return true;
                            }
                        }
                    }
                    else
                    {
                        if(operation->opType == OperationContainer::NOOPERATION)
                        {
                            if(ofVec2f(x,y).distance(ofVec2f(southScale.x, southScale.y)) < 30)
                            {
                                return true;
                            }
                        }
                        else if(operation->opType == OperationContainer::SCALE)
                        {
                            if(ofVec2f(x,y).distance(ofVec2f(southScale.x, southScale.y)) < 30)
                            {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }
    
    bool checkEastScale(float x, float y)
    {
        if(onTable)
        {
            if(token->getX()!=0 && token->getY()!=0)
            {
                if(links.size() >= 1 )
                {
                    if(alternative)
                    {
                        if(operationAlt->opType == OperationContainerAlternative::SCALE)
                        {
                            if( ofVec2f(x,y).distance(ofVec2f(eastScale.x, eastScale.y)) < 30)
                            {
                                return true;
                            }
                        }
                    }
                    else
                    {
                        if(operation->opType == OperationContainer::SCALE)
                        {
                            if( ofVec2f(x,y).distance(ofVec2f(eastScale.x, eastScale.y)) < 30)
                            {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }
    
    void checkLinkCut(int x, int y, int xprev, int yprev)
    {
        if(onTable)
        {
            if(token->getX()!=0 && token->getY()!=0)
            {
                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                {
                    ofVec2f centroidToken = ofVec2f(token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
                    ofVec3f centroidCam = cam->worldToScreen((*it)->mesh.getCentroid(),view);
                    ofVec2f centroid = ofVec2f(centroidCam.x, centroidCam.y);
                    if(centroid.distance(centroidToken) >320)
                    {
                        ofVec2f pointA = calculate_line_point(centroidToken.x, centroidToken.y, centroid.x, centroid.y, 200);
                        ofVec2f pointB = calculate_line_point( centroid.x, centroid.y, centroidToken.x, centroidToken.y, 100);
                        LineSegment mainline = LineSegment(pointA,pointB);
                        LineSegment touchline = LineSegment(ofVec2f(x,y), ofVec2f(xprev,yprev));
                        ofVec2f intersection;
                        if( mainline.Intersect(touchline, intersection) == LineSegment::INTERESECTING)
                        {
                            //links.erase(it);
                            removeLink((*it));
                        }
                    }
                    else
                    {
                    }
                }
            }
        }
    }
    
    ofVec3f getCentroid()
    {
        if(links.size()==0)
        {
            return ofVec3f(0,0,0);
        }
        else if(links.size()==1)
        {
            return links[0]->getCentroid();
        }
        else
        {
            ofVec3f avg;
            for (int i = 0; i < links.size(); i++)
            {
                avg= avg +links[i]->mesh.getCentroid();
            }
            avg = avg/links.size();
            return avg;
        }
    }
    
    bool inside(float  x, float y, float z, bool excludeCenter)
    {
        if(alternative)
        {
            if(excludeCenter)
            {
                if(operationAlt->insidedepth(x, y,z))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                if(operationAlt->inside(x, y,z))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        else
        {
            if(excludeCenter)
            {
                if(operation->insideDepth(x, y,z))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                if(operation->inside(x, y,z))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
    }
    
    
    void updateScale()
    {
        southScale = ofVec2f(token->getX()*ofGetWidth() + 200, token->getY()*ofGetHeight() );
        eastScale = ofVec2f(token->getX()*ofGetWidth() , token->getY()*ofGetHeight() + 200 );
        if(onTable)
        {
            if(token->getX()!=0 && token->getY()!=0)
            {
                if(links.size()>0)
                {
                    if (hasSouthScale)
                    {
                        if(alternative)
                        {
                            if(previousPointX == ofVec2f(0,0))
                            {
                                previousPointX = ofVec2f(fingerOnSouthScale->getX()*ofGetWidth(),fingerOnSouthScale->getY()*ofGetHeight());
                                prevDistanceX = abs(previousPointX.x-token->getX()*ofGetWidth());
                            }
                            else
                            {
                                ofVec2f pointa = ofVec2f(fingerOnSouthScale->getX()*ofGetWidth(),fingerOnSouthScale->getY()*ofGetHeight());
                                float newDistanceX = abs(pointa.x-token->getX()*ofGetWidth());
                                float newDistanceScaleVelocity = - (prevDistanceX - newDistanceX)/(float)prevDistanceX;
                                if(operationAlt->opType == OperationContainerAlternative::NOOPERATION)
                                {
                                    for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                    {
                                        (*it)->changeScaleXYZ(Shape3D::INCR, newDistanceScaleVelocity*10);
                                    }
                                }
                                else
                                {
                                    switch(selectedPlane->axis)
                                    {
                                        case AxisPlane::NOAXIS:
                                            break;
                                        case AxisPlane::X_Z:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::X, Shape3D::INCR, newDistanceScaleVelocity);
                                            }
                                            break;
                                        }
                                        case AxisPlane::X_Y:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::X, Shape3D::INCR, newDistanceScaleVelocity );
                                            }
                                            break;
                                        }
                                        case AxisPlane::Y_Z:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::Z, Shape3D::INCR,newDistanceScaleVelocity );
                                            }
                                            break;
                                        }
                                        default:
                                            break;
                                    }
                                }
                                previousPointX = pointa;
                                prevDistanceX = newDistanceX;
                            }
                        }
                        else
                        {
                            if(previousPointX == ofVec2f(0,0))
                            {
                                previousPointX = ofVec2f(fingerOnSouthScale->getX()*ofGetWidth(),fingerOnSouthScale->getY()*ofGetHeight());
                                prevDistanceX = abs(previousPointX.x-token->getX()*ofGetWidth());
                            }
                            else
                            {
                                ofVec2f pointa = ofVec2f(fingerOnSouthScale->getX()*ofGetWidth(),fingerOnSouthScale->getY()*ofGetHeight());
                                float newDistanceX = abs(pointa.x-token->getX()*ofGetWidth());
                                float newDistanceScaleVelocity = - (prevDistanceX - newDistanceX)/(float)prevDistanceX;
                                if(operation->opType == OperationContainer::NOOPERATION)
                                {
                                    for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                    {
                                        (*it)->changeScaleXYZ(Shape3D::INCR, newDistanceScaleVelocity*10);
                                    }
                                }
                                else
                                {
                                    switch(selectedPlane->axis)
                                    {
                                        case AxisPlane::NOAXIS:
                                            break;
                                        case AxisPlane::X_Z:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::X, Shape3D::INCR, newDistanceScaleVelocity);
                                            }
                                            break;
                                        }
                                        case AxisPlane::X_Y:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::X, Shape3D::INCR, newDistanceScaleVelocity );
                                            }
                                            break;
                                        }
                                        case AxisPlane::Y_Z:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::Z, Shape3D::INCR,newDistanceScaleVelocity );
                                            }
                                            break;
                                        }
                                        default:
                                            break;
                                    }
                                }
                                previousPointX = pointa;
                                prevDistanceX = newDistanceX;
                            }
                        }
                    }
                    else
                    {
                        previousPointX = ofVec2f(0,0);
                    }
                    if (hasEastScale)
                    {
                        if(alternative)
                        {
                            if(previousPointY == ofVec2f(0,0))
                            {
                                previousPointY = ofVec2f(fingerOnEastScale->getX()*ofGetWidth(),fingerOnEastScale->getY()*ofGetHeight());
                                prevDistanceY = abs(previousPointY.y-token->getY()*ofGetWidth());
                            }
                            else
                            {
                                ofVec2f pointb = ofVec2f(fingerOnEastScale->getX()*ofGetWidth(),fingerOnEastScale->getY()*ofGetHeight());
                                float newDistanceY = abs(pointb.y-token->getY()*ofGetWidth());
                                float newDistanceScaleVelocity = - (prevDistanceY - newDistanceY)/(float)prevDistanceY;
                                if(operationAlt->opType == OperationContainerAlternative::SCALE)
                                {
                                    switch(selectedPlane->axis)
                                    {
                                        case AxisPlane::NOAXIS:
                                            break;
                                        case AxisPlane::X_Z:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::Z, Shape3D::INCR, newDistanceScaleVelocity);
                                            }
                                            break;
                                        }
                                        case AxisPlane::X_Y:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::Y, Shape3D::INCR, newDistanceScaleVelocity );
                                            }
                                            break;
                                        }
                                        case AxisPlane::Y_Z:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::Y, Shape3D::INCR,newDistanceScaleVelocity );
                                            }
                                            break;
                                        }
                                        default:
                                            break;
                                    }
                                }
                                previousPointY = pointb;
                                prevDistanceY = newDistanceY;
                            }
                        }
                        else
                        {
                            if(previousPointY == ofVec2f(0,0))
                            {
                                previousPointY = ofVec2f(fingerOnEastScale->getX()*ofGetWidth(),fingerOnEastScale->getY()*ofGetHeight());
                                prevDistanceY = abs(previousPointY.y-token->getY()*ofGetWidth());
                            }
                            else
                            {
                                ofVec2f pointb = ofVec2f(fingerOnEastScale->getX()*ofGetWidth(),fingerOnEastScale->getY()*ofGetHeight());
                                float newDistanceY = abs(pointb.y-token->getY()*ofGetWidth());
                                float newDistanceScaleVelocity = - (prevDistanceY - newDistanceY)/(float)prevDistanceY;
                                if(operation->opType == OperationContainer::SCALE)
                                {
                                    switch(selectedPlane->axis)
                                    {
                                        case AxisPlane::NOAXIS:
                                            break;
                                        case AxisPlane::X_Z:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::Z, Shape3D::INCR, newDistanceScaleVelocity);
                                            }
                                            break;
                                        }
                                        case AxisPlane::X_Y:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::Y, Shape3D::INCR, newDistanceScaleVelocity );
                                            }
                                            break;
                                        }
                                        case AxisPlane::Y_Z:
                                        {
                                            for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                            {
                                                (*it)->change(Shape3D::SCALE, Shape3D::Y, Shape3D::INCR,newDistanceScaleVelocity );
                                            }
                                            break;
                                        }
                                        default:
                                            break;
                                    }
                                }
                                previousPointY = pointb;
                                prevDistanceY = newDistanceY;
                            }
                        }
                    }
                    else
                    {
                        previousPointY = ofVec2f(0,0);
                    }
                }
            }
        }
    }
    
    
    void click(float x, float y, float z)
    {
        if(alternative)
        {
            if( operationAlt->inside(x, y,z))
            {
                operationAlt->action(x, y,z);
            }
            else
            {
                cout << "nothing" << endl;
                //nothing
            }
        }
        else
        {
            if( operation->inside(x, y,z))
            {
                operation->action(x, y,z);
            }
            else
            {
                cout << "nothing" << endl;
                //nothing
            }
        }
    }
    
    void update()
    {
        if(left == NULL)
        {
            hasEastScale = false;
        }
        if(right == NULL)
        {
            hasSouthScale = false;
        }
        if(alternative)
        {
            operationAlt->update(token->getX(), token->getY(),0);
            //operationAlt->update(token->getX(), token->getY(), -token->getAngle());
        }
        else
        {
            operation->update(token->getX(), token->getY(),0);
        }
        // operation->update(point1r, point4l, point3l, point2r);
        if(onTable)
        {
            if(token->getX()!=0 && token->getY()!=0)
            {
                if(links.size() == 2)
                {
                    // boolLock->center = ofVec2f(token->getX()*ofGetWidth(), token->getY()*ofGetHeight() + 100);
                }
                if(links.size()>0)
                {
                    switch(selectedPlane->axis)
                    {
                        case AxisPlane::NOAXIS:
                            break;
                        case AxisPlane::X_Z:
                        {
                            if( token->getXSpeed() >0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::X, Shape3D::INCR,token->getXSpeed()*50/10);
                                }
                            }
                            else if( token->getXSpeed() < -0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::X, Shape3D::DECR,-token->getXSpeed()*50/10);
                                }
                            }
                            if( token->getYSpeed() >0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::Z, Shape3D::INCR,-token->getYSpeed()*50/10);
                                }
                            }
                            else if( token->getYSpeed() < -0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::Z, Shape3D::DECR,token->getYSpeed()*50/10);
                                }
                            }
                            if( token->getRotationSpeed()>0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::ROTATE, Shape3D::Y, Shape3D::INCR, token->getRotationSpeed()*50/10);
                                }
                            }
                            else if( token->getRotationSpeed()< -0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::ROTATE, Shape3D::Y, Shape3D::DECR,-token->getRotationSpeed()*50/10);
                                }
                            }
                            break;
                        }
                        case AxisPlane::X_Y:
                        {
                            if( token->getXSpeed() >0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    ( *it)-> change(Shape3D::TRANSLATE, Shape3D::X, Shape3D::INCR, token->getXSpeed()*50/10);
                                }
                            }
                            else if( token->getXSpeed() < -0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::X, Shape3D::DECR,-token->getXSpeed()*50/10);
                                }
                            }
                            if( token->getYSpeed() < - 0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::Y, Shape3D::INCR, token->getYSpeed()*50/10);
                                }
                            }
                            else if( token->getYSpeed() > 0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::Y, Shape3D::DECR, -token->getYSpeed()*50/10);
                                }
                            }
                            if( token->getRotationSpeed()>0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::ROTATE, Shape3D::Z, Shape3D::INCR,token->getRotationSpeed()*50/10);
                                }
                            }
                            else if( token->getRotationSpeed()< -0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::ROTATE, Shape3D::Z, Shape3D::DECR,-token->getRotationSpeed()*50/10);
                                }
                            }
                            break;
                        }
                        case AxisPlane::Y_Z:
                        {
                            if( token->getXSpeed() >0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::Z, Shape3D::INCR,token->getXSpeed()*50/10);
                                }
                            }
                            else if( token->getXSpeed() < -0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::Z, Shape3D::DECR,-token->getXSpeed()*50/10);
                                }
                            }
                            if( token->getYSpeed() < -0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::Y, Shape3D::INCR, token->getYSpeed()*50/10);
                                }
                            }
                            else if( token->getYSpeed() > 0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::TRANSLATE, Shape3D::Y, Shape3D::DECR, -token->getYSpeed()*50/10);
                                }
                            }
                            if( token->getRotationSpeed()>0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::ROTATE, Shape3D::X, Shape3D::INCR,-token->getRotationSpeed()*50/10);
                                }
                            }
                            else if( token->getRotationSpeed()< -0.05)
                            {
                                for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
                                {
                                    (*it)->change(Shape3D::ROTATE, Shape3D::X, Shape3D::DECR,token->getRotationSpeed()*50/10);
                                }
                            }
                            break;
                        }
                        default:
                            break;
                    }
                }
            }
        }
    }
    
    
    void Notify(Subject* subject_, ObserverMessage* message_)
    {
        switch(message_->message)
        {
            case ObserverMessage::UPDATED:
                break;
            case ObserverMessage::REMOVED:
            {
                Shape3D * d1 = dynamic_cast<Shape3D*>(subject_);
                if((bool)d1)
                {
                    removeLink(d1);
                }
                break;
            }
            default:
                break;
        }
    }
    
    
    void hovering()
    {
        hover = true;
        for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
        {
            if((*it)->style != Shape3D::BASICWITHAXIS)
            {
                (*it)->changeStyle(Shape3D::BASICWITHAXIS);
            }
        }
    }
    
    void noHovering()
    {
        hover = false;
        for (std::vector<Shape3D*>::iterator it  = links.begin()  ; it < links.end(); it++ )
        {
            if((*it)->style != Shape3D::BASIC)
            {
                (*it)->changeStyle(Shape3D::BASIC);
            }
        }
    }
    
};



#endif
