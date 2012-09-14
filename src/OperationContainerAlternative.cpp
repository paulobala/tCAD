#include <iostream>
#include "OperationContainerAlternative.h"
#include "ColorScheme.h"
/*
 constructor
 */
OperationContainerAlternative::OperationContainerAlternative (std::vector<Shape3D*> * links_,  AxisPlane * selectedPlane_, LockableVector<Shape3D* > * shapes_)
{
    links = links_;
    shapes = shapes_;
    selectedPlane = selectedPlane_;
    opType = NOOPERATION;
    chosen = NOBOOLEAN;
    east = new OperationAlternativeOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    west = new OperationAlternativeOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    south = new OperationAlternativeOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    north = new OperationAlternativeOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    center = new OperationAlternativeOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    translateImg.loadImage("images/translatealternative.png");
    scaleImg.loadImage("images/scalealternative.png");
    rotateLeftImg.loadImage("images/rotatealternativeleft.png");
    rotateUpImg.loadImage("images/rotatealternativeup.png");
    minusImg.loadImage("images/minus.png");
    plusImg.loadImage("images/plus.png");
    rightscaleImg.loadImage("images/rightscale.png");
    leftscaleImg.loadImage("images/leftscale.png");
    leftArrowImg.loadImage("images/leftarrow.png");
    rightArrowImg.loadImage("images/rightarrow.png");
    unionImg.loadImage("images/Union.png");
    noBoolImg.loadImage("images/noBoolean.png");
    symDifImg.loadImage("images/symDifference.png");
    diffABImg.loadImage("images/differenceAB.png");
    diffBAImg.loadImage("images/differenceBA.png");
    intersectImg.loadImage("images/intersection.png");
}
/*
 Links changed, update Manipulation UI
 */
void OperationContainerAlternative::changedLinks()
{
    if(links->size()== 0)
    {
        opType = NOOPERATION;
    }
    else
    {
        if(opType == BOOLEANS)
        {
            opType = NOOPERATION;
        }
    }
}
/*
 Axis changed, restart Manipulation UI
 */
void OperationContainerAlternative::changeAxis()
{
    if (selectedPlane->axis == AxisPlane::NOAXIS)
    {
        opType = NOOPERATION;
    }
}
/*
 Update edges of GUI elements
 */
void OperationContainerAlternative::update(float x, float y, float angle)
{
    ofVec2f centerpoint = ofVec2f(x*ofGetWidth(), y*ofGetHeight());
    ofVec2f point1 = ofVec2f(x*ofGetWidth() +cos(angle)*70, y*ofGetHeight()+sin(angle)*70);
    point1.rotate(-45, centerpoint);
    ofVec2f point2 = ofVec2f(x*ofGetWidth()+cos(angle+PI/2)*70,
                             y*ofGetHeight()+sin(angle+PI/2)*70);
    point2.rotate(-45, centerpoint);
    ofVec2f point3 =  ofVec2f(x*ofGetWidth() +cos(angle+PI)*70, y*ofGetHeight()+sin(angle+PI)*70);
    point3.rotate(-45, centerpoint);
    ofVec2f point4 = ofVec2f(x*ofGetWidth() +cos(angle+PI+PI/2)*70, y*ofGetHeight()+sin(angle+PI+PI/2)*70);
    point4.rotate(-45, centerpoint);
    ofVec2f point1l = point4;
    ofVec2f point2l = point1;
    point1l.rotate(90, point2);
    point2l.rotate(90, point2);
    ofVec2f point3r = point4;
    ofVec2f point4r = point1;
    point3r.rotate(-90, point3);
    point4r.rotate(-90, point3);
    ofVec2f point4u = point1;
    ofVec2f point1u = point2;
    point1u.rotate(-90, point4);
    point4u.rotate(-90, point4);
    ofVec2f point2d = point1;
    ofVec2f point3d = point2;
    point2d.rotate(90, point3);
    point3d.rotate(90, point3);
    east->update(point1, point1l, point2l, point2);
    west->update(point4r, point4, point3, point3r);
    north->update(point4u, point1u, point1, point4);
    south->update(point3, point2, point2d, point3d);
    center->update(point4, point1, point2, point3);
}
/*
 Draw GUI elements depending on Manipulation Mode and plane of 3D scene
 */
void OperationContainerAlternative::draw()
{
    ofPushMatrix();
    ofPushStyle();
    switch(opType)
    {
        case NOOPERATION:
        {
            center->color = COLORSCHEME_MAGENTA;
            east->color = COLORSCHEME_GREY;
            west->color = COLORSCHEME_GREY;
            north->color = COLORSCHEME_GREY;
            south->color = COLORSCHEME_GREY;
            center->draw();
            break;
        }
        case TRANSLATE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    changeAxis();
                    break;
                case AxisPlane::X_Z:
                    center->color = COLORSCHEME_GREEN;
                    north->color = COLORSCHEME_GREEN;
                    north->shape = OperationAlternativeOption::HALFCIRCLEUP;
                    center->draw();
                    north->draw(translateImg);
                    break;
                case AxisPlane::X_Y:
                    center->color = COLORSCHEME_BLUE;
                    north->color = COLORSCHEME_BLUE;
                    north->shape = OperationAlternativeOption::HALFCIRCLEUP;
                    center->draw();
                    north->draw(translateImg);
                    break;
                case AxisPlane::Y_Z:
                    center->color = COLORSCHEME_RED;
                    north->color = COLORSCHEME_RED;
                    north->shape = OperationAlternativeOption::HALFCIRCLEUP;
                    center->draw();
                    north->draw(translateImg);
                    break;
            }
            break;
        }
        case ROTATE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    changeAxis();
                    break;
                case AxisPlane::X_Z:
                    center->color = COLORSCHEME_GREY;
                    east->color = COLORSCHEME_BLUE_TRANS;
                    north->color = COLORSCHEME_RED_TRANS;
                    center->draw();
                    east->shape = OperationAlternativeOption::HALFCIRCLELEFT;
                    east->draw(rotateLeftImg);
                    north->shape = OperationAlternativeOption::HALFCIRCLEUP;
                    north->draw(rotateUpImg);
                    break;
                case AxisPlane::X_Y:
                    center->color = COLORSCHEME_GREY;
                    east->color = COLORSCHEME_GREEN_TRANS;
                    north->color = COLORSCHEME_RED_TRANS;
                    center->draw();
                    east->shape = OperationAlternativeOption::HALFCIRCLELEFT;
                    east->draw(rotateLeftImg);
                    north->shape = OperationAlternativeOption::HALFCIRCLEUP;
                    north->draw(rotateUpImg);
                    break;
                case AxisPlane::Y_Z:
                    center->color = COLORSCHEME_GREY;
                    east->color = COLORSCHEME_BLUE_TRANS;
                    north->color = COLORSCHEME_GREEN_TRANS;
                    center->draw();
                    east->shape = OperationAlternativeOption::HALFCIRCLELEFT;
                    east->draw(rotateLeftImg);
                    north->shape = OperationAlternativeOption::HALFCIRCLEUP;
                    north->draw(rotateUpImg);
                    break;
            }
            break;
        }
        case SCALE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    changeAxis();
                    break;
                case AxisPlane::X_Z:
                    center->color = COLORSCHEME_GREEN;
                    north->color = COLORSCHEME_GREEN;
                    center->draw();
                    north->shape = OperationAlternativeOption::HALFCIRCLEUP;
                    north->draw(scaleImg);
                    break;
                case AxisPlane::X_Y:
                    center->color = COLORSCHEME_BLUE;
                    north->color = COLORSCHEME_BLUE;
                    center->draw();
                    north->shape = OperationAlternativeOption::HALFCIRCLEUP;
                    north->draw(scaleImg);
                    break;
                case AxisPlane::Y_Z:
                    center->color = COLORSCHEME_RED;
                    north->color = COLORSCHEME_RED;
                    center->draw();
                    north->shape = OperationAlternativeOption::HALFCIRCLEUP;
                    north->draw(scaleImg);
                    break;
            }
            break;
        }
        case BOOLEANS:
        {
            if(selectedPlane->axis == AxisPlane::NOAXIS)
            {
                changeAxis();
            }
            else
            {
                center->color = COLORSCHEME_GREY;
                east->color = COLORSCHEME_DARKGREY;
                west->color = COLORSCHEME_DARKGREY;
                east->shape = OperationAlternativeOption::HALFCIRCLELEFT;
                switch(chosen)
                {
                    case BOOLEAN_UNION:
                        east->draw(unionImg);;
                        break;
                    case BOOLEAN_INTERSECTION:
                        east->draw(intersectImg);
                        break;
                    case BOOLEAN_DIFFERENCE_A_MINUS_B:
                        east->draw(diffABImg);
                        break;
                    case BOOLEAN_DIFFERENCE_B_MINUS_A:
                        east->draw(diffBAImg);
                        break;
                    case BOOLEAN_SYMMETRIC_DIFFERENCE:
                        east->draw(symDifImg);
                        break;
                    case NOBOOLEAN:
                        east->draw(noBoolImg);
                        break;
                    default:
                        break;
                }
                center->draw();
            }
            break;
        }
        default:
            break;
    }
    ofPopStyle();
    ofPopMatrix();
}
/*
 Is point over area?
 */
bool OperationContainerAlternative::inside(float x, float y, float z)
{
    switch(opType)
    {
        case NOOPERATION:
        {
            if(center->inside(x, y))
            {
                return true;
            }
            break;
        }
        case TRANSLATE:
        {
            if(north->inside(x, y))
            {
                return true;
            }
            else  if(center->inside(x, y))
            {
                return true;
            }
            break;
        }
        case ROTATE:
        {
            if(east->inside(x, y))
            {
                return true;
            }
            else if(north->inside(x, y))
            {
                return true;
            }
            else  if(center->inside(x, y))
            {
                return true;
            }
            break;
        }
        case SCALE:
        {
            if(center->inside(x, y))
            {
                return true;
            }
            else if(north->inside(x, y))
            {
                return true;
            }
            break;
        }
        case BOOLEANS:
        {
            if(east->inside(x, y))
            {
                return true;
            }
            else if(west->inside(x, y))
            {
                return true;
            }
            else  if(center->inside(x, y))
            {
                return true;
            }
            break;
        }
        default:
            break;
    }
    return false;
}
/*
 Is point over area? Exclude center area
 */
bool OperationContainerAlternative::insidedepth(float x, float y, float z)
{
    switch(opType)
    {
        case NOOPERATION:
        {
            break;
        }
        case TRANSLATE:
        {
            if(center->inside(x, y))
            {
                return false;
            }
            else if(north->inside(x, y))
            {
                return true;
            }
            break;
        }
        case ROTATE:
        {
            if(east->inside(x, y))
            {
                return true;
            }
            else if(north->inside(x, y))
            {
                return true;
            }
            break;
        }
        case SCALE:
        {
            if(center->inside(x, y))
            {
                return false;
            }
            else if(north->inside(x, y))
            {
                return true;
            }
            break;
        }
        case BOOLEANS:
        {
            if(center->inside(x, y))
            {
                return false;
            }
            else if(east->inside(x, y))
            {
                return true;
            }
            else if(west->inside(x, y))
            {
                return true;
            }
            break;
        }
        default:
            break;
    }
    return false;
}
/*
 If point is inside an area, perform action deticated to that area
 */
void OperationContainerAlternative::action(float x, float y, float z)
{
    switch(opType)
    {
        case NOOPERATION:
        {
            if(center->inside(x, y))
            {
                if(!center->isWaiting())
                {
                    centerHit();
                    center->action();
                }
            }
            break;
        }
        case TRANSLATE:
        {
            if(north->inside(x, y))
            {
                north->action();
                north->updateReferenceValue(z);
                if(z-north->referenceValue > 10) //going up
                {
                    north->referenceValue = z;
                    northHit(true);
                }
                else if(north->referenceValue - z > 10)  //goingdown
                {
                    north->referenceValue = z;
                    northHit(false);
                }
                else
                {
                }
            }
            else if(center->inside(x, y))
            {
                if(!center->isWaiting())
                {
                    centerHit();
                    center->action();
                }
            }
            break;
        }
        case ROTATE:
        {
            if(east->inside(x, y))
            {
                east->action();
                east->updateReferenceValue(z);
                if(z-east->referenceValue > 10) //going up
                {
                    east->referenceValue = z;
                    eastHit(true);
                }
                else if(east->referenceValue - z > 10)  //goingdown
                {
                    east->referenceValue = z;
                    eastHit(false);
                }
                else
                {
                }
            }
            else if(north->inside(x, y))
            {
                north->updateReferenceValue(z);
                north->action();
                if(z-north->referenceValue > 10) //going up
                {
                    north->referenceValue = z;
                    northHit(true);
                }
                else if(north->referenceValue - z > 10)  //goingdown
                {
                    north->referenceValue = z;
                    northHit(false);
                }
                else
                {
                }
            }
            else  if(center->inside(x, y))
            {
                if(!center->isWaiting())
                {
                    centerHit();
                    center->action();
                }
            }
            break;
        }
        case SCALE:
        {
            if(center->inside(x, y))
            {
                if(!center->isWaiting())
                {
                    centerHit();
                    center->action();
                }
            }
            else if(north->inside(x, y))
            {
                north->updateReferenceValue(z);
                north->action();
                if(z-north->referenceValue > 10) //going up
                {
                    north->referenceValue = z;
                    northHit(true);
                }
                else if(north->referenceValue  - z> 10)  //goingdown
                {
                    north->referenceValue = z;
                    northHit(false);
                }
                else
                {
                }
            }
            break;
        }
        case BOOLEANS:
        {
            if(east->inside(x, y))
            {
                east->updateReferenceValue(z);
                east->action();
                if(z-east->referenceValue > 30) //going up
                {
                    east->referenceValue = z;
                    eastHit(true);
                }
                else if(east->referenceValue - z > 30)  //goingdown
                {
                    east->referenceValue = z;
                    eastHit(false);
                }
                else
                {
                }
            }
            if(center->inside(x, y))
            {
                if(!center->isWaiting())
                {
                    centerHit();
                    center->action();
                }
            }
            break;
        }
        default:
            break;
    }
}
/*
 Finger in center area
 */
void OperationContainerAlternative::centerHit()
{
    if(selectedPlane->axis == AxisPlane::NOAXIS)
    {
    }
    else
    {
        switch(opType)
        {
            case NOOPERATION:
            {
                if(links->size() > 0)
                {
                    opType = TRANSLATE;
                    east->limitTime = 1000;
                }
                break;
            }
            case TRANSLATE:
            {
                opType = ROTATE;
                east->limitTime = 1000;
                break;
            }
            case ROTATE:
            {
                opType = SCALE;
                east->limitTime = 1000;
                break;
            }
            case SCALE:
            {
                if(links->size() ==2)
                {
                    opType = BOOLEANS;
                    //make booleans
                    east->limitTime = 2000;
                    makePreviews(links->at(0), links->at(1));
                    leftArrowTime = ofGetElapsedTimeMillis();
                    rightArrowTime = ofGetElapsedTimeMillis();
                    chosen = possiblebooleans.front();
                }
                else
                {
                    east->limitTime = 1000;
                    opType = NOOPERATION;
                }
                break;
            }
            case BOOLEANS:
            {
                switch(chosen)
                {
                    case BOOLEAN_UNION:
                    case BOOLEAN_INTERSECTION:
                    case BOOLEAN_DIFFERENCE_A_MINUS_B:
                    case BOOLEAN_DIFFERENCE_B_MINUS_A:
                    case BOOLEAN_SYMMETRIC_DIFFERENCE:
                    {
                        shapes->lockVector();
                        shapes->addElement(newComposite);
                        shapes->unlockVector();
                        break;
                    }
                    case NOBOOLEAN:
                    default:
                        break;
                }
                east->limitTime = 1000;
                opType = NOOPERATION;
                break;
            }
            default:
                break;
        }
    }
}
/*
 Finger in west area
 */
void OperationContainerAlternative::westHit(bool directionUp)
{
    float translatevalue = 5;
    float rotatevalue = 5;
    float scalevalue =  0.05;
    Shape3D::INCRTYPE incrtype;
    if(directionUp)
    {
        incrtype = Shape3D::INCR;
    }
    else
    {
        incrtype = Shape3D::DECR;
    }
    switch(opType)
    {
        case NOOPERATION:
        {
            break;
        }
        case TRANSLATE:
        {
            break;
        }
        case ROTATE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::Z, incrtype,rotatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::Y, incrtype,-rotatevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::Y, incrtype,-rotatevalue);
                    }
                    break;
            }
            break;
        }
        case SCALE:
        {
            break;
        }
        case BOOLEANS:
        {
            if(ofGetElapsedTimeMillis() - leftArrowTime > 2000)
            {
                if(incrtype == Shape3D::INCR)
                {
                    for(int i = 0; i < possiblebooleans.size(); i++)
                    {
                        if(chosen == possiblebooleans.at(i))
                        {
                            if(1+i==possiblebooleans.size())
                            {
                                chosen = possiblebooleans.front();
                            }
                            else
                            {
                                chosen = possiblebooleans.at(i+1);
                            }
                            leftArrowTime = ofGetElapsedTimeMillis();
                            break;
                        }
                    }
                }
                else
                {
                    for(int i = 0; i < possiblebooleans.size(); i++)
                    {
                        if(chosen == possiblebooleans.at(i))
                        {
                            if(i-1 < 0)
                            {
                                chosen = possiblebooleans.back();
                            }
                            else
                            {
                                chosen = possiblebooleans.at(i-1);
                            }
                            leftArrowTime = ofGetElapsedTimeMillis();
                            break;
                        }
                    }
                }
            }
            break;
        }
        default:
            break;
    }
}
/*
 Finger in east area
 */
void OperationContainerAlternative::eastHit(bool directionUp)
{
    float translatevalue = 5;
    float rotatevalue = 5;
    float scalevalue =  0.05;
    Shape3D::INCRTYPE incrtype;
    if(directionUp)
    {
        incrtype = Shape3D::INCR;
    }
    else
    {
        incrtype = Shape3D::DECR;
    }
    switch(opType)
    {
        case NOOPERATION:
        {
            break;
        }
        case TRANSLATE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::Y, incrtype, translatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::Z, incrtype, translatevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::X, incrtype, translatevalue);
                    }
                    break;
            }
            break;
        }
        case ROTATE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::Z, incrtype,rotatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::X, incrtype,rotatevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::Y, incrtype,rotatevalue);
                    }
                    break;
            }
            break;
        }
        case SCALE:
        {
            break;
        }
        case BOOLEANS:
        {
            break;
        }
        default:
            break;
    }
}
/*
 Finger in north area
 */
void OperationContainerAlternative::northHit(bool directionUp)
{
    float translatevalue = 5;
    float rotatevalue = 5;
    float scalevalue =  0.5;
    Shape3D::INCRTYPE incrtype;
    if(directionUp)
    {
        incrtype = Shape3D::INCR;
    }
    else
    {
        incrtype = Shape3D::DECR;
    }
    switch(opType)
    {
        case NOOPERATION:
        {
            break;
        }
        case TRANSLATE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::Y, incrtype, translatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::Z, incrtype, translatevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::X, incrtype, -translatevalue);
                    }
                    break;
            }
            break;
        }
        case ROTATE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::X, incrtype,rotatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::X, incrtype,rotatevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::Z, incrtype,rotatevalue);
                    }
                    break;
            }
            break;
        }
        case SCALE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::SCALE, Shape3D::Y, incrtype, scalevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::SCALE, Shape3D::Z, incrtype, scalevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::SCALE, Shape3D::X, incrtype, scalevalue);
                    }
                    break;
            }
            break;
        }
        case BOOLEANS:
        {
            break;
        }
        default:
            break;
    }
}
/*
 Finger in south area
 */
void OperationContainerAlternative::southHit(bool directionUp)
{
    float translatevalue = 5;
    float rotatevalue = 5;
    float scalevalue = 0.05;
    Shape3D::INCRTYPE incrtype;
    if(directionUp)
    {
        incrtype = Shape3D::INCR;
    }
    else
    {
        incrtype = Shape3D::DECR;
    }
    switch(opType)
    {
        case NOOPERATION:
        {
            break;
        }
        case TRANSLATE:
        {
            break;
        }
        case ROTATE:
        {
            switch(selectedPlane->axis)
            {
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::X, incrtype,rotatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::Y,incrtype,rotatevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ )
                    {
                        (*it)->change(Shape3D::ROTATE, Shape3D::Z, incrtype,rotatevalue);
                    }
                    break;
            }
            break;
        }
        case SCALE:
        {
            break;
        }
        case BOOLEANS:
        {
            break;
        }
        default:
            break;
    }
}
/*
 Make boolean shape from 2 3D shapes
 */
void OperationContainerAlternative::makePreviews(Shape3D* objA, Shape3D * objB)
{
    newComposite = new Composite3DObject();
    newComposite->add(objA);
    newComposite->add(objB);
    newComposite->allOperations();
    possiblebooleans.push_back(NOBOOLEAN);
    if(newComposite->meshUnion.getNumVertices()!=0)
    {
        possiblebooleans.push_back(BOOLEAN_UNION);
    }
    if(newComposite->meshIntersect.getNumVertices()!=0)
    {
        possiblebooleans.push_back(BOOLEAN_INTERSECTION);
    }
    if(newComposite->meshDifferenceAB.getNumVertices()!=0)
    {
        possiblebooleans.push_back(BOOLEAN_DIFFERENCE_A_MINUS_B);
    }
    if(newComposite->meshDifferenceBA.getNumVertices()!=0)
    {
        possiblebooleans.push_back(BOOLEAN_DIFFERENCE_B_MINUS_A);
    }
    if(newComposite->meshSymDifference.getNumVertices()!=0)
    {
        possiblebooleans.push_back(BOOLEAN_SYMMETRIC_DIFFERENCE);
    }
    newComposite->changeColor(new ofColor(CLAMP((int)((objA->color->r + objB->color->r)/2), 0,255),
                                          CLAMP((int)((objA->color->g + objB->color->g)/2), 0,255),
                                          CLAMP((int)((objA->color->b + objB->color->b)/2), 0,255)
                                          ));//boolean color is a mix of the original colors
}