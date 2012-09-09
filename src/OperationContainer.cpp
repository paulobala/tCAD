//
//  OperationContainer.cpp
//  Carver
//
//  Created by paulobala on 03/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#include <iostream>
#include "OperationContainer.h"
#include "ColorScheme.h"

OperationContainer::OperationContainer (std::vector<Shape3D*> * links_,  AxisPlane * selectedPlane_, LockableVector<Shape3D* > * shapes_){
    
    links = links_;
    shapes = shapes_;
    selectedPlane = selectedPlane_;
    opType = NOOPERATION;
    
    ofTrueTypeFont::setGlobalDpi(72);
    font.loadFont("verdana.ttf", 14, true, true);
    
    chosen = NOBOOLEAN;
    
    east = new EastContainer(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    west = new WestContainer(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    south = new SouthContainer(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    north = new NorthContainer(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    center = new CenterContainer(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0));
    
    translateImg.loadImage("images/translate.png");
    scaleImg.loadImage("images/scale.png");
    rotateImg.loadImage("images/rotate.png"); 
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
    scaledown.loadImage("images/scaledown.png");
    scaleup.loadImage("images/scaleup.png");
    rotateleft.loadImage("images/rotateleft.png");
    rotateright.loadImage("images/rotateright.png");
    rotatedown.loadImage("images/rotatedown.png");
    rotateup.loadImage("images/rotateup.png");
    godown.loadImage("images/godown.png");
    goup.loadImage("images/goup.png");

} 

void OperationContainer::changedLinks(){
    if(links->size()== 0){
        opType = NOOPERATION;
    }
    else{
        if(opType == BOOLEANS){
            opType = NOOPERATION;
        }
    }

}

void OperationContainer::changeAxis(){
    if (selectedPlane->axis == AxisPlane::NOAXIS) {
        opType = NOOPERATION;
    }
}

void OperationContainer::update(float x, float y, float angle){
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
    
    ofVec2f point4BiggerSquare = point1u;
    point4BiggerSquare.rotate(-90,point4);
    ofVec2f point1BiggerSquare = point4u;
    point1BiggerSquare.rotate(90, point1);
    ofVec2f point2BiggerSquare = point1l;
    point2BiggerSquare.rotate(90,point2);
    ofVec2f point3BiggerSquare = point4r;
    point3BiggerSquare.rotate(-90,point3);
    
    if(opType == ROTATE){
    east->update(point1u, point1BiggerSquare, point2BiggerSquare, point2d);
    west->update(point4BiggerSquare, point4u, point3d, point3BiggerSquare);
    
    north->update(point4BiggerSquare, point1BiggerSquare, point1l, point4r);
    south->update(point3r, point2l, point2BiggerSquare, point3BiggerSquare);
    
    center->update(point4, point1, point2, point3);
        
    }else{
        east->update(point1, point1l, point2l, point2);
        west->update(point4r, point4, point3, point3r);
        
        north->update(point4u, point1u, point1, point4);
        south->update(point3, point2, point2d, point3d);
        
        center->update(point4, point1, point2, point3);
    }
}

void OperationContainer::draw(){
    ofPushMatrix();
    ofPushStyle();
    //ofTranslate(0, -100);
    center->color = COLORSCHEME_GREY;
    east->color = COLORSCHEME_GREY;
    west->color = COLORSCHEME_GREY;
    east->color = COLORSCHEME_GREY;
    west->color = COLORSCHEME_GREY;
    
    switch(opType){
        case NOOPERATION:{
            center->color = COLORSCHEME_GREY;
            east->color = COLORSCHEME_GREY;
            west->color = COLORSCHEME_GREY;
            east->color = COLORSCHEME_GREY;
            west->color = COLORSCHEME_GREY;
            
            center->draw();
            //east->draw();
            //west->draw();
            
            break;}
        case TRANSLATE:{
            
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    changeAxis();
                    break;
                case AxisPlane::X_Z:
                    center->color = COLORSCHEME_GREEN;
                    east->color = COLORSCHEME_GREEN;
                    west->color = COLORSCHEME_GREEN;
                    center->draw(translateImg);
                    east->draw(goup);
                    west->draw(godown);
                    break;
                case AxisPlane::X_Y:
                    center->color = COLORSCHEME_BLUE;
                    east->color = COLORSCHEME_BLUE;
                    west->color = COLORSCHEME_BLUE;
                    center->draw(translateImg);
                    east->draw(goup);
                    west->draw(godown);
                    break;
                case AxisPlane::Y_Z:
                    center->color = COLORSCHEME_RED;
                    east->color = COLORSCHEME_RED;
                    west->color = COLORSCHEME_RED;
                    center->draw(translateImg);
                    east->draw(goup);
                    west->draw(godown);
                    break;
            }
            
           
            break;}
        case ROTATE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    changeAxis();
                    break;
                case AxisPlane::X_Z:
                    center->color = COLORSCHEME_GREY;
                    east->color = COLORSCHEME_BLUE_TRANS;
                    west->color = COLORSCHEME_BLUE_TRANS;
                    south->color = COLORSCHEME_RED_TRANS;
                    north->color = COLORSCHEME_RED_TRANS;
                    center->draw(rotateImg);
                    east->draw(rotateleft);
                    west->draw(rotateright);
                    north->draw(rotateup);
                    south->draw(rotatedown);
                    break;
                case AxisPlane::X_Y:
                    center->color = COLORSCHEME_GREY;
                    east->color = COLORSCHEME_GREEN_TRANS;
                    west->color = COLORSCHEME_GREEN_TRANS;
                    south->color = COLORSCHEME_RED_TRANS;
                    north->color = COLORSCHEME_RED_TRANS;
                    center->draw(rotateImg);
                    east->draw(rotateleft);
                    west->draw(rotateright);
                    north->draw(rotateup);
                    south->draw(rotatedown);
                    break;
                case AxisPlane::Y_Z:
                    center->color = COLORSCHEME_GREY;
                    east->color = COLORSCHEME_GREEN_TRANS;
                    west->color = COLORSCHEME_GREEN_TRANS;
                    south->color = COLORSCHEME_BLUE_TRANS;
                    north->color = COLORSCHEME_BLUE_TRANS;
                    center->draw(rotateImg);
                    east->draw(rotateleft);
                    west->draw(rotateright);
                    north->draw(rotateup);
                    south->draw(rotatedown);
                    break;
            }
           
            break;}
            
        case SCALE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    changeAxis();
                    break;
                case AxisPlane::X_Z:
                    center->color = COLORSCHEME_GREEN;
                    north->color = COLORSCHEME_GREEN;
                    west->color = COLORSCHEME_GREEN;
                    center->draw(scaleImg);
                    north->draw(scaleup);
                    west->draw(scaledown);
                    break;
                case AxisPlane::X_Y:
                    center->color = COLORSCHEME_BLUE;
                    north->color = COLORSCHEME_BLUE;
                    west->color = COLORSCHEME_BLUE;
                    center->draw(scaleImg);
                    north->draw(scaleup);
                    west->draw(scaledown);
                                break;
                case AxisPlane::Y_Z:
                    center->color = COLORSCHEME_RED;
                    north->color = COLORSCHEME_RED;
                    west->color = COLORSCHEME_RED;
                    center->draw(scaleImg);
                    north->draw(scaleup);
                    west->draw(scaledown);
                    break;
            }
          
            break;}
        case BOOLEANS:{
            
            if(selectedPlane->axis == AxisPlane::NOAXIS){
             changeAxis();
            }
            else{
            center->color = COLORSCHEME_DARKGREY;
            east->color = COLORSCHEME_DARKGREY;
            west->color = COLORSCHEME_DARKGREY;
            north->color = COLORSCHEME_DARKGREY;
            
             switch(chosen){
                case BOOLEAN_UNION: north->draw(unionImg);;break;
                case BOOLEAN_INTERSECTION: north->draw(intersectImg);break;
                case BOOLEAN_DIFFERENCE_A_MINUS_B: north->draw(diffABImg);break;
                case BOOLEAN_DIFFERENCE_B_MINUS_A: north->draw(diffBAImg);break; 
                case BOOLEAN_SYMMETRIC_DIFFERENCE: north->draw(symDifImg);break;
                case NOBOOLEAN:north->draw	(noBoolImg);break;
                default: break;
            }

           
            east->draw(leftArrowImg);
            west->draw(rightArrowImg);
            center->draw();
            }
            break;}
        default:break;
    }
    ofPopStyle();
    
    ofPopMatrix();
}

bool OperationContainer::inside(float x, float y, float z){
    if(opType == NOOPERATION){
    
        if(center->inside(x, y)){return true;}
    }else {
        if(west->inside(x, y)){return true;
        }else if(east->inside(x, y)){return true;
        }else  if(center->inside(x, y)){return true;
        }else if(north->inside(x, y)){return true;
        }else  if(south->inside(x, y)){return true;
        }   
    }
    return false;
    
}
bool OperationContainer::insideDepth(float x, float y, float z){
    if(opType == NOOPERATION){
        
    }else {
        if(west->inside(x, y)){return true;
        }else if(east->inside(x, y)){return true;
        }else if(north->inside(x, y)){return true;
        }else  if(south->inside(x, y)){return true;
        }   
    }
    return false;
    
}

void OperationContainer::action(float x, float y, float z){
    if(opType == NOOPERATION){
        
        if(center->inside(x, y)){
            if(!center->isWaiting()){
            centerHit();
            center->action();
            }
        }}
        else {
            if(west->inside(x, y)){
                westHit();west->action();
            }
            if(east->inside(x, y)){
               eastHit();east->action();
            }
            if(center->inside(x, y)){
                if(!center->isWaiting()){
                    centerHit();
                    center->action();
                }
            }
            if(north->inside(x, y)){ northHit();north->action();
            }
            if(south->inside(x, y)){ southHit();south->action();
            }
        }
        
    }

void OperationContainer::centerHit(){
    if(selectedPlane->axis == AxisPlane::NOAXIS){
        
//        if(links->size() == 2){
//            if(opType == NOOPERATION){
//                opType = BOOLEANS;
//            }else{
//                opType = NOOPERATION;
//            }
//        }
//        else{
//            opType = NOOPERATION;
//        }
        
    }else{
        
    switch(opType){
            case NOOPERATION:{
                if(links->size() >0){
                    opType = TRANSLATE;
                    east->limitTime = 1000;
                    west->limitTime = 1000;
                }
                break;}
            case TRANSLATE:{
                opType = ROTATE;
                east->limitTime = 1000;
                west->limitTime = 1000;
                break;}
            case ROTATE:{
                opType = SCALE;
                east->limitTime = 1000;
                west->limitTime =1000;
                break;}
            case SCALE:{
                if(links->size() ==2)
                {   opType = BOOLEANS;
                    east->limitTime = 2000;
                    west->limitTime = 2000;
                    //make booleans
                    makePreviews(links->at(0), links->at(1));
                    
                    leftArrowTime = ofGetElapsedTimeMillis();
                    rightArrowTime = ofGetElapsedTimeMillis();
                    chosen = possibleBooleans.front();
                }
                else{
                    east->limitTime = 1000;
                    west->limitTime = 1000;
                    opType = NOOPERATION;
                }
                break;}
            case BOOLEANS:{
                switch(chosen){
                    case BOOLEAN_UNION: 
                    case BOOLEAN_INTERSECTION:
                    case BOOLEAN_DIFFERENCE_A_MINUS_B:
                    case BOOLEAN_DIFFERENCE_B_MINUS_A:
                    case BOOLEAN_SYMMETRIC_DIFFERENCE:{
                        shapes->lockVector();
                        shapes->addElement(newComposite); 
                        shapes->unlockVector();
                        break;
                    }
                    case NOBOOLEAN:
                    default: break;
                }
                east->limitTime = 1000;
                west->limitTime = 1000;
                opType = NOOPERATION;
                break;}
            default:
                break;
        }   
        
    }
    
    
}

void OperationContainer::eastHit(){
    float translatevalue = 5;
    float rotatevalue = 5;
    float scalevalue =  0.05;
    switch(opType){
        case NOOPERATION:{
            
            break;}
        case TRANSLATE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::Y, Shape3D::INCR, translatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::Z, Shape3D::INCR, translatevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::X, Shape3D::DECR, translatevalue);
                    }
                    break;
            }
            break;}
        case ROTATE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::Z, Shape3D::DECR,rotatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::Y, Shape3D::INCR,rotatevalue);
                    }
                    
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::Y, Shape3D::INCR,rotatevalue);
                    }
                    break;
            }
            break;}
        case SCALE:{
            break;}
        case BOOLEANS:{
            if(ofGetElapsedTimeMillis() - leftArrowTime > 2000){
            for(int i = 0; i < possibleBooleans.size(); i++){
                
                if(chosen == possibleBooleans.at(i)){
                    if(1+i==possibleBooleans.size()){
                        chosen = possibleBooleans.front();
                    }else{
                        chosen = possibleBooleans.at(i+1);
                    }
                    leftArrowTime = ofGetElapsedTimeMillis();
                    break;
                }
            }
            }
            break;}
        default:break;
    }
    
}
void OperationContainer::westHit()
{
    float translatevalue = 5;
    float rotatevalue = 5;
    float scalevalue =  0.05;
    switch(opType){
        case NOOPERATION:{
            
            break;}
        case TRANSLATE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::Y, Shape3D::DECR, translatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::Z, Shape3D::DECR, translatevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::TRANSLATE, Shape3D::X, Shape3D::INCR, translatevalue);
                    }
                    break;
            }
            break;}
        case ROTATE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::Z, Shape3D::INCR,rotatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::Y, Shape3D::DECR,rotatevalue);
                    }
                    
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::Y, Shape3D::DECR,rotatevalue);
                    }
                    break;
            }
            break;}
        case SCALE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::SCALE, Shape3D::Y, Shape3D::DECR, scalevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::SCALE, Shape3D::Z, Shape3D::DECR, scalevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::SCALE, Shape3D::X, Shape3D::DECR, scalevalue);
                    }
                    break;
            }
            break;}
        case BOOLEANS:{
            if(ofGetElapsedTimeMillis()-rightArrowTime > 2000){
            for(int i = 0; i < possibleBooleans.size(); i++){
                
                if(chosen == possibleBooleans.at(i)){
                    if(i-1 < 0){
                        chosen = possibleBooleans.back();
                    }else{
                        chosen = possibleBooleans.at(i-1);
                    }
                    rightArrowTime = ofGetElapsedTimeMillis();
                    break;
                }
            }
            }
            break;}
        default:break;
    }
    
    
}
void OperationContainer::northHit(){
    float translatevalue = 5;
    float rotatevalue = 5;
    float scalevalue =  0.05;
    switch(opType){
        case NOOPERATION:{
            
            break;}
        case TRANSLATE:{
            break;}
        case ROTATE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::X, Shape3D::DECR,rotatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::X, Shape3D::DECR,rotatevalue);
                    }
                    
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::Z, Shape3D::DECR,rotatevalue);
                    }
                    break;
            }
            break;}
        case SCALE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::SCALE, Shape3D::Y, Shape3D::INCR, scalevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::SCALE, Shape3D::Z, Shape3D::INCR, scalevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::SCALE, Shape3D::X, Shape3D::INCR, scalevalue);
                    }
                    break;
            }

            break;}
        case BOOLEANS:{
            
            break;}
        default:break;
    }
    
}

void OperationContainer::southHit(){
    float translatevalue = 5;
    float rotatevalue = 5;
    float scalevalue = 0.05;
    switch(opType){
        case NOOPERATION:{
            
            break;}
        case TRANSLATE:{
            break;}
        case ROTATE:{
            switch(selectedPlane->axis){
                case AxisPlane::NOAXIS:
                    break;
                case AxisPlane::X_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::X, Shape3D::INCR,rotatevalue);
                    }
                    break;
                case AxisPlane::X_Y:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::X, Shape3D::INCR,rotatevalue);
                    }
                    break;
                case AxisPlane::Y_Z:
                    for (std::vector<Shape3D*>::iterator it  = links->begin()  ; it < links->end(); it++ ){
                        (*it)->change(Shape3D::ROTATE, Shape3D::Z, Shape3D::INCR,rotatevalue);
                    }
                    break;
            }
            break;}
        case SCALE:{
            break;}
        case BOOLEANS:{
            
            break;}
        default:break;
    }
    
}

void OperationContainer::makePreviews(Shape3D* objA, Shape3D * objB)
{
//    objA->changeStyle(Shape3D::WIREFRAME);
//    objB->changeStyle(Shape3D::WIREFRAME);
    newComposite = new Composite3DObject();
    newComposite->add(objA);
    newComposite->add(objB);
    newComposite->allOperations();
    
    possibleBooleans.push_back(NOBOOLEAN);
    
    if(newComposite->meshUnion.getNumVertices()!=0){
        
        possibleBooleans.push_back(BOOLEAN_UNION);
        
    }
    if(newComposite->meshIntersect.getNumVertices()!=0){
       
        possibleBooleans.push_back(BOOLEAN_INTERSECTION);
        
    }
    if(newComposite->meshDifferenceAB.getNumVertices()!=0){
       
        possibleBooleans.push_back(BOOLEAN_DIFFERENCE_A_MINUS_B);
        
    }
    if(newComposite->meshDifferenceBA.getNumVertices()!=0){
      
        possibleBooleans.push_back(BOOLEAN_DIFFERENCE_B_MINUS_A);
        
    }
    if(newComposite->meshSymDifference.getNumVertices()!=0){
        
        possibleBooleans.push_back(BOOLEAN_SYMMETRIC_DIFFERENCE);
        
    }
    
    newComposite->changeColor(new ofColor(
                                          CLAMP((int)((objA->color->r + objB->color->r)/2), 0,255),
                                          CLAMP((int)((objA->color->g + objB->color->g)/2), 0,255),
                                          CLAMP((int)((objA->color->b + objB->color->b)/2), 0,255)
                                          ));
}
