//
//  CopyContainer.h
//  Carver
//
//  Created by paulobala on 31/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_CopyContainer_h
#define Carver_CopyContainer_h

#include "ContainerToken.h"
#include "Basic3DObjectFromCopy.h"
#include "OnScreenOption.h"

class CopyContainerOption: public OnScreenOption
{
public:
    ofTrueTypeFont font14;
    ofTrueTypeFont font10;
    ContainerToken * container;
    
public:
    CopyContainerOption(ofVec2f center_, float radius_, string name_, ofVec3f * point_, LockableVector<Shape3D* > * shapes_, ofColor color_, ContainerToken * container_){
        center = center_; 
        radius = radius_;
        name = name_;
        entryPoint = point_;
        color = color_;
        shapes = shapes_;
        container = container_;
        ofTrueTypeFont::setGlobalDpi(72);
        font14.loadFont("fonts/helveticaNeue.ttf", 14, true, true);
        font10.loadFont("fonts/helveticaNeue.ttf", 10, true, true);
    }
    
    void draw(){
        ofPushStyle();
        ofSetColor(color);
        ofCircle(center, radius);
        
         if(container->getIsOnTable()){
             
            int numOption = container->links.size();
            
            if(numOption == 0){
                
                ofVec2f centroidMarker = ofVec2f(container->getToken()->getX()*ofGetWidth(),
                                                 container->getToken()->getY()*ofGetHeight());
                ofSetColor(COLORSCHEME_DARKGREY);
                ofSetLineWidth(4);
                ofLine(center,centroidMarker);
                
                ofPushMatrix();
                ofTranslate(center.x,center.y, 0);
                name = "EMPTY";
                ofSetColor(COLORSCHEME_TEXT_WHITE);
                font14.drawString(name, -15,0);
                ofPopMatrix();
            }else{
                float sizerect = 360/(float)(numOption);
                
                ofVec2f centroidMarker = ofVec2f(container->getToken()->getX()*ofGetWidth(),
                                                 container->getToken()->getY()*ofGetHeight());
                ofSetColor(COLORSCHEME_DARKGREY);
                ofSetLineWidth(4);
                ofLine(center,centroidMarker);
                
                for(int i = 0; i < container->links.size(); i++){
                    ofColor * coltemp = container->links.at(i)->color;
                    ofSetColor(coltemp->r, coltemp->g, coltemp->b);
                    ofVec2f firstpoint = center;
                    firstpoint.y = firstpoint.y - radius -5;
                    firstpoint.rotate(sizerect*i, center);
                    ofCircle(firstpoint,8);
                }
                
                ofPushMatrix();
                ofTranslate(center.x,center.y, 0);
                name = "COPY";
                ofSetColor(COLORSCHEME_TEXT_WHITE);
                font14.drawString(name, -15,0);
                ofPopMatrix();
            }
        }else{
            
            
            int numOption = container->links.size();
            
            if(numOption == 0){
                
                ofPushMatrix();
                ofTranslate(center.x,center.y, 0);
                name = "EMPTY";
                ofSetColor(COLORSCHEME_TEXT_WHITE);
                font14.drawString(name, -15,0);
                ofPopMatrix();
                
            }else{
                float sizerect = 360/(float)(numOption);
                
                for(int i = 0; i < container->links.size(); i++){
                    ofColor * coltemp = container->links.at(i)->color;
                    ofVec2f firstpoint = center;
                    ofSetColor(coltemp->r, coltemp->g, coltemp->b);
                    firstpoint.y = firstpoint.y - radius -5;
                    firstpoint.rotate(sizerect*i, center);
                    ofCircle(firstpoint,8);
                }
                
                ofPushMatrix();
                ofTranslate(center.x,center.y, 0);
                name = "COPY";
                ofSetColor(COLORSCHEME_TEXT_WHITE);
                font14.drawString(name, -15,0);
                ofPopMatrix();
            }
            
            
            ofPushMatrix();
            ofTranslate(center.x,center.y, 0);
            name = "NOT ON TABLE";
            ofSetColor(ofColor::white);
            font10.drawString(name, -25,20);
            ofPopMatrix();
        }
        
        
        ofPopStyle();
    };
    
    void action(){
        ofVec3f centroidcontainer = container->getCentroid();
        for (std::vector<Shape3D*>::iterator it  = container->links.begin()  ; it < container->links.end(); it++ ){
            ofMeshtCAD tempMesh = (*it)->mesh;
           
            ofVec3f point = (*entryPoint) + (tempMesh.getCentroid() - centroidcontainer);
            cout << point << endl;
            
            Basic3DObjectFromCopy * copy = new Basic3DObjectFromCopy( point, tempMesh);
            
            copy->changeColor(new ofColor(ofRandom(0, 255),ofRandom(0, 255),ofRandom(0, 255)));
            shapes->lockVector();
            shapes->addElement(copy); 
            shapes->unlockVector();
        }
    };
    
    bool checkHit(float x, float y){
        if( ofVec2f(x,y).distance(center) < radius){
            action();
            return true;
        };
        return false;
    }
};



#endif
