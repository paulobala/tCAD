//
//  SecondMenu.h
//  Carver
//
//  Created by paulobala on 04/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_SecondMenu_h
#define Carver_SecondMenu_h

#include "OnScreenOption.h"
#include "lockableVector.h"
#include "Shape3D.h"
#include "STLSubOptions.h"
#include "Token.h"
#include "OnTokenOption.h"
#include "ColorScheme.h"
#include "SaveKinectOption.h"
#include "SaveContourOption.h"

class OnTokenContentCreationUI{
    Token * tuioObject;
    LockableVector<Shape3D* > * shapes;
    LockableVector<ContainerToken* > * containers;
    ofVec3f * entryPoint; 
    ofVec2f nw, ne, se, sw, center;
    float radius;
    OnTokenOption * stlOption;
    OnTokenOption * kinectOption;
    OnTokenOption * contourOption;
    OnTokenOption * copyOption;
    vector<OnScreenOption *> options;
    ofImage textureCircle;
    
    float setupSTLSubOptions(){
        int numOptions = 6;
        float center = (numOptions* 100)/2;
        float x1 = tuioObject->getX()*ofGetWidth() - center;
        float y1 = tuioObject->getY()*ofGetHeight() - 300;
        
        x1 = x1 + 30 + 20;
        ofImage img;
        img.loadImage("images/cube.png");
        
        options.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Cube",entryPoint, shapes, "stls/cube.stl",ofColor(233,29,45),img));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        img.loadImage("images/cone.png");
        options.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Cone",entryPoint, shapes, "stls/cone.stl",ofColor(245,131,31),img));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        img.loadImage("images/cylinder.png");
        options.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Cylinder",entryPoint, shapes, "stls/cylinder.stl",ofColor(70,183,73),img));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        img.loadImage("images/pyramid.png");
        options.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Pyramid",entryPoint, shapes, "stls/pyramid.stl",ofColor(245,131,31),img));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        img.loadImage("images/sphere.png");
        options.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Sphere",entryPoint, shapes, "stls/sphere.stl",ofColor(117,206,219),img));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        img.loadImage("images/tube.png"); 
        options.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Tube",entryPoint, shapes, "stls/tube.stl",ofColor(0,159,215),img));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        options.push_back(new BackOption(ofVec2f(x1,y1),40, "BACK", entryPoint, COLORSCHEME_GREY));
        return 2;
    }
    void setupCopyContainerOptions(){
        int numOptions = 0;
        vector<ContainerToken*> temp = containers->getObjects();
        for (std::vector<ContainerToken*>::iterator it=temp.begin() ; it < temp.end(); it++ ){
            //if((*it)->onTable){
            if((*it)->links.size()>0){
                numOptions ++;
            }
        }
        float center = (numOptions* 100)/2;
        float x1 = tuioObject->getX()*ofGetWidth() - center;
        float y1 = tuioObject->getY()*ofGetHeight() - 300;
        int count = 0;
        for (std::vector<ContainerToken*>::iterator it=temp.begin() ; it < temp.end(); it++ ){
            //if((*it)->onTable){
            if((*it)->links.size()>0){
                x1 = x1 + 30 + 20;
                options.push_back(new CopyContainerOption(ofVec2f(x1,y1),40, "Copy",entryPoint, shapes, ofColor(233,29,45), (*it) ));
                x1 = x1 + 20 +30;
                count++;
            }
        }
        if(count!=0){ x1 = x1 + 30 + 20;}
        options.push_back(new BackOption(ofVec2f(x1,y1),40, "BACK", entryPoint, COLORSCHEME_GREY ));
    }
    void setupKinectOptions(){
        float center = (2* 100)/2;
        float x1 = tuioObject->getX()*ofGetWidth() - center;
        float y1 = tuioObject->getY()*ofGetHeight() - 300;
        
        x1 = x1 + 30 + 20;
        options.push_back(new SaveKinectOption(ofVec2f(x1,y1),40, "SAVE",entryPoint,COLORSCHEME_MENU_GREEN));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        options.push_back(new BackOption(ofVec2f(x1,y1),40, "DISCARD",entryPoint,COLORSCHEME_MENU_ORANGE));
        
        
    }
    void setupContourOptions(){
        float center = (2* 100)/2;
        float x1 = tuioObject->getX()*ofGetWidth() - center;
        float y1 = tuioObject->getY()*ofGetHeight() - 300;
        
        x1 = x1 + 30 + 20;
        options.push_back(new SaveContourOption(ofVec2f(x1,y1),40, "SAVE",entryPoint,COLORSCHEME_MENU_GREEN));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        options.push_back(new BackOption(ofVec2f(x1,y1),40, "DISCARD",entryPoint,COLORSCHEME_MENU_ORANGE));
    }
    bool isOnTable;
    bool visible;
public:
    void setToken(Token * value){tuioObject = value;}
    
    enum CURRENTMENU{
        STL, KINECT, COPY, CONTOUR, MAINMENU
    };
    
    enum RETURNTYPE{
        RETURNKINECT, RETURNSTL, RETURNCOPY, RETURNCONTOUR, NORETURN
    };
    
    CURRENTMENU current;
    bool getIsOnTable(){return isOnTable;}
    void setIsOnTable(bool value){isOnTable = value;}
    
    OnTokenContentCreationUI(Token * token_, ofVec3f * point_, LockableVector<Shape3D* > * shapes_, LockableVector<ContainerToken* > * containers_){
        entryPoint = point_;
        tuioObject = token_;
        shapes = shapes_;
        containers = containers_;
        radius = 200;
        stlOption = new OnTokenOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), OnTokenOption::STL);
        stlOption->color = COLORSCHEME_MENU_GREEN;
        kinectOption = new OnTokenOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), OnTokenOption::KINECT);
        kinectOption->color = COLORSCHEME_MENU_BLUE;
        contourOption = new OnTokenOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), OnTokenOption::CONTOUR);
        contourOption->color = COLORSCHEME_MENU_YELLOW,
        copyOption = new OnTokenOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), OnTokenOption::COPY);
        copyOption->color = COLORSCHEME_MENU_ORANGE;
        current = MAINMENU;
        visible = true;
        isOnTable = true;
        textureCircle.loadImage("images/textureCircleMenu.png");
    }
    
    bool insideToken(float x, float y){
        ofPolyline polyline;
        polyline.addVertex(nw.x, nw.y);
        polyline.addVertex(ne.x, ne.y);
        polyline.addVertex(se.x, se.y);
        polyline.addVertex(sw.x, sw.y);
        polyline.close();
        int counter = 0;
        int i;
        double xinters;
        ofPoint p1,p2;
        
        int N = polyline.size();
        
        p1 = polyline[0];
        for (i=1;i<=N;i++) {
            p2 = polyline[i % N];
            if (y > MIN(p1.y,p2.y)) {
                if (y <= MAX(p1.y,p2.y)) {
                    if (x <= MAX(p1.x,p2.x)) {
                        if (p1.y != p2.y) {
                            xinters = (y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                            if (p1.x == p2.x || x <= xinters)
                                counter++;
                        }
                    }
                }
            }
            p1 = p2;
        }
        
        if (counter % 2 == 0) return false;
        else return true;
        
    }
    
    bool insideProtectedOptions(float x, float y){
        float center = (2* 100)/2;
        float x1 = tuioObject->getX()*ofGetWidth();
        float y1 = tuioObject->getY()*ofGetHeight() - 300;
        float radius = 150;
        if( ofVec2f(x,y).distance(ofVec2f(x1,y1)) < radius){
            return true;
        };
        return false;
    }
    
    void update(){
        if(isOnTable){
            float x = tuioObject->getX();
            float y = tuioObject->getY();
            float angle = -tuioObject->getAngle();
            
            center = ofVec2f(x*ofGetWidth(), y*ofGetHeight());
            
            ofVec2f centerpoint = ofVec2f(x*ofGetWidth(), y*ofGetHeight());
            
            ofVec2f point1 = ofVec2f(x*ofGetWidth() +cos(angle)*140, y*ofGetHeight()+sin(angle)*140);
            point1.rotate(-45, centerpoint);
            ofVec2f point2 = ofVec2f(x*ofGetWidth()+cos(angle+PI/2)*140,
                                     y*ofGetHeight()+sin(angle+PI/2)*140);
            point2.rotate(-45, centerpoint);
            ofVec2f point3 =  ofVec2f(x*ofGetWidth()+cos(angle+PI)*140, y*ofGetHeight()+sin(angle+PI)*140);
            point3.rotate(-45, centerpoint);
            ofVec2f point4 = ofVec2f(x*ofGetWidth()+cos(angle+PI+PI/2)*140, y*ofGetHeight()+sin(angle+PI+PI/2)*140);
            point4.rotate(-45, centerpoint);
            
            nw = point4;
            ne = point1;
            se = point2;
            sw = point3;
            
            switch(current){
                case STL: {
                    
                    float center = (options.size()* 100)/2;
                    float x1 = tuioObject->getX()*ofGetWidth() - center;
                    float y1 = tuioObject->getY()*ofGetHeight() - 300;
                    for(int i = 0; i < options.size(); i++){
                        x1 = x1 + 30 + 20;
                        options[i]->center = ofVec2f(x1,y1);
                        x1 = x1 + 20 +30;
                    }
                    
                    break;}
                case KINECT:  {
                    float center = (options.size()* 100)/2;
                    float x1 = tuioObject->getX()*ofGetWidth() - center;
                    float y1 = tuioObject->getY()*ofGetHeight() - 300;
                    for(int i = 0; i < options.size(); i++){
                        x1 = x1 + 30 + 20;
                        options[i]->center = ofVec2f(x1,y1);
                        x1 = x1 + 20 +30;
                    }
                    break;}
                case CONTOUR:  {
                    float center = (options.size()* 100)/2;
                    float x1 = tuioObject->getX()*ofGetWidth() - center;
                    float y1 = tuioObject->getY()*ofGetHeight() - 300;
                    for(int i = 0; i < options.size(); i++){
                        x1 = x1 + 30 + 20;
                        options[i]->center = ofVec2f(x1,y1);
                        x1 = x1 + 20 +30;
                    }
                    break;}
                case COPY:  {
                    float center = (options.size()* 100)/2;
                    float x1 = tuioObject->getX()*ofGetWidth() - center;
                    float y1 = tuioObject->getY()*ofGetHeight() - 300;
                    for(int i = 0; i < options.size(); i++){
                        x1 = x1 + 30 + 20;
                        options[i]->center = ofVec2f(x1,y1);
                        x1 = x1 + 20 +30;
                    }
                    break;}
                case MAINMENU:{
                    contourOption->update(nw,ne,center,angle);
                    copyOption->update(ne,se,center,angle+PI/2);
                    stlOption->update(se,sw,center,angle+PI);
                    kinectOption->update(sw,nw,center,angle+PI/2+PI);
                    break;}
                default:break;
            }
        }
    }
    
    void draw() {
        if(isOnTable){
            ofPushMatrix();
            if(visible){
                switch(current){
                    case STL: {
                        ofPushStyle();
                        ofSetColor(stlOption->color);
                        ofLine(nw,ne);
                        ofLine(ne, se);
                        ofLine(se,sw);
                        ofLine(sw,nw);
                        
                        for(int i = 0; i < options.size(); i++){
                            ofSetColor(COLORSCHEME_MENU_GREEN);
                            ofLine(ofVec2f(tuioObject->getX()*ofGetWidth(),tuioObject->getY()*ofGetHeight()), options[i]->center);
                            options[i]->draw();
                        }
                        
                        ofPopStyle();
                        break;}
                    case KINECT:  {
                        ofPushStyle();
                        ofSetColor(kinectOption->color);
                        ofLine(nw,ne);
                        ofLine(ne, se);
                        ofLine(se,sw);
                        ofLine(sw,nw);
                        for(int i = 0; i < options.size(); i++){
                            ofSetColor(kinectOption->color);
                            ofLine(ofVec2f(tuioObject->getX()*ofGetWidth(),tuioObject->getY()*ofGetHeight()), options[i]->center);
                            options[i]->draw();
                        }
                        ofPopStyle();
                        break;}
                    case CONTOUR:  {
                        ofPushStyle();
                        ofSetColor(contourOption->color);
                        ofLine(nw,ne);
                        ofLine(ne,se);
                        ofLine(se,sw);
                        ofLine(sw,nw);
                        
                        float x1 = tuioObject->getX()*ofGetWidth();
                        float y1 = tuioObject->getY()*ofGetHeight() - 300;
                        float radius = 150;
                        ofNoFill();
                        ofCircle(x1, y1, radius);
                        ofPushMatrix();
                        ofTranslate(x1,y1);
                        ofSetColor(contourOption->color);
                        textureCircle.setAnchorPercent(0.5,0.5);
                        textureCircle.draw(0,0,300,300);
                        ofPopMatrix();
                        ofFill();
                        for(int i = 0; i < options.size(); i++){
                            ofSetColor(contourOption->color);
                            ofLine(ofVec2f(tuioObject->getX()*ofGetWidth(),tuioObject->getY()*ofGetHeight()), options[i]->center);
                            options[i]->draw();
                        }
                        ofPopStyle();
                        break;}
                    case COPY:  {
                        ofPushStyle();
                        ofSetColor(copyOption->color);
                        ofLine(nw,ne);
                        ofLine(ne,se);
                        ofLine(se,sw);
                        ofLine(sw,nw);
                        for(int i = 0; i < options.size(); i++){
                            ofSetColor(copyOption->color);
                            ofLine(ofVec2f(tuioObject->getX()*ofGetWidth(),tuioObject->getY()*ofGetHeight()), options[i]->center);
                            options[i]->draw();
                        }
                        ofPopStyle();
                        break;}
                    case MAINMENU:  {
                        ofPushStyle();
                        contourOption->draw();
                        kinectOption->draw();
                        stlOption->draw();
                        copyOption->draw();
                        ofPopStyle();
                        break;}
                    default:break;
                }
            }
            ofPopMatrix(); 
        }
    }
    
    RETURNTYPE action(float x, float y){
        
        switch(current){
            case STL: {
                if(insideToken(x,y)){
                   visible = !visible;
                }
                break;}
            case KINECT:  {
                if(insideToken(x,y)){
                    visible = !visible;
                }
                break;}
            case CONTOUR:  {
                if(insideToken(x,y)){
                    visible = !visible;
                }
                break;}
            case COPY:  {
                if(insideToken(x,y)){
                    visible = !visible;
                }
                break;}
            case MAINMENU:{ 
                visible = true;
                if(stlOption->inside(x, y)){
                    current = STL;
                    options.clear();
                    setupSTLSubOptions();
                    update();return RETURNSTL;
                }else if(kinectOption->inside(x, y)){ current = KINECT;
                    options.clear();
                    setupKinectOptions();
                    update();return  RETURNKINECT;
                }else if(copyOption->inside(x, y)){current = COPY;
                    options.clear();
                    setupCopyContainerOptions();
                    update(); return  RETURNCOPY;
                }else if(contourOption->inside(x, y)){ current = CONTOUR;
                    options.clear();
                    setupContourOptions();
                    update();return  RETURNCONTOUR;
                }
            }
            default:
                ;
        }
        
        return NORETURN;
    }
    
    std::pair<bool, OnScreenOption*> checkHit(float x, float y){
        if(visible){
            for(int i = 0; i < options.size(); i++){
                if(options[i]->checkHit(x, y)){
                    STLSubOptions *d2 = dynamic_cast<STLSubOptions*>(options[i]);
                    if((bool)d2){
                        options.clear();
                        current = MAINMENU;
                        visible = true;
                        //                        setup1(4);
                        return pair<bool,OnScreenOption*>(true, d2);
                    }else{
                        CopyContainerOption *d5 = dynamic_cast<CopyContainerOption*>(options[i]);
                        if((bool)d5){
                            options.clear();
                            visible = true;
                            current = MAINMENU;
                            //                        setup1(4);
                            return pair<bool,CopyContainerOption*>(true, d5);
                        }else {
                            BackOption *d7 = dynamic_cast<BackOption*>(options[i]);
                            if((bool)d7){
                                options.clear();
                                current = MAINMENU;
                                visible = true;
                                update();
                                return pair<bool,BackOption*>(true, d7);
                            }else{ SaveKinectOption *d8 = dynamic_cast<SaveKinectOption*>(options[i]);
                                if((bool)d8){
                                    options.clear();
                                    current = MAINMENU;
                                    visible = true;
                                    return pair<bool,SaveKinectOption*>(true, d8);
                                }
                                else{
                                    SaveContourOption *d9 = dynamic_cast<SaveContourOption*>(options[i]);
                                    if((bool)d9){
                                        options.clear();
                                        current = MAINMENU;
                                        visible = true;
                                        return pair<bool,SaveContourOption*>(true, d9);
                                    }
                                }
                            }
                        }
                    }
                    visible = true;
                    return pair<bool,OnScreenOption*>(true, NULL);
                }
            }
            return pair<bool,OnScreenOption*>(false, NULL);
        }
    }
};
#endif
