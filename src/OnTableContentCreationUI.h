#ifndef Carver_CircleMenu_h
#define Carver_CircleMenu_h

#include "STLOption.h"

#include "OnScreenOption.h"
#include "KinectOption.h"
#include "STLSubOptions.h"
#include "CopyOption.h"
#include "CopyContainerOption.h"
#include "ContourOption.h"
#include "BackOption.h"
#include "lockableVector.h"
#include "SaveKinectOption.h"
#include "SaveContourOption.h"

#include "Shape3D.h"

#include "Token.h"



class OnTableContentCreationUI{
private:    
    float radius; 
    LockableVector<Shape3D* > * shapes;
    LockableVector<ContainerToken* > * containers;
    ofVec3f * entryPoint; 
    ofImage textureCircle;
    vector<OnScreenOption * > options; 
    bool onMainLevel;
    bool onContourMode;
    Token * token;
    bool isOnTable;
    void setupMain(){
        int numOptions = 4;
        float center = (numOptions* 100)/2;
        float x1 = token->getX()*ofGetWidth() - center;
        float y1 = token->getY()*ofGetHeight() - 100;
        x1 = x1 + 30 + 20;
        options.push_back(new ContourOption(ofVec2f(x1,y1),40, "Contour" ,entryPoint, COLORSCHEME_MENU_YELLOW ));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        options.push_back(new CopyOption(ofVec2f(x1,y1),40, "Copy" , entryPoint, shapes, COLORSCHEME_MENU_ORANGE)); 
        x1 = x1 + 20 +30;
        x1 = x1 + 30 + 20;
        options.push_back(new STLOption(ofVec2f(x1,y1),40, "STL",entryPoint, shapes, COLORSCHEME_MENU_GREEN ));
                x1 = x1 + 20 +30;
        options.push_back(new KinectOption(ofVec2f(x1,y1),40, "Kinect",entryPoint, shapes, COLORSCHEME_MENU_BLUE));
        onMainLevel = true;
        onContourMode = false;
        
    }
    
    void setupSTLSubOptions(){ 
        int numOptions = 6;
        float center = (numOptions* 100)/2;
        float x1 = token->getX()*ofGetWidth() - center;
        float y1 = token->getY()*ofGetHeight() - 200;
        
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
        options.push_back(new BackOption(ofVec2f(x1,y1),40, "BACK", entryPoint, ofColor(59,85,162)));
        
        onMainLevel = false;onContourMode = false; 
        
    }
    
    void setupCopyContainerOptions(){
        int numOptions = 0;
        vector<ContainerToken*> temp = containers->getObjects();
        for (std::vector<ContainerToken*>::iterator it=temp.begin() ; it < temp.end(); it++ ){
            
            if((*it)->links.size()>0){
                numOptions ++;
            }
            
        }
        float center = (numOptions* 100)/2;
        float x1 = token->getX()*ofGetWidth() - center;
        float y1 = token->getY()*ofGetHeight() - 200;
        int count = 0;
        for (std::vector<ContainerToken*>::iterator it=temp.begin() ; it < temp.end(); it++ ){
            
            if((*it)->links.size()>0){
                x1 = x1 + 30 + 20;
                options.push_back(new CopyContainerOption(ofVec2f(x1,y1),40, "Copy",entryPoint, shapes, ofColor(233,29,45), (*it) ));
                x1 = x1 + 20 +30;
                count++;
            }
            
        }
        if(count != 0){ x1 = x1 + 30 + 20;}
        options.push_back(new BackOption(ofVec2f(x1,y1),40, "BACK", entryPoint, COLORSCHEME_GREY));
        onMainLevel = false;onContourMode = false;
    }
    void setupContourOptions(){
        float center = (2* 100)/2;
        float x1 = token->getX()*ofGetWidth() - center;
        float y1 = token->getY()*ofGetHeight() - 200;
        
        x1 = x1 + 30 + 20;
        options.push_back(new SaveContourOption(ofVec2f(x1,y1),40, "SAVE",entryPoint,COLORSCHEME_MENU_GREEN));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        options.push_back(new BackOption(ofVec2f(x1,y1),40, "BACK",entryPoint,COLORSCHEME_MENU_ORANGE));
        onMainLevel = false;onContourMode = true;
    }
    void setupKinectOptions(){
        float center = (2* 100)/2;
        float x1 = token->getX()*ofGetWidth() - center;
        float y1 = token->getY()*ofGetHeight() - 200;
        
        x1 = x1 + 30 + 20;
        options.push_back(new SaveKinectOption(ofVec2f(x1,y1),40, "SAVE",entryPoint, COLORSCHEME_MENU_GREEN));
        x1 = x1 + 20 + 30;
        x1 = x1 + 30 + 20;
        options.push_back(new BackOption(ofVec2f(x1,y1),40, "DISCARD",entryPoint,COLORSCHEME_MENU_ORANGE));
        onMainLevel = false;onContourMode = false;
        
    }
public:
    
    void setToken(Token * value){token = value;}
    bool getIsOnTable(){return isOnTable;}
    void setIsOnTable(bool value){isOnTable = value;}
    
    
    OnTableContentCreationUI(Token * token_, ofVec3f * point_, LockableVector<Shape3D* > * shapes_, LockableVector<ContainerToken* > * containers_){
        entryPoint = point_;
        token = token_;
        shapes = shapes_;
        containers = containers_;
        radius = 200;
        isOnTable = true;
        setupMain();
        onMainLevel = true;
        update();
        onContourMode = false;
        textureCircle.loadImage("images/textureCircleMenu.png");
    }
    
    
    bool insideProtectedOptions(float x, float y){
        float center = (2* 100)/2;
        float x1 = token->getX()*ofGetWidth();
        float y1 = token->getY()*ofGetHeight() - 200;
        float radius = 150;
        if( ofVec2f(x,y).distance(ofVec2f(x1,y1)) < radius){
            return true;
        };
        return false;
        
    }
    void update(){
        float center = (options.size()* 100)/2;
        float x1 = token->getX()*ofGetWidth() - center;
        float y1;
        
        if(onMainLevel){y1 =token->getY()*ofGetHeight() - 100;}
        else{y1 =token->getY()*ofGetHeight() - 200;
        }
        for(int i = 0; i < options.size(); i++){
            x1 = x1 + 30 + 20;
            options[i]->center = ofVec2f(x1,y1);
            x1 = x1 + 20 +30;
        }
    }
    
    void draw() {
        ofPushStyle();
        if(onContourMode){
            float x1 = token->getX()*ofGetWidth();
            float y1 = token->getY()*ofGetHeight() - 200;
            float radius = 150;
            ofNoFill();
            ofCircle(x1, y1, radius);
            ofPushMatrix();
            ofTranslate(x1,y1);
            ofSetColor(COLORSCHEME_YELLOW);
            textureCircle.setAnchorPercent(0.5,0.5);
            textureCircle.draw(0,0,300,300);
            ofPopMatrix();
            ofFill();
        }
        
        for(int i = 0; i < options.size(); i++){
            ofSetColor(COLORSCHEME_LIGHTGREY);
            ofLine(ofVec2f(token->getX()*ofGetWidth(),token->getY()*ofGetHeight()), options[i]->center);
            options[i]->draw();
        }
        ofPopStyle();
    }
    
    std::pair<bool, OnScreenOption *> checkHit(float x, float y){
        for(int i = 0; i < options.size(); i++){
            if(options[i]->checkHit(x, y)){
                STLOption *d1 = dynamic_cast<STLOption*>(options[i]);
                if((bool)d1){
                    options.clear();
                    setupSTLSubOptions();
                    update();
                    return pair<bool,OnScreenOption*>(true, d1);
                }else{
                    STLSubOptions *d2 = dynamic_cast<STLSubOptions*>(options[i]);
                    if((bool)d2){
                        options.clear();
                        setupMain(); 
                        update();
                        return pair<bool,OnScreenOption*>(true, d2);
                    }
                    else{
                        KinectOption *d3 = dynamic_cast<KinectOption*>(options[i]);
                        if((bool)d3){
                            options.clear();
                            setupKinectOptions(); 
                            update();
                            return pair<bool,OnScreenOption*>(true, d3);
                        }else{
                            CopyOption *d4 = dynamic_cast<CopyOption*>(options[i]);
                            if((bool)d4){
                                options.clear();
                                setupCopyContainerOptions(); 
                                update();
                                return pair<bool,CopyOption*>(true, d4);
                            }else{
                                CopyContainerOption *d5 = dynamic_cast<CopyContainerOption*>(options[i]);
                                if((bool)d5){
                                    options.clear();
                                    setupMain(); 
                                    update();
                                    return pair<bool,CopyContainerOption*>(true, d5);
                                }else {
                                    ContourOption *d6 = dynamic_cast<ContourOption*>(options[i]);
                                    if((bool)d6){
                                        options.clear();
                                        setupContourOptions();
                                        update();
                                        return pair<bool,ContourOption*>(true, d6);
                                    }else{
                                        BackOption *d7 = dynamic_cast<BackOption*>(options[i]);
                                        if((bool)d7){
                                            options.clear();
                                            setupMain();
                                            update();
                                            return pair<bool,BackOption*>(true, d7);
                                        }else{
                                            SaveKinectOption *d8 = dynamic_cast<SaveKinectOption*>(options[i]);
                                            if((bool)d8){
                                                options.clear();
                                                setupMain();
                                                update();
                                                return pair<bool,SaveKinectOption*>(true, d8);
                                            }
                                            else{
                                                SaveContourOption *d9 = dynamic_cast<SaveContourOption*>(options[i]);
                                                if((bool)d9){
                                                    options.clear();
                                                    setupMain(); update();
                                                    return pair<bool,SaveContourOption*>(true, d9);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            
                        }
                    }
                }
                return pair<bool,OnScreenOption*>(true, NULL);
            }
        }
        return pair<bool,OnScreenOption*>(false, NULL);
    }
    
};


#endif
