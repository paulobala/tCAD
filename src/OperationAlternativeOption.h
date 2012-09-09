//
//  TranslateAlternative.h
//  Carver
//
//  Created by paulobala on 11/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_TranslateAlternative_h
#define Carver_TranslateAlternative_h
#include "lineIntersection.h"

class OperationAlternativeOption {
    ofVec2f nw, ne, sw, se;
    ofPath path;
    unsigned long clickTime; 
    unsigned long referenceValueTime;
    bool acceptNewReference;
public:
  
    enum SHAPETYPE{
    SQUARE, HALFCIRCLEUP, HALFCIRCLELEFT
    };
   
    ofColor color;
    SHAPETYPE shape;
    float limitTime;
    float referenceValue;
    
    OperationAlternativeOption(ofVec2f nw_, ofVec2f ne_, ofVec2f sw_, ofVec2f se_){
        nw = nw_; ne = ne_; sw = sw_; se = se_;
        color = ofColor(ofColor::gray);
        referenceValue = ofGetElapsedTimeMillis();
        acceptNewReference = true;
        shape = SQUARE;
        limitTime = 1000;
    } 
    
    void updateReferenceValue(float z){
        if(acceptNewReference){
            referenceValue = z;
            referenceValueTime = ofGetElapsedTimeMillis();
            acceptNewReference= false;
        }
        else{
            if(ofGetElapsedTimeMillis() - referenceValueTime > 2000){
                referenceValue = z;
                referenceValueTime = ofGetElapsedTimeMillis();
            }else{
                referenceValueTime = ofGetElapsedTimeMillis();  
            }
        }
    }
    
    
    void update( ofVec2f nw_, ofVec2f ne_, ofVec2f se_, ofVec2f sw_){
        nw = nw_; ne = ne_; sw = sw_; se = se_;
        path.clear();
        path.setColor(color);
        path.lineTo(nw);
        path.lineTo(ne);
        path.lineTo(se);
        path.lineTo(sw);
        path.close();
    }

    void draw(ofImage img_){
        
        switch(shape){
            case SQUARE:{
                ofPushStyle();
                
                if(ofGetElapsedTimeMillis() - clickTime < limitTime){
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difb);
                    
                    path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }else {
                    path.setColor(color);
                }
                path.draw();
                
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                if(ofGetElapsedTimeMillis() - clickTime < limitTime){
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difb);
                    
                    ofSetColor(ofColor(ofClamp(color.r+newr, 0, 255), ofClamp(color.g+newg, 0, 255),ofClamp(color.b+newb, 0, 255),color.a));
                }else { 
                    ofSetColor(ofColor::white,color.a);
                }
                
                img_.setAnchorPercent(0.5, 0.5); 
                img_.draw(intersection.x, intersection.y);
                
                ofPopStyle();

                
                break;}
            case HALFCIRCLEUP:{
                
                ofPushStyle();
                
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                
                path.clear();
                path.moveTo((se+sw)/2);
                ofVec2f circleline = (se+sw)/2;
                circleline.x = circleline.x + 200;
                path.lineTo(circleline);
                for (int i = 0; i < 10; i++) {
                    ofVec2f tempp= circleline.rotate(-18,(se+sw)/2)
                    ;
                    path.lineTo(tempp);
                }
                path.close();
                
               
                
                if(ofGetElapsedTimeMillis() - clickTime < limitTime){
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difb);
                    
               
                    
                    path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }else {
                   path.setColor(color);
                }
                
                path.draw();
                
                if(ofGetElapsedTimeMillis() - clickTime < limitTime){
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difb);
                    
                    ofSetColor(ofColor(ofClamp(color.r+newr, 0, 255), ofClamp(color.g+newg, 0, 255),ofClamp(color.b+newb, 0, 255)));
                }else { 
                    ofSetColor(ofColor::white);
                    
                }
                
                img_.setAnchorPercent(0.5, 0.5); 
                img_.draw(intersection.x, intersection.y);
                
                ofPopStyle();

                break;}
            case HALFCIRCLELEFT:{
                
                ofPushStyle();
                
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                
                path.clear();
                path.moveTo((sw+nw)/2);
                ofVec2f circleline = (sw+nw)/2;
                circleline.y = circleline.y + 200;
                path.lineTo(circleline);
                for (int i = 0; i < 10; i++) {
                    ofVec2f tempp= circleline.rotate(-18,(sw+nw)/2)
                    ;
                    path.lineTo(tempp);
                }
                path.close();
                
                
                
                if(ofGetElapsedTimeMillis() - clickTime < limitTime){
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difb);
                    
                    
                    
                    path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }else {
                    path.setColor(color);
                }
                
                path.draw();
                
                if(ofGetElapsedTimeMillis() - clickTime < limitTime){
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difb);
                    
                    ofSetColor(ofColor(ofClamp(color.r+newr, 0, 255), ofClamp(color.g+newg, 0, 255),ofClamp(color.b+newb, 0, 255)));
                }else { 
                    ofSetColor(ofColor::white);
                    
                }
                img_.setAnchorPercent(0.5, 0.5); 
                img_.draw(intersection.x + 30, intersection.y);
                
                ofPopStyle();
                
                break;}
            default:break;
        }
        
        
               
    }
    void draw(){
        
        switch(shape){
            case SQUARE:{
                ofPushStyle();
                
                if(ofGetElapsedTimeMillis() - clickTime < limitTime){
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difb);
                    
                    path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }else {
                    path.setColor(color);
                }
                path.draw();
                
                ofPopStyle();
                
                
                break;}
            case HALFCIRCLEUP:{
                
                ofPushStyle();
                
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                
                path.clear();
                path.moveTo((se+sw)/2);
                ofVec2f circleline = (se+sw)/2;
                circleline.x = circleline.x + 200;
                path.lineTo(circleline);
                for (int i = 0; i < 10; i++) {
                    ofVec2f tempp= circleline.rotate(-18,(se+sw)/2)
                    ;
                    path.lineTo(tempp);
                }
                path.close();
                
                
                
                if(ofGetElapsedTimeMillis() - clickTime < limitTime){
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difb);
                    
                    
                    
                    path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }else {
                    path.setColor(color);
                }
                
                path.draw();
                
                
                ofPopStyle();
                
                break;}
            case HALFCIRCLELEFT:{
                
                ofPushStyle();
                
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                
                path.clear();
                path.moveTo((sw+nw)/2);
                ofVec2f circleline = (sw+nw)/2;
                circleline.y = circleline.y + 200;
                path.lineTo(circleline);
                for (int i = 0; i < 10; i++) {
                    ofVec2f tempp= circleline.rotate(-18,(sw+nw)/2)
                    ;
                    path.lineTo(tempp);
                }
                path.close();
                
                
                
                if(ofGetElapsedTimeMillis() - clickTime < limitTime){
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, limitTime, 0, difb);
                    
                    
                    
                    path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }else {
                    path.setColor(color);
                }
                
                path.draw();
                
                
                ofPopStyle();
                
                break;}
            default:break;
        }
    }
    
    bool inside(float x, float y){
        switch(shape){
            case SQUARE:{ 
                
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

                
              }  break;
            case HALFCIRCLEUP: 
            {
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                
                
                ofPolyline polyline;
                polyline.addVertex((se+sw)/2);
                ofVec2f circleline = (se+sw)/2;
                circleline.x = circleline.x + 200;
                polyline.addVertex(circleline);
                for (int i = 0; i < 10; i++) {
                    ofVec2f tempp= circleline.rotate(-18,(se+sw)/2)
                    ;
                    polyline.addVertex(tempp);
                }
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
                break;
            case HALFCIRCLELEFT: {
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                
                
                ofPolyline polyline;
                polyline.addVertex((sw+nw)/2);
                ofVec2f circleline = (sw+nw)/2;
                circleline.y = circleline.y + 200;
                polyline.addVertex(circleline);
                for (int i = 0; i < 10; i++) {
                    ofVec2f tempp= circleline.rotate(-18,(sw+nw)/2)
                    ;
                    polyline.addVertex(tempp);
                }
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

                break;    }
            default: break;
        }
        return false;       
    }
    
    void action(){
        clickTime = ofGetElapsedTimeMillis();
    }

    bool isWaiting(){
        if(ofGetElapsedTimeMillis() - clickTime < 1000){
            return true;
        }else {
            return false;
        }
    }
};


#endif
