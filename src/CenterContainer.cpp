//
//  CenterContainer.cpp
//  Carver
//
//  Created by paulobala on 03/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#include <iostream>

#include "CenterContainer.h"
#include "lineIntersection.h"

CenterContainer::CenterContainer(  ofVec2f nw_, ofVec2f ne_, ofVec2f sw_, ofVec2f se_ ){
    nw = nw_; ne = ne_; sw = sw_; se = se_;
    color = ofColor(ofColor::gray);
} 

void CenterContainer::update( ofVec2f nw_, ofVec2f ne_, ofVec2f se_, ofVec2f sw_){
     nw = nw_; ne = ne_; sw = sw_; se = se_;
    path.clear();
    path.setColor(color);
    path.lineTo(nw);
    path.lineTo(ne);
    path.lineTo(se);
    path.lineTo(sw);
    path.close();
}

bool CenterContainer::isWaiting(){
    if(ofGetElapsedTimeMillis() - clickTime < 1000){
        return true;
    }else {
        return false;
    }
}

void CenterContainer::draw(ofImage img_){
    ofPushStyle();

    if(ofGetElapsedTimeMillis() - clickTime < 1000){
        float difr = 255-color.r;
        float difg = 255-color.g;
        float difb = 255-color.b;
        
        float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
        float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
        float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
        
        path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
    }else {
        path.setColor(color);
    }
    path.draw();
    
    LineSegment line1 = LineSegment(nw,se);
    LineSegment line2 = LineSegment(ne,sw);
    ofVec2f intersection;
    line1.Intersect(line2, intersection);
    if(ofGetElapsedTimeMillis() - clickTime < 1000){
        float difr = 255-color.r;
        float difg = 255-color.g;
        float difb = 255-color.b;
        float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
        float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
        float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
        
        ofSetColor(ofColor(ofClamp(color.r+newr, 0, 255), ofClamp(color.g+newg, 0, 255),ofClamp(color.b+newb, 0, 255),color.a));
        //color to white
    }else { 
        ofSetColor(ofColor::white,color.a);
        
    }
    
    img_.setAnchorPercent(0.5, 0.5); 
    img_.draw(intersection.x, intersection.y,30,30);
    
    ofPopStyle();
    
}
void CenterContainer::draw(){
    ofPushStyle();
    if(ofGetElapsedTimeMillis() - clickTime < 1000){
        float difr = 255-color.r;
        float difg = 255-color.g;
        float difb = 255-color.b;
        float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
        float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
        float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
        
        path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
    }else {
        path.setColor(color);
    }
    path.draw();
    
    ofPopStyle();
    
}

bool CenterContainer::inside(float x, float y){
    
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

void CenterContainer::action(){
     clickTime = ofGetElapsedTimeMillis();
}
