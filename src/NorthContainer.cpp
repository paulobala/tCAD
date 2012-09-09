
#include <iostream>
#include "NorthContainer.h"
#include "lineIntersection.h"
NorthContainer::NorthContainer(  ofVec2f nw_, ofVec2f ne_, ofVec2f sw_, ofVec2f se_ ){
     nw = nw_; ne = ne_; sw = sw_; se = se_;
    color = ofColor(ofColor::black);
} 

void NorthContainer::update( ofVec2f nw_, ofVec2f ne_, ofVec2f se_, ofVec2f sw_){
     nw = nw_; ne = ne_; sw = sw_; se = se_;
    path.clear();
    path.lineTo(nw);
    path.lineTo(ne);
    path.lineTo(se);
    path.lineTo(sw);
    path.close();
}

void NorthContainer::draw(ofImage img_){
    ofPushStyle();  
    if(ofGetElapsedTimeMillis() - clickTime < 1000){
        float difr = 255-color.r;
        float difg = 255-color.g;
        float difb = 255-color.b;
        float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
        float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
        float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
        
        path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newg, 0, 255),color.a));//white to color
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
    img_.draw(intersection.x, intersection.y,60,60);

    ofPopStyle();
}
void NorthContainer::draw(){
    ofPushStyle();
    if(ofGetElapsedTimeMillis() - clickTime < 1000){
        float difr = 255-color.r;
        float difg = 255-color.g;
        float difb = 255-color.b;
        float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
        float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
        float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
        
        path.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newg, 0, 255),color.a));//white to color
    }else {
        path.setColor(color);
    }
    path.draw();
    ofPopStyle();
}

bool NorthContainer::inside(float x, float y){
    
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

void NorthContainer::action(){
  clickTime = ofGetElapsedTimeMillis();
}
