//
//  StlSecondMenuOption.h
//  Carver
//
//  Created by paulobala on 04/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_StlSecondMenuOption_h
#define Carver_StlSecondMenuOption_h
#include "lineIntersection.h"


class OnTokenOption{ 
    ofImage stlImg, kinectImg, contourImg, copyImg, leftArrow, rightArrow;
    ofVec2f edge1, edge2, center;
    ofPath p;
    float angle;
public:
    
    enum TriangleType{
        STL, KINECT, CONTOUR, COPY
    };
    
    ofColor color;
    TriangleType triType;
    
    
    OnTokenOption( ofVec2f edge1_, ofVec2f edge2_, ofVec2f center_, TriangleType triType_){
        edge1 = edge1_; edge2 = edge2_; center = center_; triType = triType_;
        color = ofColor(ofRandom(0,255), ofRandom(0,255), ofRandom(0,255));
        angle = 0;
        
        stlImg.loadImage("images/box.png");
        kinectImg.loadImage("images/kinect.png");
        contourImg.loadImage("images/contour.png");
        copyImg.loadImage("images/copy.png");
        leftArrow.loadImage("images/leftarrow.png");
        rightArrow.loadImage("images/rightarrow.png");
    } 
    
    void update( ofVec2f edge1_, ofVec2f edge2_, ofVec2f center_, float angle_){
        edge1 = edge1_; edge2 = edge2_; center = center_;
        p.clear();
        p.lineTo(edge1);
        p.lineTo(edge2);
        p.lineTo(center);
        p.close();
        angle = angle_;
    }
    
    void draw(){
        ofPushStyle();
        p.setColor(color);
        p.draw();
        ofVec2f point1 = (edge1+center)/2;
        ofVec2f point2 = (edge2+center)/2;
        LineSegment line1 = LineSegment(point1, edge2);
        LineSegment line2 = LineSegment(point2, edge1);
        ofVec2f intersectionp;
        line1.Intersect(line2, intersectionp);
        
        ofVec2f point3= (edge1+intersectionp)/2;
        ofVec2f point4 = (edge2+intersectionp)/2;
        LineSegment line3 = LineSegment(point3, edge2);
        LineSegment line4 = LineSegment(point4, edge1);
        ofVec2f intersection;
        line3.Intersect(line4, intersection);
        
        
        ofPushMatrix();
        ofTranslate(intersection.x,intersection.y, 0);
        ofRotate(ofRadToDeg(angle+PI));
        
        
        switch(triType){
            case STL:{
                stlImg.setAnchorPercent(0.5, 0.5);
                stlImg.draw(0,0 ,30,30);
                break;}
            case KINECT:{
                kinectImg.setAnchorPercent(0.5, 0.5);
                kinectImg.draw(0,0 ,30,30);
                break;}
            case CONTOUR:{
                contourImg.setAnchorPercent(0.5, 0.5);
                contourImg.draw(0,0 ,30,30);
                break;}
            case COPY:{
                copyImg.setAnchorPercent(0.5, 0.5);
                copyImg.draw(0,0,30,30);
                break;}
            default:break;
        }
        ofPopMatrix();
        
        ofPopStyle();
    }
    
    bool inside(float x, float y){
        ofPolyline polyline;
        polyline.addVertex(edge1.x, edge1.y);
        polyline.addVertex(edge2.x, edge2.y);
        polyline.addVertex(center.x, center.y);
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
    
    void action(){
        color = ofColor(ofRandom(0,255), ofRandom(0,255), ofRandom(0,255));
    }
    
    
};
#endif
