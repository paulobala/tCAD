//
//  ThirdMenuOption.h
//  Carver
//
//  Created by paulobala on 05/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_ThirdMenuOption_h
#define Carver_ThirdMenuOption_h

#include "ofMain.h"
#include "ColorScheme.h"

class InAirOption
{
    ofImage stlImg, kinectImg, contourImg, copyImg, leftArrow, rightArrow, okayImg, notOkayImg;
    
    ofPath p;
    unsigned long clickTime;
    float angle;
    ofTrueTypeFont font;
    
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
    ofVec2f nw, ne, sw, se;
    string name;
    ofImage shapeImg;
    enum FUNCTION
    {
        LEFTARROWMAINMENU, RIGHTARROWMAINMENU, MAINMENUENTER, LEFTARROWSTL, RIGHTARROWSTL, STLENTER,
        LEFTARROWCOPY, RIGHTARROWCOPY, COPYENTER, KINECTSAVE, KINECTDISCARD, CONTOURSAVE, CONTOURDISCARD,
        BACK, NOFUNCTION
    };
    
    ofColor color;
    
    FUNCTION function;
    
    CopyContainerOption * copyCont;
    
    InAirOption( ofVec2f anw, ofVec2f ane, ofVec2f asw, ofVec2f ase, float aangle)
    {
        nw = anw;
        ne = ane;
        sw = asw;
        se = ase;
        color = ofColor(COLORSCHEME_GREY);
        function = NOFUNCTION;
        angle = aangle;
        ofTrueTypeFont::setGlobalDpi(72);
        font.loadFont("fonts/helveticaNeue.ttf", 14, true, true);
        name = "";
        stlImg.loadImage("images/box.png");
        kinectImg.loadImage("images/kinect.png");
        contourImg.loadImage("images/contour.png");
        copyImg.loadImage("images/copy.png");
        leftArrow.loadImage("images/leftarrow.png");
        rightArrow.loadImage("images/rightarrow.png");
        shapeImg.loadImage("images/box.png");
        okayImg.loadImage("images/okay.png");
        notOkayImg.loadImage("images/notokay.png");
    }
    
    void update(ofVec2f anw, ofVec2f ane, ofVec2f ase, ofVec2f asw, float aangle)
    {
        nw = anw;
        ne = ane;
        sw = asw;
        se = ase;
        angle = aangle;
        p.clear();
        p.lineTo(nw);
        p.lineTo(ne);
        p.lineTo(se);
        p.lineTo(sw);
        p.close();
    }
    
    void draw()
    {
        switch(function)
        {
            case LEFTARROWMAINMENU:
            {
                ofPushStyle();
                if(ofGetElapsedTimeMillis() - clickTime < 1000)
                {
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
                    p.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }
                else
                {
                    p.setColor(COLORSCHEME_GREY);
                }
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                leftArrow.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                leftArrow.draw(0,0,40,40);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case RIGHTARROWMAINMENU:
            {
                ofPushStyle();
                if(ofGetElapsedTimeMillis() - clickTime < 1000)
                {
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
                    p.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }
                else
                {
                    p.setColor(COLORSCHEME_GREY);
                }
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                rightArrow.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                rightArrow.draw(0,0,30,30);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case MAINMENUENTER:
            {
                ofPushStyle();
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                if(name == "STL")
                {
                    p.setColor(COLORSCHEME_MENU_GREEN);
                    p.draw();
                    stlImg.setAnchorPercent(0.5, 0.5);
                    ofPushMatrix();
                    ofTranslate(intersection.x,intersection.y, 0);
                    ofRotate(ofRadToDeg(angle));
                    stlImg.draw(0,0,40,40);
                    ofSetColor(ofColor::white);
                    font.drawString(name, -8,-25);
                    ofPopMatrix();
                }
                else if(name == "KINECT")
                {
                    p.setColor(COLORSCHEME_MENU_BLUE);
                    p.draw();
                    kinectImg.setAnchorPercent(0.5, 0.5);
                    ofPushMatrix();
                    ofTranslate(intersection.x,intersection.y, 0);
                    ofRotate(ofRadToDeg(angle));
                    kinectImg.draw(0,0,40,40);
                    ofSetColor(ofColor::white);
                    font.drawString(name,-18,-25);
                    ofPopMatrix();
                }
                else if(name == "COPY")
                {
                    p.setColor(COLORSCHEME_MENU_ORANGE);
                    p.draw();
                    copyImg.setAnchorPercent(0.5, 0.5);
                    ofPushMatrix();
                    ofTranslate(intersection.x,intersection.y, 0);
                    ofRotate(ofRadToDeg(angle));
                    copyImg.draw(0,0,40,40);
                    ofSetColor(ofColor::white);
                    font.drawString(name, -15,-25);
                    ofPopMatrix();
                }
                else if(name == "CONTOUR")
                {
                    p.setColor(COLORSCHEME_MENU_YELLOW);
                    p.draw();
                    contourImg.setAnchorPercent(0.5, 0.5);
                    ofPushMatrix();
                    ofTranslate(intersection.x,intersection.y, 0);
                    ofRotate(ofRadToDeg(angle));
                    contourImg.draw(0,0,40,40);
                    ofSetColor(COLORSCHEME_DARKGREY);
                    font.drawString(name, -25,-25);
                    ofPopMatrix();
                }
                ofPopStyle();
                break;
            }
            case LEFTARROWSTL:
            {
                ofPushStyle();
                if(ofGetElapsedTimeMillis() - clickTime < 1000)
                {
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
                    p.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }
                else
                {
                    p.setColor(COLORSCHEME_GREY);
                }
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                leftArrow.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                leftArrow.draw(0,0,30,30);
                name = "";
                ofSetColor(ofColor::white);
                font.drawString(name, 0,0);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case RIGHTARROWSTL:
            {
                ofPushStyle();
                if(ofGetElapsedTimeMillis() - clickTime < 1000)
                {
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
                    p.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }
                else
                {
                    p.setColor(COLORSCHEME_GREY);
                }
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                rightArrow.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                rightArrow.draw(0,0,30,30);
                name = "";
                ofSetColor(ofColor::white);
                font.drawString(name, 0,0);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case STLENTER:
            {
                ofPushStyle();
                p.setColor(color);
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                ofSetColor(ofColor::white);
                shapeImg.setAnchorPercent(0.5, 0.5);
                shapeImg.draw(0,0);
                ofSetColor(COLORSCHEME_TEXT_WHITE);
                font.drawString(name, -20 ,-30);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case  LEFTARROWCOPY:
            {
                ofPushStyle();
                if(ofGetElapsedTimeMillis() - clickTime < 1000)
                {
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
                    p.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }
                else
                {
                    p.setColor(COLORSCHEME_GREY);
                }
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                leftArrow.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                leftArrow.draw(0,0,30,30);
                name = "";
                ofSetColor(ofColor::white);
                font.drawString(name, 0,0);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case  RIGHTARROWCOPY:
            {
                ofPushStyle();
                if(ofGetElapsedTimeMillis() - clickTime < 1000)
                {
                    float difr = 255-color.r;
                    float difg = 255-color.g;
                    float difb = 255-color.b;
                    float newr = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difr);
                    float newg = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difg);
                    float newb = ofMap(ofGetElapsedTimeMillis() - clickTime, 0, 1000, 0, difb);
                    p.setColor(ofColor(ofClamp(255-newr, 0, 255),ofClamp(255-newg, 0, 255),ofClamp(255-newb, 0, 255),color.a));//white to color
                }
                else
                {
                    p.setColor(COLORSCHEME_GREY);
                }
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                rightArrow.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                rightArrow.draw(0,0,30,30);
                name = "";
                ofSetColor(ofColor::white);
                font.drawString(name, 0,0);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case COPYENTER:
            {
                ofPushStyle();
                p.setColor(color);
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                if(copyCont!=NULL)
                {
                    if(copyCont->container->getIsOnTable())
                    {
                        int numOption = copyCont->container->links.size();
                        if(numOption == 0)
                        {
                            ofVec2f centroidMarker = ofVec2f(copyCont->container->getToken()->getX()*ofGetWidth(),
                                                             copyCont->container->getToken()->getY()*ofGetHeight());
                            ofSetColor(COLORSCHEME_DARKGREY);
                            ofSetLineWidth(4);
                            ofLine(intersection,centroidMarker);
                            ofPushMatrix();
                            ofTranslate(intersection.x,intersection.y, 0);
                            ofRotate(ofRadToDeg(angle));
                            name = "NOTHING TO COPY";
                            ofSetColor(COLORSCHEME_TEXT_WHITE);
                            font.drawString(name, -50,0);
                            ofPopMatrix();
                        }
                        else
                        {
                            float sizerect = nw.distance(ne)/(float)(numOption);
                            float distance = sizerect;
                            ofVec2f t1 = nw;
                            ofVec2f t2 = sw;
                            for(int i = 0; i < copyCont->container->links.size(); i++)
                            {
                                ofVec2f calc = calculate_line_point( nw.x, nw.y, ne.x, ne.y, distance);
                                ofVec2f calc2 = calculate_line_point( sw.x, sw.y, se.x, se.y, distance);
                                ofColor * coltemp = copyCont->container->links.at(i)->color;
                                ofPath temppath;
                                temppath.moveTo(t1);
                                temppath.lineTo(calc);
                                temppath.lineTo(calc2);
                                temppath.lineTo(t2);
                                temppath.close();
                                temppath.setColor(ofColor(coltemp->r,coltemp->g, coltemp->b));
                                temppath.draw();
                                distance = sizerect + distance;
                                t1 = calc;
                                t2 = calc2;
                            }
                            ofVec2f centroidMarker = ofVec2f(copyCont->container->getToken()->getX()*ofGetWidth(),
                                                             copyCont->container->getToken()->getY()*ofGetHeight());
                            ofSetColor(COLORSCHEME_DARKGREY);
                            ofSetLineWidth(4);
                            ofLine(intersection,centroidMarker);
                            ofPushMatrix();
                            ofTranslate(intersection.x,intersection.y, 0);
                            ofRotate(ofRadToDeg(angle));
                            name = "COPY";
                            ofSetColor(COLORSCHEME_TEXT_WHITE);
                            font.drawString(name, -15,0);
                            ofPopMatrix();
                        }
                    }
                    else
                    {
                        int numOption = copyCont->container->links.size();
                        if(numOption == 0)
                        {
                            ofPushMatrix();
                            ofTranslate(intersection.x,intersection.y, 0);
                            ofRotate(ofRadToDeg(angle));
                            name = "NO SHAPES TO COPY";
                            ofSetColor(COLORSCHEME_TEXT_WHITE);
                            font.drawString(name, -40,0);
                            ofPopMatrix();
                        }
                        else
                        {
                            float sizerect = nw.distance(ne)/(float)(numOption);
                            float distance = sizerect;
                            ofVec2f t1 = nw;
                            ofVec2f t2 = sw;
                            for(int i = 0; i < copyCont->container->links.size(); i++)
                            {
                                ofVec2f calc = calculate_line_point( nw.x, nw.y, ne.x, ne.y, distance);
                                ofVec2f calc2 = calculate_line_point( sw.x, sw.y, se.x, se.y, distance);
                                ofColor * coltemp = copyCont->container->links.at(i)->color;
                                ofPath temppath;
                                temppath.moveTo(t1);
                                temppath.lineTo(calc);
                                temppath.lineTo(calc2);
                                temppath.lineTo(t2);
                                temppath.close();
                                temppath.setColor(ofColor(coltemp->r,coltemp->g, coltemp->b));
                                temppath.draw();
                                distance = sizerect + distance;
                                t1 = calc;
                                t2 = calc2;
                            }
                            ofPushMatrix();
                            ofTranslate(intersection.x,intersection.y, 0);
                            ofRotate(ofRadToDeg(angle));
                            name = "COPY";
                            ofSetColor(COLORSCHEME_TEXT_WHITE);
                            font.drawString(name, -15,0);
                            ofPopMatrix();
                        }
                        ofPushMatrix();
                        ofTranslate(intersection.x,intersection.y, 0);
                        ofRotate(ofRadToDeg(angle));
                        name = "NOT ON TABLE";
                        ofSetColor(ofColor::white);
                        font.drawString(name, -35,30);
                        ofPopMatrix();
                    }
                }
                ofPopStyle();
                break;
            }
            case  KINECTSAVE:
            {
                ofPushStyle();
                p.setColor(COLORSCHEME_MENU_GREEN);
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                okayImg.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                okayImg.draw(0,0,30,30);
                name = "SAVE";
                ofSetColor(ofColor::white);
                font.drawString(name, -15,-15);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case KINECTDISCARD:
            {
                ofPushStyle();
                p.setColor(COLORSCHEME_MENU_ORANGE);
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                notOkayImg.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                notOkayImg.draw(0,0,30,30);
                name = "DISCARD";
                ofSetColor(ofColor::white);
                font.drawString(name, -25,-15);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case  CONTOURSAVE:
            {
                ofPushStyle();
                p.setColor(COLORSCHEME_MENU_GREEN);
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                okayImg.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                okayImg.draw(0,0,30,30);
                name = "SAVE";
                ofSetColor(ofColor::white);
                font.drawString(name,-15,-15);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case CONTOURDISCARD:
            {
                ofPushStyle();
                p.setColor(COLORSCHEME_MENU_ORANGE);
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                notOkayImg.setAnchorPercent(0.5, 0.5);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                notOkayImg.draw(0,0,30,30);
                name = "DISCARD";
                ofSetColor(ofColor::white);
                font.drawString(name, -25,-15);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case BACK:
            {
                ofPushStyle();
                p.setColor(COLORSCHEME_GREY);
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                //rightArrow.draw(0,0,30,30);
                name = "BACK";
                ofSetColor(ofColor::white);
                font.drawString(name, 0,0);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            case NOFUNCTION:
            {
                ofPushStyle();
                p.setColor(COLORSCHEME_GREY);
                p.draw();
                ofSetColor(ofColor::white);
                LineSegment line1 = LineSegment(nw,se);
                LineSegment line2 = LineSegment(ne,sw);
                ofVec2f intersection;
                line1.Intersect(line2, intersection);
                ofPushMatrix();
                ofTranslate(intersection.x,intersection.y, 0);
                ofRotate(ofRadToDeg(angle));
                //rightArrow.draw(0,0,30,30);
                name = "";
                ofSetColor(ofColor::white);
                font.drawString(name, 0,0);
                ofPopMatrix();
                ofPopStyle();
                break;
            }
            default:
                break;
        }
    }
    
    bool inside(float x, float y)
    {
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
        for (i=1; i<=N; i++)
        {
            p2 = polyline[i % N];
            if (y > MIN(p1.y,p2.y))
            {
                if (y <= MAX(p1.y,p2.y))
                {
                    if (x <= MAX(p1.x,p2.x))
                    {
                        if (p1.y != p2.y)
                        {
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
    
    void action()
    {
        // color = ofColor(ofRandom(0,255), ofRandom(0,255), ofRandom(0,255));
        clickTime = ofGetElapsedTimeMillis();
    }
    
    
};

#endif
