#ifndef tCAD_SaveToken_h
#define tCAD_SaveToken_h
#include "ColorScheme.h"
#include "Token.h"
/*
 Represents Save token
 */
class SaveToken
{
    Token * token;
    ofVec2f nw, ne, se, sw, center;//token edges
    bool isOnTable;
    ofxSTLExporter stlExporter;//exports 3D shapes into a STL file
    LockableVector<Shape3D* > * shapes;//3D shapes
    unsigned long clickTime;//timer
    ofColor color;
    
public:
    bool getIsOnTable()
    {
        return isOnTable;
    }
    void setIsOnTable(bool value)
    {
        isOnTable = value;
    }
    void setToken(Token * value)
    {
        token = value;
    }
    Token * getToken()
    {
        return token;
    }
    
    /*
     Constructor
     */
    SaveToken(Token * token_,  LockableVector<Shape3D* > * shapes_)
    {
        token = token_;
        shapes = shapes_;
        clickTime = ofGetElapsedTimeMillis();
        color = COLORSCHEME_LIGHTGREY;
        isOnTable = false;
    }
    
    /*
     Is point over token
     */
    bool insideToken(float x, float y)
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
    /*
     Update token edges
     */
    void update()
    {
        if(isOnTable)
        {
            float x = token->getX();
            float y = token->getY();
            float angle = -token->getAngle();
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
        }
    }
    /*
     Token was pressed on
     */
    void action()
    {
        if(ofGetElapsedTimeMillis() - clickTime < 1000)
        {
            //prevents double "clicking"
        }
        else
        {
            clickTime = ofGetElapsedTimeMillis();
            stlExporter.beginModel("Tese");
            vector<Shape3D*> temp = shapes->getObjects();
            shapes->lockVector();
            for (std::vector<Shape3D*>::iterator it=temp.begin() ; it < temp.end(); it++ )
            {
                std::vector<ofxSTLFacet>* facets = new std::vector<ofxSTLFacet>();
                (*it)->getFacets(facets);
                for (std::vector<ofxSTLFacet>::iterator it2 = facets->begin() ; it2 < facets->end(); it2++ )
                {
                    stlExporter.addTriangle((*it2).vert1, (*it2).vert2, (*it2).vert3, (*it2).normal);
                }
            }
            shapes->unlockVector();
            stlExporter.useASCIIFormat(true); //export as ASCII (default is binary)
            string save = ofGetTimestampString();
            save.append("ASCII.stl");
            string savebin = ofGetTimestampString();
            savebin.append("Bin.stl");
            stlExporter.saveModel(save);
            stlExporter.useASCIIFormat(false); //export as binary
            stlExporter.saveModel(savebin);
        }
    }
    /*
     
     */
    void draw()
    {
        if(isOnTable)
        {
            ofPath p;
            p.clear();
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
                p.setColor(color);
            }
            p.lineTo(nw);
            p.lineTo(ne);
            p.lineTo(se);
            p.lineTo(sw);
            p.close();
            p.draw();
        }
    }
    
    
};

#endif
