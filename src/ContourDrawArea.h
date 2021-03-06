#ifndef ModellerCarve_polygonDrawArea_h
#define ModellerCarve_polygonDrawArea_h

#include "lockableVector.h"

class ContourDrawArea{
protected:
    ofVec3f* entryPoint;
    LockableVector<Shape3D*> * shapes;//vector of shapes
    ofPolyline contour;
    vector<ofPoint> verts;//current points in contour
    
    ofPolyline deliverContour(){
        contour.close();
        contour = contour.getSmoothed(5,1);
        contour.simplify();
        return contour; 
    }
    
public:
    float x,y,width,height;//position and size of draw area
    bool active;
    /*
     Constructor
     */
    ContourDrawArea(){
    }
    /*
     Constructor
     */
    ContourDrawArea(ofVec3f* entryPoint_, LockableVector<Shape3D*> * shapes_, float x_, float y_, float width_, float height_){
        entryPoint = entryPoint_;
        shapes = shapes_;
        x = x_;
        y= y_; 
        width = width_; 
        height= height_;
        active = false;
    }
    
    /*
     Erase contour
     */
    void clearDrawArea(){
        verts.clear();
        contour.clear();
    }
    /*
     Inside draw area
     */
    bool inside(float x_, float y_)
    {
        if( x_ > x && y_ > y && x_ < x + width && y_ < y + height ){
            return true;
        }
        return false;
        
    }
    /*
     Draw contour area
     */
    void draw()
    {
        ofPushStyle();
        ofSetColor(0, 0, 0);
        ofRect(x,y,width,height);
        
        ofPolyline temp;
        
        ofSetColor(255, 255);
        
        temp.addVertexes(verts);
        temp.draw();
        
        ofSetColor(255, 255, 255);
        
        contour.draw();
        ofPopStyle();
        
    }
    /*
     Add vertice to contour
     */
    void addVert(Finger * finger){
        
        if(inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight())){
            verts.push_back(ofPoint(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()));    
        }
        
    }
    /*
     Erase contour
     */
    void erase(float x_, float y_, float size){
        for(vector<ofPoint>::iterator it=verts.begin() ; it < verts.end(); it++ ){
            ofVec3f sur = (*it);
            if(ofVec2f(sur.x, sur.y).distance(ofVec2f(x_,y_)) < size){
                verts.erase(it);
            }
        }
    }
    /*
     Convert contour into 3D shape
     */
    void save(){
        for(int i = 0; i < verts.size(); i++){
            verts.at(i).y = ofGetHeight() - verts.at(i).y;
        }
        
        contour.addVertexes(verts);
        contour.close();
        verts.clear();
        Basic3DObjectFromContour * ste =new Basic3DObjectFromContour( entryPoint, deliverContour());
        ofColor * tempColor = new ofColor(0,0,0);
        tempColor->setHsb(ofRandom(0,255), ofRandom(180,255), ofRandom(180,255));
        ste->changeColor(tempColor);
        shapes->lockVector(); 
        shapes->addElement(ste);
        shapes->unlockVector();
        active = false;
    }
};

#endif
