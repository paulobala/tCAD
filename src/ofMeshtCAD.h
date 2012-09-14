#pragma once

#include "TCADTriangle.h"
#include "ofVec3f.h"
#include "ofMesh.h"
#include "ofRectangle.h"
#include "of3dUtils.h"
#include "ofGraphics.h"
#include "ofNode.h"

/*
 3D Mesh. Extends native ofMesh. Used for the representation of 3D Content
 */
class ofMeshtCAD : public ofMesh {
public:  
    
    ofMeshtCAD():ofMesh(){
    }
    void addFace(ofVec3f a, ofVec3f b, ofVec3f c);
    void addFace(ofVec3f a, ofVec3f b, ofVec3f c, ofVec3f d);
    void addFace(ofRectangle r);
    void addBox(ofRectangle r, float height);
    void translate(ofVec3f pos);
    void translate(float x, float y, float z);
    void rotate(float angle, ofVec3f axis);
    void scale(float x, float y, float z=1);
    void invertNormals();
    ofMeshtCAD &addMesh(ofMesh b);
    ofVec3f getCentroid();
    vector<TCADTriangle> getTriangles();
    void ofDrawAxis(float size);
    vector<float> ofLimits();
};
