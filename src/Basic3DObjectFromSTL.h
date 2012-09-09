#ifndef Modeller_Basic3DObjectFromSTL_h
#define Modeller_Basic3DObjectFromSTL_h
#include "Basic3DObject.h"
#include "ofxSTL.h"
#include "ofEasyFingerCam.h"

class Basic3DObjectFromSTL: public Basic3DObject{
public:  
       
   Basic3DObjectFromSTL(string path,ofVec3f entryPoint):Basic3DObject(){
    
       ofxSTLImporter stlImporter;
       stlImporter.loadSTL(path);
       
       vector<ofxSTLFacet> facets = stlImporter.getFacets();
       mesh.setMode(OF_PRIMITIVE_TRIANGLES);
       
       for (vector<ofxSTLFacet>::iterator it=facets.begin() ; it < facets.end(); it++ ){
           mesh.addFace((*it).vert1, (*it).vert2, (*it).vert3);
       }
       mesh.invertNormals();
       
       ofVec3f centroid = mesh.getCentroid();
       centroid = entryPoint - centroid;
       mesh.translate(centroid);
       
    }
    
       
};


#endif
