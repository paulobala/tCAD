#ifndef ModellerCarve_Basic3DObjectFromCopy_h
#define ModellerCarve_Basic3DObjectFromCopy_h
#include "Basic3DObject.h"
#include "ofEasyFingerCam.h"

/*
 Copy Object
 */
class Basic3DObjectFromCopy: public Basic3DObject {
public:
    /*
     Constructor
     */
    Basic3DObjectFromCopy(ofVec3f entryPoint, ofMeshtCAD mesh_):Basic3DObject()
    {
        mesh.setMode(OF_PRIMITIVE_TRIANGLES);
        mesh.clear();
        mesh.addMesh(mesh_);
        ofVec3f centroid = mesh.getCentroid();
        centroid = entryPoint - centroid;
        mesh.translate(centroid);
    }
};

#endif
