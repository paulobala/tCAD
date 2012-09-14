
#ifndef ModellerCarve_Basic3DObjectBlank_h
#define ModellerCarve_Basic3DObjectBlank_h
#include "Basic3DObject.h"

class Basic3DObjectBlank: public Basic3DObject {
        
public:
    Basic3DObjectBlank():Basic3DObject()
    {
        mesh.setMode(OF_PRIMITIVE_TRIANGLES);
    }
    
};


#endif
