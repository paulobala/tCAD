//
//  ShapeVariables.h
//  Carver
//
//  Created by paulobala on 13/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_ShapeVariables_h
#define Carver_ShapeVariables_h

class ShapeVariables{
public:
float rotateX;
float rotateY;
float rotateZ;
float scaleX;
float scaleY;
float scaleZ;
float scaleXYZ;
float translateX;
float translateY;
float translateZ;
    
    ShapeVariables(){
        rotateX = 0;
        rotateY= 0;
        rotateZ= 0;
        scaleX= 1;
        scaleY= 1;
        scaleZ= 1;
        scaleXYZ = 1;
        translateX= 0;
        translateY= 0;
        translateZ= 0;
    } 
};
#endif
