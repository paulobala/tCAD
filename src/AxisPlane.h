//
//  AxisPlane.h
//  Carver
//
//  Created by paulobala on 21/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_AxisPlane_h
#define Carver_AxisPlane_h

class AxisPlane{
public:
    enum AXIS{
        X_Z,X_Y,Y_Z,NOAXIS
    };
    
    AXIS axis;
    AXIS axisPrevious;
    
    AxisPlane(){
        axis = NOAXIS;
        axisPrevious = NOAXIS;
    }
    
    void changeAxis(AXIS axis_){
        axisPrevious = axis;
        axis = axis_;
    }
};


#endif
