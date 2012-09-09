//
//  DashedLine.h
//  Carver
//
//  Created by paulobala on 05/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_DashedLine_h
#define Carver_DashedLine_h

class DashedLine{
    ofVec2f point1;
    ofVec2f point2;
public:

    DashedLine(ofVec2f point1_ , ofVec2f point2_){
        point1 = point1_;
        point2 = point2_;
    }
    
    void draw()
    {
        glEnable (GL_LINE_STIPPLE);
        glLineStipple (1, 0x00FF); /* dashed */
        ofLine(point1, point2);
        glDisable (GL_LINE_STIPPLE);
    }
};

#endif
