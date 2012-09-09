//
//  panningTween.h
//  Carver
//
//  Created by paulobala on 18/07/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_panningTween_h
#define Carver_panningTween_h

class PanningTween {
public: 
    ofVec3f pan;
    
    PanningTween(ofVec3f pan_){
        pan = pan_;
    }
};

#endif
