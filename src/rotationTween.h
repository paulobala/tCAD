//
//  rotationTween.h
//  Carver
//
//  Created by paulobala on 07/06/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_rotationTween_h
#define Carver_rotationTween_h

class RotationTween {
public: 
    int x, y, z,speed;
    
    RotationTween(int x_,  int y_, int z_, int speed_){
        x = x_; 
        y = y_; 
        z = z_;
        speed = speed_;
    }
};


#endif
