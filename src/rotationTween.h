#ifndef tCAD_rotationTween_h
#define tCAD_rotationTween_h

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
