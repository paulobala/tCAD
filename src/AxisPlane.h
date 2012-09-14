#ifndef tCAD_AxisPlane_h
#define tCAD_AxisPlane_h

/*
 Represents Plane formed by 2 axes in 3D space.
 */
class AxisPlane{
public:
    enum AXIS{
        X_Z,X_Y,Y_Z,NOAXIS
    };
    
    AXIS axis;
    AXIS axisPrevious;
    
    /*
     Constructor
     */
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
