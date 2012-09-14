#ifndef tCAD_ShapeVariables_h
#define tCAD_ShapeVariables_h

/*
 Variables for 3D Shapes
 */
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
