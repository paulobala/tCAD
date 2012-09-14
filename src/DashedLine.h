#ifndef tCAD_DashedLine_h
#define tCAD_DashedLine_h

/*
Visual element - 2D Dashed Line
 */
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
