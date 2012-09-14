#ifndef tCAD_Dash_h
#define tCAD_Dash_h

/*
Visual element. Dash anything by putting it between the begin and end method invocation
 */
class Dash {
public:
    Dash(){}
    
    void begin(){
        glEnable (GL_LINE_STIPPLE);
        glLineStipple (1, 0x00FF); 
    }
    
    void end(){
        glDisable (GL_LINE_STIPPLE);
    }
   
};

#endif
