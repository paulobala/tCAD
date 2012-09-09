#ifndef Carver_Dash_h
#define Carver_Dash_h

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
