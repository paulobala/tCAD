#ifndef tCAD_shreder_h
#define tCAD_shreder_h
#include "Token.h"
/*
 Represents shredder token
 */
class ShredderToken
{
    Token * token;
public:
    Token * getToken()
    {
        return token;
    }
    
    /*
     Constructor
     */
    ShredderToken(Token * token_)
    {
        token = token_;
    }
    
    /*
     Is point above token?
     */
    bool inside(float x, float y)
    {
        ofVec2f center = ofVec2f(token->getX()*ofGetWidth(), token->getY()*ofGetHeight());
        
        if(center.distance(ofVec2f(x,y)) < 100)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
};

#endif

