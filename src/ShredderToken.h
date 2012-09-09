//
//  shreder.h
//  Carver
//
//  Created by paulobala on 20/05/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_shreder_h
#define Carver_shreder_h
#include "Token.h"

class ShredderToken
{
    Token * token;
public:
    Token * getToken()
    {
        return token;
    }
    
    ShredderToken(Token * token_)
    {
        token = token_;
    }
    
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

