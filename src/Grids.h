#ifndef tCAD_grids_h
#define tCAD_grids_h
#include "ColorScheme.h"

class Grids{
public:
    Grids(){}

    void ofDrawAxis(float size) {
        ofPushStyle();
        ofSetLineWidth(3);
        
        // draw x axis
        ofSetColor(ofColor::red);
        ofLine(0, 0, 0, size, 0, 0);
        
        // draw y axis
        ofSetColor(ofColor::green);
        ofLine(0, 0, 0, 0, size, 0);
        
        // draw z axis
        ofSetColor(ofColor::blue);
        ofLine(0, 0, 0, 0, 0, size);
        
        ofPopStyle();
    }
    void ofDrawGrid(float scale = 10.0f, float ticks = 8.0f, bool labels = false, bool x = true, bool y = true, bool z = true) {
        
        ofColor c(255,0,0);
        
        ofPushStyle();
        
        if (x) {
         c = COLORSCHEME_CYAN;
            
            ofSetColor(c);
            ofDrawGridPlane(scale, ticks, labels);
        }
        if (y) {
            c = COLORSCHEME_MAGENTA;
           
            ofSetColor(c);
            ofPushMatrix();
            ofRotate(90, 0, 0, -1);
            ofDrawGridPlane(scale, ticks, labels);
            ofPopMatrix();
        }
        if (z) {
            c = COLORSCHEME_YELLOW;   
            
            ofSetColor(c);
            ofPushMatrix();
            ofRotate(90, 0, 1, 0);
            ofDrawGridPlane(scale, ticks, labels);
            ofPopMatrix();
        }
        
        if (labels) {
            ofPushStyle();
            ofSetColor(255, 255, 255);
            float labelPos = scale * (1.0f + 0.5f / ticks);
            ofSetDrawBitmapMode(OF_BITMAPMODE_MODEL_BILLBOARD);
            ofDrawBitmapString("x", labelPos, 0, 0);
            ofDrawBitmapString("y", 0, labelPos, 0);
            ofDrawBitmapString("z", 0, 0, labelPos);
            ofPopStyle();
        }
        ofPopStyle();
    }
    
    void ofDrawGridPlane(float scale, float ticks = 8.0f, bool labels = false) {
        
        float minor = scale / ticks;
        float major =  minor * 2.0f;
        
        ofPushStyle();
        for (int iDimension=0; iDimension<2; iDimension++)
        {
            for (float yz=-scale; yz<=scale; yz+= minor)
            {bool jump = false;
                //major major
                if (fabs(yz) == scale)
                    ofSetLineWidth(2);
               else if(yz == 0)
               {ofSetLineWidth(0);jump = true;}
                //major
                else if (yz / major == floor(yz / major) )
                    ofSetLineWidth(1.5);
                
                //minor
                else
                    ofSetLineWidth(1);
                if(!jump){
                if (iDimension==0)
                    ofLine(0, yz, -scale, 0, yz, scale);
                else
                    ofLine(0, -scale, yz, 0, scale, yz);
                }
            }
        }
        ofPopStyle();
        
        if (labels) {
            //draw numbers on axes
            ofPushStyle();
            ofSetColor(255, 255, 255);
            
            float accuracy = ceil(-log(scale/ticks)/log(10.0f));
            
            ofSetDrawBitmapMode(OF_BITMAPMODE_MODEL_BILLBOARD);
            for (float yz = -scale; yz<=scale; yz+=minor)
            {
                ofDrawBitmapString(ofToString(yz, accuracy), 0, yz, 0);
                ofDrawBitmapString(ofToString(yz, accuracy), 0, 0, yz);		
            }
            ofPopStyle();
        }
        
    }
    
    
};

#endif
