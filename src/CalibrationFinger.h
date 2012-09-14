#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "ofxXmlSettings.h" // LOAD CONFIG.XML
#include "ofMain.h"
#include "CalibrationUtilsFinger.h"
#include "Finger.h"

/*
 Calibration process. Based on http://ccv.nuigroup.com/
 */
class CalibrationFinger{
    
public:
    
    CalibrationFinger() {
        ofAddListener(ofEvents.keyPressed, this, &CalibrationFinger::_keyPressed);
        ofAddListener(ofEvents.keyReleased, this, &CalibrationFinger::_keyReleased);
        calibrating = false;
        bShowTargets = true;
        bW = false;
        bA = false;
        bS = false;
        bD = false;
        targetColor = 0xFF0000;
        arcAngle    = 0;
        calibrated = false;
    }
    
    std::vector<std::pair<Finger *,unsigned long > > touches;
    //Basic Methods
    void setup(int _camWidth, int _camHeight);
    //Key Events
    void _keyPressed(ofKeyEventArgs &e);
    void _keyReleased(ofKeyEventArgs &e);
    
    void doCalibration();
    
    bool calibrating;
    
    
    void touchDown(Finger *  touch);
    void touchMoved(Finger *  touch);
    void touchUp(Finger *  touch);
    void update();
    ofVec2f applyCalibration(float x, float y);
    bool calibrated;
    
    bool insideCalibrationRectangle(float x, float y);
    
private:
    
    void drawFingerOutlines();
    void drawCalibrationBlobs();
    void drawCalibrationPointsAndBox();
    void saveConfiguration();
    void DrawCircleLoader(double xctr, double yctr, double radius, double startAngle, double endAngle);
    
    bool bW;
    bool bS;
    bool bA;
    bool bD;
    bool bShowTargets;
    int  camWidth;
    int  camHeight;
    float arcAngle;
    float targetColor;
    //Fonts
    ofTrueTypeFont	font;
    ofTrueTypeFont	calibrationText;
    //Draw Particle Image
    ofImage calibrationParticle;		
    CalibrationUtilsFinger calibrate;
    ofxXmlSettings	calibrationXML;
};

#endif
