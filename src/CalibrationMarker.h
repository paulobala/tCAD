

#ifndef CALIBRATIONMARKER_H
#define CALIBRATIONMARKER_H

#include "ofxXmlSettings.h" // LOAD CONFIG.XML
#include "CalibrationUtilsMarker.h"
#include "Token.h"

/*
 Calibration process. Based on http://ccv.nuigroup.com/
 */
class CalibrationMarker{

	public:

		CalibrationMarker() {
            ofAddListener(ofEvents.keyPressed, this, &CalibrationMarker::_keyPressed);
            ofAddListener(ofEvents.keyReleased, this, &CalibrationMarker::_keyReleased);
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
    
        std::vector<std::pair<Token * ,unsigned long > > tokens;
    
		//Basic Methods
        void setup(int _camWidth, int _camHeight);
		//Key Events
		void _keyPressed(ofKeyEventArgs &e);
		void _keyReleased(ofKeyEventArgs &e);

        void doCalibration();
		bool calibrating;
    
        void objectAdded(Token *  touch);
        void objectRemoved(Token *  touch);
        void objectUpdated(Token *  touch);

        void update();
        ofVec2f applyCalibration(float x, float y);
        bool calibrated;
        void drawBox(ofColor color);
    
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
		float           arcAngle;
		float targetColor;
		//Fonts
		ofTrueTypeFont	font;
		ofTrueTypeFont	calibrationText;
 
		CalibrationUtilsMarker calibrate;
		ofxXmlSettings	calibrationXML;
    
   
};

#endif
