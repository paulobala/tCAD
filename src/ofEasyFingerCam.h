#ifndef tCAD_ofEasyCamerafinger_h
#define tCAD_ofEasyCamerafinger_h

#pragma once

#include "ofCamera.h"
#include "ofEvents.h"
#include "Shape3D.h"
#include "Finger.h"
#include "lockableVector.h"
#include "rotationTween.h"
#include "scalingTween.h"
#include "panningTween.h"
#include "Dash.h"
#include "AxisPlane.h"

/*
 Camera Object. Based on source code for native ofEasyCam
 */
class ofEasyFingerCam : public ofCamera {
protected:
  
private:
	void setDistance(float distance, bool save);
    std::vector<Shape3D*> glSelect(int x, int y);
    std::vector<Shape3D*> list_hits(GLint hits, GLuint *buffer);
    void updateRotation();
    void setAnglesFromOrientation();
    float rotationX;
	float rotationY;
	float rotationZ;	
	float targetXRot;
	float targetYRot;
	float targetZRot;
    bool zooming;
	void	keyPressed(ofKeyEventArgs & args);
	float drag;
	float zoomSpeed;
	bool bMouseInputEnabled;
    
	ofVec3f mousePosViewPrev;
	ofVec3f mousePosScreenPrev;
    int lastFrame;
	
	bool mousePressedPrev[2];
    
	bool bDistanceSet;
	float lastDistance;
	float distanceScaleVelocity;
	
	ofQuaternion rotation;
	ofVec3f translation;
    
    bool bFingerInputEnabled;
    
    std::vector<Finger *> fingers;
    std::vector<Finger *> fingersPickers;
    
    bool fingerPressedPrev[2];
    
    float prevDistance;
    
    GLint		viewport[4];
	GLdouble	matM[16], matP[16];
    ofRectangle viewportRect;
    
    std::vector<RotationTween *> rTweens;
    std::vector<ScalingTween *> sTweens;
    std::vector<PanningTween *> pTweens;
    ofNode target;
    
public:
    enum STATE{
    ROTATING, SCALING, PANNING, TWEENING, STABLE, POINTING, RESET
    };
    
    ofEasyFingerCam();
	~ofEasyFingerCam();
    
    STATE currentState;
    AxisPlane * selectedPlane;
    LockableVector<Shape3D *> * shapes;
    
    void draw();
    void checkSelectable();
	virtual void begin(ofRectangle viewport = ofGetCurrentViewport());
	void reset();
    void end();
	//----------------------------------------
	// advanced functions
    
	void setTarget(const ofVec3f& target);
	void setTarget(ofNode& target);
	ofNode& getTarget();
    
	void setDistance(float distance);
	float getDistance() const;
    
	// drag is how quickly the camera picks up and slows down
	// it is a normalized value between 0-1
	void setDrag(float drag);
	float getDrag() const;
	
	void mouseDragged(ofMouseEventArgs& mouse);
	void mouseMoved(ofMouseEventArgs& mouse);
	void mousePressed(ofMouseEventArgs& mouse);
	void mouseReleased(ofMouseEventArgs& mouse);
    
	// enable or disable mouse input to navigate
	void enableMouseInput();
	void disableMouseInput();
	bool getMouseInputEnabled();
    
    void fingerIn(Finger *  finger);
    void fingerMoved(Finger *  finger);
    void fingerOut(Finger *  finger);
    
    void fingerPickerIn(Finger *  finger);
    void fingerPickerMoved(Finger *  finger);
    void fingerPickerOut(Finger *  finger);
    
    // enable or disable finger input to navigate
    void enableFingerInput();
    void disableFingerInput();
    bool getFingerInputEnabled();

    void findCursor();
    ofVec3f cursor; 
 
    void addRotationTween(int ax, int ay, int az, int speed);
    void addScaleTween(float scale);
    void addPanningTween(ofVec3f value);
    
    void clearFingersPickers();
    void clearFingers();

};


#endif
