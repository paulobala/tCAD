#include <iostream>
/*
 TUIO Client Wrapper for OpenFrameworks 
 Copyright (c) 2009 Matthias Dörfelt <info@mokafolio.de>
 Based on the TUIO Demo by Martin Kaltenbrunner:
 
 Copyright (c) 2005-2009 Martin Kaltenbrunner <mkalten@iua.upf.edu>
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "ofxTuioClientCCV.h"

ofxTuioClientCCV::ofxTuioClientCCV() {
    bIsConnected = false;	
	bVerbose = false;
}

void ofxTuioClientCCV::connect(int port){
	client = new TuioClient(port);
	client->addTuioListener(this);
	client->connect();
	
	if (!client->isConnected()) {
		cout<<"Could not connect TUIO Client"<<endl;
	} else {
	    bIsConnected = true;	
	}
	
}

void ofxTuioClientCCV::disconnect(){
	if(bIsConnected) client->disconnect();
	bIsConnected = false;
}

void ofxTuioClientCCV::setVerbose(bool b){
	bVerbose = b;
}

void ofxTuioClientCCV::drawCursors(){
    std::list<TuioCursor*> cursorList = client->getTuioCursors();
	std::list<TuioCursor*>::iterator tit;
	client->lockCursorList();
	for (tit=cursorList.begin(); tit != cursorList.end(); tit++) {
		TuioCursor * cur = (*tit);
		//if(tcur!=0){
        //TuioCursor cur = *tcur;
        glColor3f(0.0,0.0,0.0);
        ofEllipse(cur->getX()*ofGetWidth(), cur->getY()*ofGetHeight(), 10.0, 10.0);
        string str = "SessionId: "+ofToString((int)(cur->getSessionID()));
        ofDrawBitmapString(str, cur->getX()*ofGetWidth()-10.0, cur->getY()*ofGetHeight()+25.0);
        str = "CursorId: "+ofToString((int)(cur->getCursorID()));
        ofDrawBitmapString(str, cur->getX()*ofGetWidth()-10.0, cur->getY()*ofGetHeight()+40.0);
		//}
	}
	client->unlockCursorList();
}

void ofxTuioClientCCV::drawObjects(){
    std::list<TuioObject*> objectList = client->getTuioObjects();
	list<TuioObject*>::iterator tobj;
	client->lockObjectList();
	for (tobj=objectList.begin(); tobj != objectList.end(); tobj++) {
		TuioObject *obj = (*tobj);
		glColor3f(1.0,0.0,0.0);
		glPushMatrix();
		glTranslatef(obj->getX()*ofGetWidth(), obj->getY()*ofGetHeight(), 0.0);
		glRotatef(obj->getAngleDegrees(), 0.0, 0.0, 1.0);
		ofRect(-10.0, -10.0, 20.0, 20.0);
		glColor3f(1.0,1.0,1.0);
		ofLine(0, 0, 0, -10);
		glPopMatrix();
		string str = "SymbolId: "+ofToString((int)(obj->getSymbolID()));
		ofDrawBitmapString(str, obj->getX()*ofGetWidth()-10.0, obj->getY()*ofGetHeight()+25.0);
		str = "SessionId: "+ofToString((int)(obj->getSessionID()));
		ofDrawBitmapString(str, obj->getX()*ofGetWidth()-10.0, obj->getY()*ofGetHeight()+40.0);
	}
	client->unlockObjectList();
}

void ofxTuioClientCCV::addTuioObject(TuioObject *tobj) {
	
	ofNotifyEvent(objectAdded, *tobj, this);
	
	if (bVerbose)
		std::cout << "add obj " << tobj->getSymbolID() << " (" << tobj->getSessionID() << ") "<< tobj->getX() << " " << tobj->getY() << " " << tobj->getAngle() << std::endl;
	
}

void ofxTuioClientCCV::updateTuioObject(TuioObject *tobj) {
	
	ofNotifyEvent(objectUpdated, *tobj, this);
	
	if (bVerbose) 	
		std::cout << "set obj " << tobj->getSymbolID() << " (" << tobj->getSessionID() << ") "<< tobj->getX() << " " << tobj->getY() << " " << tobj->getAngle() 
		<< " " << tobj->getMotionSpeed() << " " << tobj->getRotationSpeed() << " " << tobj->getMotionAccel() << " " << tobj->getRotationAccel() << std::endl;
	
}

void ofxTuioClientCCV::removeTuioObject(TuioObject *tobj) {
	
	ofNotifyEvent(objectRemoved, *tobj, this);
	
	if (bVerbose)
		std::cout << "del obj " << tobj->getSymbolID() << " (" << tobj->getSessionID() << ")" << std::endl;
}

void ofxTuioClientCCV::addTuioCursor(TuioCursor *tcur) {
//	ofTouchEventArgs touch;
//    touch.x=tcur->getX();
//	touch.y=tcur->getY();
//	touch.id=tcur->getSessionID();
	
	ofNotifyEvent(cursorAdded, *tcur, this);
	
	if (bVerbose) 
		std::cout << "add cur " << tcur->getCursorID() << " (" <<  tcur->getSessionID() << ") " << tcur->getX() << " " << tcur->getY() << std::endl;
	
}

void ofxTuioClientCCV::updateTuioCursor(TuioCursor *tcur) {
    
//	ofTouchEventArgs touch;
//	touch.x=tcur->getX();
//	touch.y=tcur->getY();
//	touch.id=tcur->getSessionID();
	
	ofNotifyEvent(cursorUpdated, *tcur, this);
	
	if (bVerbose) 	
		std::cout << "set cur " << tcur->getCursorID() << " (" <<  tcur->getSessionID() << ") " << tcur->getX() << " " << tcur->getY() 
		<< " " << tcur->getMotionSpeed() << " " << tcur->getMotionAccel() << " " << std::endl;
}

void ofxTuioClientCCV::removeTuioCursor(TuioCursor *tcur) {
//    
//	ofTouchEventArgs touch;
//	touch.x=tcur->getX();
//	touch.y=tcur->getY();
//	touch.id=tcur->getSessionID();
	
	ofNotifyEvent(cursorRemoved, *tcur, this);
	
	if (bVerbose)
		std::cout << "del cur " << tcur->getCursorID() << " (" <<  tcur->getSessionID() << ")" << std::endl;
}

void ofxTuioClientCCV::refresh(TuioTime frameTime) {
	
}