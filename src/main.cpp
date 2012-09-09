#include "ofMain.h"
#include "testApp.h"
#include "ofAppGlutWindow.h"

//========================================================================
int main( ){

    ofAppGlutWindow window;
    window.setGlutDisplayString("rgba double samples>=4");//changed so lines in 3D would be aliased
    window.setGlutDisplayString("rgba double depth alpha samples>=6");

	ofSetupOpenGL(&window,640*1.5,480+100, OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp( new testApp());

}
