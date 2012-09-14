#ifndef tCAD_kinectDrawVariables_h
#define tCAD_kinectDrawVariables_h

/*
 variables for Kinect Mode. Based on source code at https://github.com/cibomahto/Makerbot-1/tree/master/KinectToStl
*/
class kinectDrawVariables{
public:
    int zCutoff, smoothingAmount, backOffset;
    float fovWidth, fovHeight, temporalBlur, randomCount,randomBlur, randomWeight ;
	bool useRandom, useSmoothing, useSimplify, drawMesh, drawWireFrame, lock;	
	kinectDrawVariables(){
        zCutoff = 960;
        smoothingAmount = 4;
        backOffset = 4 ;
        fovWidth = 0.16;
        fovHeight = 0.17;
        temporalBlur = 0.9;
        randomCount = 10000;
        randomBlur= 6;
        randomWeight = 1.5;
        useRandom = false,
        useSmoothing = true;
        useSimplify = true;
        drawMesh = true;
        drawWireFrame = false;
        lock = false;
    }
};
#endif
