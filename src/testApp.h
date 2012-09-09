#pragma once

#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxTuioClient.h"
#include "ofxKinect.h"
#include "lineGeom.h"
#include "plane.h"
#include "lineIntersection.h"
#include "KalmanPixel.h"
#include "Eigen.h"
#include "AxisPlane.h"
#include "ofEasyFingerCam.h"
#include "ofxSTL.h"
#include "Shape3D.h"
#include "Basic3DObject.h"
#include "Composite3DObject.h"
#include "Basic3DObjectFromSTL.h"
#include "Basic3DObjectFromContour.h"
#include "ContourDrawArea.h"
#include "kinectDrawVariables.h"
#include "Basic3DObjectFromKinect.h"
#include "Basic3DObjectFromCopy.h"
#include "Basic3DObjectBlank.h"
#include "ofxUI.h"
#include "ofMeshtCAD.h"
#include "lockableVector.h"
#include "ContainerToken.h"
#include "ShredderToken.h"
#include "grids.h"
#include "SelectionMovement.h"
#include "OnTableContentCreationUI.h"
#include "TokenData.h"
#include "Token.h"
#include "Finger.h"
#include "ofxTuioClientCCV.h"
#include "DetectedFinger.h"
#include "Observer.h"
#include "Subject.h"
#include "CalibrationFinger.h"
#include "CalibrationMarker.h"
#include "OnTokenContentCreationUI.h"
#include "InAirContentCreationUI.h"
#include "ColorScheme.h"
#include "Dash.h"
#include "OrbitToken.h"
#include "SaveToken.h"

//Fiducial Markers IDs
#define MARKER_ID_SHREDDER1 6
#define MARKER_ID_SHREDDER2 7
#define MARKER_ID_X_Z 11
#define MARKER_ID_X_Y 8
#define MARKER_ID_Y_Z 9
#define MARKER_ID_ONTABLECCTOKEN 0
#define MARKER_ID_ONTOKENCCTOKEN 4
#define MARKER_ID_INAIRCCTOKEN 5
#define MARKER_ID_ORBIT 10
#define MARKER_ID_SAVE 12

class testApp : public ofBaseApp{
    
public:
    
    //Defines Application screens
    enum SCREEN{
        SCREEN1,// Physical Setup with Point Cloud View
        SCREEN2,// Physical Setup with Depth Images View
        SCREEN3,// Physical Setup with Blob tracking
        SCREEN4,// tCAD application   
        SCREEN5,// Calibration Screen for fingers
        SCREEN6 //Calibration Screen for markers
    };
    
    //Graphical User Interfaces - ofxUI addon
    enum GUI{
        NOGUI,//No GUI
        GUISETUP,//GUI for Setup Screens
        GUIKINECT//GUI for Kinect Scannning Mode
    };
    
    /*Setup*/
    
    void setup();//setup of parameters about the physical setup
    void setuptCAD();//setup of parameters about the tCAD application
    //--------------------------------------------------------------
    /*Update*/
    
    void update();//image processing, plane detection, tabletop corner detection, etc.
    void updatetCAD();//finger and token processing
    //--------------------------------------------------------------
    /*Draw*/
    
    void draw();//draw screens
    void drawtCAD();//draw Screen 4; includes drawing the 3D Scene and 3D Content
    //--------------------------------------------------------------
    /*Exit*/
    
    void exit();//close Kinect
    //--------------------------------------------------------------
    /*Axis/Plane*/
    
    AxisPlane * axis;//plane chosen in the 3D Scene
    //--------------------------------------------------------------
    /*GUIs */
    
    SCREEN screenChoice;//current Screen
    GUI guiChoice;//current GUI choice
    ofxUICanvas * gui;//responsible for showing GUI from ofxUI
    void unsetGUI();//erase GUI
    void setGUISETUP();//set GUI for Setup Screens
    void guiSetupEvent(ofxUIEventArgs &e);//receives events from the GUI of Setup Screens
    void setGUIKINECT();//set GUI for Kinect Scanninf Mode
    void guiKinectEvent(ofxUIEventArgs &e);//receives events from the GUI of Kinect Scanning Mode
    //--------------------------------------------------------------
    /*Keys*/
    
    void keyPressed (int key);//deals with key events
    void keyReleased (int key);//deals with key events
    //--------------------------------------------------------------
	/*Mouse*/
    
    void mouseMoved(int x, int y );//deals with mouse events
    void mouseDragged(int x, int y, int button);//deals with mouse events
    void mousePressed(int x, int y, int button);//deals with mouse events
    void mouseReleased();//deals with mouse events
    //--------------------------------------------------------------
    /*Tabletop Corner Lines*/ 
    
    lineGeom *topLine, *leftLine, *bottomLine, *rightLine;//edge lines of the tabletop
    void resetLines();//erase tabletop edge lines
    void findLines(IplImage * asrc);//finds tabletop edge lines
    //--------------------------------------------------------------
    /*Corners*/
    
    cv::Point *ltCorner,*lbCorner, *rtCorner, *rbCorner;//Left Top Corner, Left Bottom Corner, ...
    KalmanPixel *ltCornerKalman, *lbCornerKalman, *rtCornerKalman, *rbCornerKalman;//Kalman Pixels for each corner
    ofPoint* centerTabletop;//Center of rectangle made by the corners
    bool lockCorners;//if true, finds corners
    void updateCorners();
    void showCorners(int offsetX, int offsetY);//Show Corners on Screen 2
    //--------------------------------------------------------------
    /*Plane */
    
    bool findingPlanes;//if true, finding plane
    
    //Planes
    Plane planeAboveSurface;
    Plane planeAboveObjects;
    
    //Plane variables for GUI
    int radiusForPlaneDetection;//pixels in the circle formed by the center of the tabletop and this radius are used in plane detection
    float distanceForPlaneAboveSurface;
    float distanceForPlaneAboveObjects;
    bool lockPlane;//if true, finds and locks plane
    float addNormalX;//correct x normal
    float addNormalY;//correct y normal
    float addNormalZ;//correct z normal
    
    //Plane Detection Stabilization
    double normalUnitVec[3];//x,y,z normals
    double distancePlane;//distance of plane
    double normalUnitVecPredict[3];//normals predicted by kalman filter
    double distancePlanePredict;//distance of plane predicted by kalman filter
    CvKalman *m_pKalmanFilter; //for more info, see http://en.wikipedia.org/wiki/Kalman_filter
    CvRandState rng; 
    CvMat* state; 
    CvMat* process_noise; 
    CvMat* measurement; 
    CvMat* measurement_noise; 
    void setupKalmanPlane();// setup kalman filter
    
    //converts a group of points into a best fit plane
    //based on http://bullet.googlecode.com/svn/trunk/Extras/ConvexDecomposition/bestfit.cpp
    bool getBestFitPlane(unsigned int vcount, const float *points,
                         unsigned int vstride, const float *weights,
                         unsigned int wstride, float *plane);
    
    //get depth images according to the orientation of the plane
    void getDepthImgForPlanes();
    void getDepthImgForPlanesAlternative();
    
    //Depth Images for both planes
    ofxCvGrayscaleImage depthImgForSurfacePlane;
    ofxCvGrayscaleImage depthImgForPlaneAboveObjects;
    
    //correct perspective of depth images
    void perspectiveCorrectionSurfacePlane();
    void perspectiveCorrectionPlaneAboveObjects();
    
    //corrects depth values according to perspective
    void perspectiveCorrectionDepthValues();
    std::vector<float> correctedDepthValues;//result of perspectiveCorrectionDepthValues()
    
    //Corrected depth images for both planes
    ofxCvGrayscaleImage perspectiveDepthImgForSurfacePlane;
    ofxCvGrayscaleImage perspectiveDepthImgForPlaneAboveObjects;
    //--------------------------------------------------------------
    /*Finger Detection*/
    
    vector<DetectedFinger *> detectFingers();//identifies fingers on masked and corrected depth images
    std::vector<DetectedFinger *> detectedFingers;//fingers found in detectFingers()
    long newFingerIDCounter;//holds ID number for new fingers
    void stitchFingers();//joins detected fingers with existing fingers
    
    //masked depth images used by detectFingers()
    ofxCvGrayscaleImage maskedDepthImgForSurfacePlane;
    ofxCvGrayscaleImage maskedDepthImgForPlaneAboveObjects;
    //--------------------------------------------------------------
    /*Kinect*/
    
    ofxKinect kinect;
    ofxCvGrayscaleImage originalDepthImage;
    void drawKinectPointCloud();//used in Screen 1
    ofEasyCam camForKinectPointCloud;//camera used for 3d scene of Kinects Point Cloud
    //--------------------------------------------------------------
    /*TUIO*/
    
    ofxTuioClientCCV client;//TUIO client
    //--------------------------------------------------------------
    /*Blobs*/
    ofxCvContourFinder blobfinder;//analyzes depth image of plane for blobs; blobs will correspond to tokens 
                                  // Blob variables for GUI
    int minBlobSize;
    int maxBlobSize;
    int numBlobs;
    //--------------------------------------------------------------
    /*3D Scene*/
    
    ofEasyFingerCam cameraScene;//camera for 3D Scene; controllable with fingers
    ofRectangle viewport;//viewport for camera
    ofLight light;//lighting conditions
    ofVec3f shapeEntryPoint;//point in space for the placement of 3D Content
    LockableVector<Shape3D *> shapes;//3D Shapes on the 3D Scene = 3D Content
    void drawScene();//Draw the 3D Scene
    //--------------------------------------------------------------
    /*Finger and Tokens Pipeline*/
    
    LockableVector<Finger *> fingers;//fingers above the tabletop
    LockableVector<Token *> tokens;//tokens on the tabletop
    LockableVector<TokenData *> tokensData;//Extra data about tokens
    
    //Token actions
    void objectAdded(TuioObject & obj);
    void objectRemoved(TuioObject & obj);
    void objectUpdated(TuioObject & obj);
    void changeMarkerType(Token * marker);
    
    //Average Depth
    float averageDepth(ofRectangle bounding);//inside a rectangle
    float averageDepth(float xorig, float origy);//inside a circle with this center
    float averageDepthCorrected(float xorig, float yorig);//inside a circle with this center
    
    //Finger actions
    void changeFingerType(Finger * finger);
    void updateLevel(Finger * finger);//find level of finger
    void changeLevel(Finger * finger);//find if there was a transition in levels
    void onSurface(Finger * finger);//find if a finger is touching a element in the surface or a token
    //--------------------------------------------------------------
    /*Container Tokens*/
    
    LockableVector<ContainerToken*> containers;//container tokens on the system (on the table and not on the table)
    LockableVector<SelectionMovement*> selectionMovements;//shape selection movements going on above the tabletop
    bool useAlternativeManipulationUI;//if true, use Alternative Manipulation UI (finger movement containers)
    //--------------------------------------------------------------
    /*Shredder Tokens*/
    
    ShredderToken * shredder1;
    ShredderToken * shredder2;
    bool hasShredder1;
    bool hasShredder2;
    //--------------------------------------------------------------
    /*Content Creation UI*/
    
    //3 Alternative CC UI
    OnTableContentCreationUI * onTableCCToken;
    OnTokenContentCreationUI * onTokenCCToken;
    InAirContentCreationUI * inAirCCToken;
    
    //Countour Mode for Content Creation
    ContourDrawArea drawAreaForContour;
    
    //Kinect Scanning for Content Creation
    Basic3DObjectFromKinect * kinectScan;//textured 2.5D depth map shown on the 3D Scene
    bool kinectScanning;//If true, show kinectScan;
    kinectDrawVariables kinectDraw_Variables;//variable for the Kinect Scanning process
    bool showKinectGUI;//Shows variable for the Kinect Scanning process
    //--------------------------------------------------------------
    /*Orbit token*/
    
    OrbitToken * orbiter;
    bool hasOrbiter;
    //--------------------------------------------------------------
    /*Save token*/
    SaveToken * saveToken;
    //--------------------------------------------------------------
    /*Calibration*/
    //Based on CCV Source Code
    
    //Calibration markers/tokens
    void updateCalibrationMarker();
    void drawCalibrationMarker();//draw Screen 5
    CalibrationMarker * calibrationMarker;//holds calibration info
    bool calibratingMarker;//if false, markers are corrected; if true, markers are sent to the calibration defintion process
    
    //Calibration Fingers
    void updateCalibrationFingers();
    void drawCalibrationFingers();//draw Screen 6
    CalibrationFinger * calibrationFinger;//holds calibration info
    bool calibratingFingers;//if false, fingers are correctes; if true, fingers are sent to the calibration defintion process    
    //--------------------------------------------------------------
    /*Loading and Saving Configurations into XML*/
    ofxXmlSettings xmlfile;//XML file reader
    bool loadXML();
    void saveXML();
    //--------------------------------------------------------------
    /*Images and fonts*/
    ofImage logo;
    ofImage cameraImg;//If there is a camera control action happening, this image is shown
    ofTrueTypeFont font;
    //--------------------------------------------------------------
    bool inside(float x, float y, ofPolyline polyline);//inside a polygon; used for shredders
    double normCV(cv::Point pt);
    
};