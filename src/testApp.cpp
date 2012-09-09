#include "testApp.h"
//--------------------------------------------------------------
void testApp::setup()
{
    screenChoice = SCREEN1;//starts on Setup Screen
    findingPlanes = true;
    //green plane on point cloud view
    normalUnitVec[0] = 0;
    normalUnitVec[1] = 0;
    normalUnitVec[2] = 0;
    distancePlane = 0;
    //GUI elements for plane detection
    radiusForPlaneDetection = 50;//red circle on point cloud view
    distanceForPlaneAboveSurface=31.38;
    distanceForPlaneAboveObjects=52.30;
    addNormalX = 0;
    addNormalY = 0;
    addNormalZ = 0;
    //GUI elements for Blob tracking
    minBlobSize = 2140;
    maxBlobSize = 5000;
    numBlobs = 20;
    lockPlane = false;
    lockCorners = true;
    findingPlanes = true;
    ofSetLogLevel(OF_LOG_VERBOSE);
    //Start TUIO Client
    client=ofxTuioClientCCV();
    client.connect(3333);
    //Start Kinect
    kinect.setRegistration(false);
    kinect.init();
    kinect.open();
    //Allocate space for depth images 
    originalDepthImage.allocate(kinect.getWidth(), kinect.getHeight());
    depthImgForSurfacePlane.allocate(kinect.getWidth(), kinect.getHeight());
    depthImgForPlaneAboveObjects.allocate(kinect.getWidth(), kinect.getHeight());
    perspectiveDepthImgForSurfacePlane.allocate(kinect.getWidth(), kinect.getHeight());
    perspectiveDepthImgForPlaneAboveObjects.allocate(kinect.getWidth(), kinect.getHeight());
    maskedDepthImgForSurfacePlane.allocate(kinect.getWidth(), kinect.getHeight());
    maskedDepthImgForPlaneAboveObjects.allocate(kinect.getWidth(), kinect.getHeight());
    correctedDepthValues.reserve(kinect.width*kinect.height);//reserve space for matrix
                                                             
    //Setup edges of tabletop and corners
    centerTabletop = new ofPoint();
    resetLines();
    ltCorner = new cv::Point();
    rtCorner= new cv::Point();
    rbCorner = new cv::Point();
    lbCorner= new cv::Point();
    ltCornerKalman = new KalmanPixel(*ltCorner);
    rtCornerKalman = new KalmanPixel(*rtCorner);
    rbCornerKalman = new KalmanPixel(*rbCorner);
    lbCornerKalman = new KalmanPixel(*lbCorner);
    //Setup Calibration
    calibrationFinger = new CalibrationFinger();
    calibrationFinger->setup(640, 480);
    calibrationMarker= new CalibrationMarker();
    calibrationMarker->setup(640, 480);
    //Setup kalman filter for plane detection
    setupKalmanPlane();
    updateCorners();
    //Setup GUI
    guiChoice= NOGUI;
    setGUISETUP();
    //Setup tCAD specific options
    setuptCAD();
    ofSetFrameRate(60);
}
//--------------------------------------------------------------
void testApp::update()
{
    kinect.update();//get frame
    if(kinect.isFrameNew())
    {
        //Store Depth Image
        originalDepthImage.setFromPixels(kinect.getDepthPixels(), kinect.getWidth(), kinect.getHeight());
        
        //Find Corners of Table
        if(lockCorners)//if requested by GUI
        {
            findLines(originalDepthImage.getCvImage());
            lockCorners = false;
        }
        if(findingPlanes)//if a definite plane has not been found
        {
            vector<ofVec3f> pts;//Red Circle on Point Cloud View
            for(int x= centerTabletop->x - radiusForPlaneDetection; 
                x< centerTabletop->x + radiusForPlaneDetection && x< kinect.width; 
                x++)
            {
                for(int y= centerTabletop->y- radiusForPlaneDetection;
                    y < centerTabletop->y + radiusForPlaneDetection && y < kinect.height;
                    y++)
                {
                    if(centerTabletop->distance(ofPoint(x,y))<= radiusForPlaneDetection)
                    {
                        ofVec3f p =  kinect.getWorldCoordinateAt(x,y,kinect.getDistanceAt(x, y));
                        if(p.z!=0)//ignore infinite depth values
                        {
                            pts.push_back(p);
                        }
                    }
                }
            }
            //change vector of points into array of floats as expected by the best fit plane
            float points[pts.size()*3];
            
            for(int ff=0; ff<pts.size(); ff++)
            {
                points[ff*3] = pts[ff].x;
                points[ff*3+1] = pts[ff].y;
                points[ff*3+2] = pts[ff].z;
            }
            
            float plane[4];//result stored here
            getBestFitPlane(pts.size(),points,sizeof(float)*3,0,0,plane);
            
            //Best plane for the points inside the red circle
            normalUnitVec[0]= plane[0];
            normalUnitVec[1]= plane[1];
            normalUnitVec[2]= plane[2];
            distancePlane = plane[3];
            
            //Correct plane values with a kalman filter
            const CvMat* prediction = cvKalmanPredict( m_pKalmanFilter, 0 );
            //get predictions
            normalUnitVecPredict[0] = prediction->data.fl[0];
            normalUnitVecPredict[1] = prediction->data.fl[1];
            normalUnitVecPredict[2] = prediction->data.fl[2];
            distancePlanePredict = prediction->data.fl[3];
            //set real values
            state->data.fl[0] = normalUnitVec[0];
            state->data.fl[1] = normalUnitVec[1];
            state->data.fl[2] = normalUnitVec[2];
            state->data.fl[3] = distancePlane;
            // generate measurement noise(z_k)
            cvRandSetRange( &rng, 0, sqrt(m_pKalmanFilter->measurement_noise_cov->data.fl[0]), 0 );
            cvRand( &rng, measurement_noise );
            cvMatMulAdd( m_pKalmanFilter->measurement_matrix, state, measurement_noise, measurement );
            // adjust Kalman filter state
            const CvMat* correction = cvKalmanCorrect( m_pKalmanFilter, measurement );
            // generate process noise(w_k)
            cvRandSetRange( &rng, 0, sqrt(m_pKalmanFilter->process_noise_cov->data.fl[0]), 0 );
            cvRand( &rng, process_noise );
            cvMatMulAdd( m_pKalmanFilter->transition_matrix, state, process_noise, state );
            //Accept adjusted values
            normalUnitVec[0] = correction->data.fl[0];
            normalUnitVec[1] = correction->data.fl[1];
            normalUnitVec[2] = correction->data.fl[2];
         
            //Set planes with adjusted values; both planes are equal at this phase
            planeAboveSurface = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),distancePlane);
            planeAboveObjects = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),distancePlane);
            
            if(lockPlane)//if requested by GUI
            {
                vector<ofxCvBlob> blobsTemp;
                Boolean foundBlobs = false;
                float endDistance;
                //move the plane towards the kinect
                
                if(planeAboveSurface.normal.z >0)//check side up of plane
                {
                    endDistance = planeAboveSurface.d -20;//termination value
                    planeAboveSurface = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),-600);
                    planeAboveObjects = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),-600);
                    getDepthImgForPlanes();//masked image
                }
                else
                {
                    endDistance = planeAboveSurface.d +20;//termination value
                    planeAboveSurface = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),600);
                    planeAboveObjects = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),600);
                    getDepthImgForPlanesAlternative();//masked image
                }
                
                //take masked depth image and find blobs
                blobfinder.findContours(depthImgForSurfacePlane, minBlobSize, maxBlobSize, numBlobs, false);
                blobsTemp = blobfinder.blobs;//store blobs
                
                if(planeAboveSurface.normal.z >0)//which side is up?
                {
                    while(!foundBlobs && planeAboveSurface.d > endDistance)//if still have not found blobs and has not reached termination value
                    {
                        planeAboveSurface = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),planeAboveSurface.d-5);//lower plane
                        planeAboveObjects = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),planeAboveSurface.d-5);
                        getDepthImgForPlanes();//update masked Depth Image
                        blobfinder.findContours(depthImgForSurfacePlane, minBlobSize, maxBlobSize, numBlobs, false);
                        blobsTemp = blobfinder.blobs;
                        if(blobsTemp.size()== 3)//found the three columns
                        {
                            foundBlobs = true;
                        }
                    }
                }
                else // planeAboveSurface.normal.z >0
                {
                    while(!foundBlobs && planeAboveSurface.d < endDistance)
                    {
                        planeAboveSurface = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),planeAboveSurface.d+5);//lower plane
                        planeAboveObjects = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]),planeAboveSurface.d+5);
                        getDepthImgForPlanes();//update masked Depth Image
                        blobfinder.findContours(depthImgForSurfacePlane, minBlobSize, maxBlobSize,numBlobs, false);
                        blobsTemp = blobfinder.blobs;
                        if(blobsTemp.size()== 3)//found the three columns
                        {
                            foundBlobs = true;
                        }
                    }
                }
                if(!foundBlobs)
                {
                    lockPlane = false;//the plane detection failed; redo the plane detection
                }
                else
                {//a plane with top of columns was found
                    
                    //we need to get points inside the blobs to feed into the best fit plane
                    vector<ofVec3f> pts2;
                    for(int x=0;  x< kinect.width; x++)
                    {
                        for(int y= 0; y < kinect.height; y++)
                        {
                            Boolean added = false;
                            for(vector<ofxCvBlob>::iterator it = blobsTemp.begin() ;
                                it != blobsTemp.end() && added == false;
                                it++)
                            {
                                if(!added)
                                {
                                    vector<ofPoint> pointblobs = (*it).pts;
                                    for(vector<ofPoint>::iterator it2 = pointblobs.begin(); 
                                        it2 != pointblobs.end() && added == false;
                                        it2++)
                                    {
                                        if((*it2).x==x && (*it2).y == y)
                                        {
                                            ofVec3f p =  kinect.getWorldCoordinateAt(x,y,kinect.getDistanceAt(x, y));
                                            if(p.z!=0)//ignore infinite depth values
                                            {
                                                pts2.push_back(p);
                                                added = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    //change vector of points into array of floats as expected by the best fit plane
                    float points2[pts2.size()*3];
                    for(int ff=0; ff<pts2.size(); ff++)
                    {
                        points2[ff*3] = pts2[ff].x;
                        points2[ff*3+1] = pts2[ff].y;
                        points2[ff*3+2] = pts2[ff].z;
                    }
                    
                    float plane2[4];//result is stored here
                    getBestFitPlane(pts2.size(),points2,sizeof(float)*3,0,0,plane2);
                    
                    normalUnitVec[0]= plane2[0];
                    normalUnitVec[1]= plane2[1];
                    normalUnitVec[2]= plane2[2];
                    distancePlane = plane2[3];
                    
                    //Setup Planes for use
                    planeAboveSurface = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]), distancePlane + distanceForPlaneAboveSurface);
                    planeAboveObjects = Plane(Vector3d(normalUnitVec[0], normalUnitVec[1], normalUnitVec[2]), distancePlane + distanceForPlaneAboveObjects);
                    
                    addNormalX = 0;
                    addNormalY = 0;
                    addNormalZ= 0;
                    
                    findingPlanes = false;//stop findinf planes
                    lockPlane = false;
                    //Update GUI
                    unsetGUI();//erase GUI first
                    setGUISETUP();//reload GUI
                }
            }
        }
        else
        {
            //Update Planes
            planeAboveSurface = Plane(Vector3d(normalUnitVec[0] + addNormalX, normalUnitVec[1] +addNormalY, normalUnitVec[2]+ addNormalZ), distancePlane + distanceForPlaneAboveSurface);
            planeAboveObjects = Plane(Vector3d(normalUnitVec[0] + addNormalX, normalUnitVec[1] +addNormalY, normalUnitVec[2]+ addNormalZ), distancePlane + distanceForPlaneAboveObjects);
        }
        
        //Prepare Depth Images
        if(planeAboveSurface.normal.z >0)//Which Side up?
        {
            getDepthImgForPlanes();//masks depth images with plane
        }
        else
        {
            getDepthImgForPlanesAlternative();//masks depth images with plane
        }
        
        //Correct perspective of depth images and depth values matrix
        perspectiveCorrectionSurfacePlane();
        perspectiveCorrectionPlaneAboveObjects();
        perspectiveCorrectionDepthValues();
        
        //Find blobs for Screen 3
        blobfinder.findContours(perspectiveDepthImgForPlaneAboveObjects, minBlobSize, maxBlobSize,numBlobs, false);
        
        //Finger tracking
        detectedFingers = detectFingers();
    }
    
    //Connecting blobs to objects from the TUIO clien
    vector<ofxCvBlob> blobs;
    list<TuioObject*> objs;
    int treedepth = 50;
    int iterations = 20;//limit of iterations
    blobs = blobfinder.blobs;
    objs = client.client->getTuioObjects();
    
    for (list<TuioObject*>::iterator tobj=objs.begin(); tobj != objs.end(); tobj++)
    {
        TuioObject *obj = (*tobj);
        for(vector<ofxCvBlob>::iterator it = blobs.begin(); it != blobs.end(); it++)
        {
            ofRectangle bounds = (*it).boundingRect;//get bounds of blob
            bounds.set(bounds.x - 15, bounds.y - 15, bounds.width + 30, bounds.height +30);//increase blob bounds
            
            if(bounds.inside(kinect.getWidth() - obj->getX()*kinect.getWidth(), kinect.getHeight()- (obj->getY()* kinect.getHeight())))//is tuio object inside blob bounds?
            {
                bool found =false;
                vector<TokenData*> temp = tokensData.getObjects();
                //record information in the vector
                for(vector< TokenData *>::iterator ito=temp.begin() ; ito < temp.end(); ito++ )
                {
                    if((*ito)->getID()==obj->getSymbolID())//token already there?
                    {
                        //Update depth
                        found = true;
                        float depth = averageDepthCorrected((*it).centroid.x,(*it).centroid.y);
                        (*ito)->centroidBlob = (*it).centroid;
                        if(depth != 0)//ignore infinte depth values
                        {
                            (*ito)->updateMaxDepth(depth);
                        }
                    }
                }
                if(!found)//token is new
                {
                    //Add token to vector
                    float depth = averageDepthCorrected((*it).centroid.x,(*it).centroid.y);
                    if(depth != 0)//ignore infinte depth values
                    {
                        tokensData.addElement(new TokenData(obj,depth,(*it).centroid));
                    }
                }
            }
        }
        
        vector< TokenData*> temp = tokensData.getObjects();
        for(vector< TokenData *>::iterator ito=temp.begin() ; ito < temp.end(); ito++ )
        {
            if((*ito)->getID()==obj->getSymbolID())
            {
                float depth =0;
                depth = averageDepthCorrected((*ito)->centroidBlob.x,(*ito)->centroidBlob.y);
                if(depth != 0)
                {
                    (*ito)->updateCurrentDepth(depth);
                }
            }
        }
    }
    
    //Specific update according to Screen
    if(screenChoice == SCREEN4)
    {
        updatetCAD();//update 3D Scene, tokens, fingers, etc.
    }
    if(screenChoice == SCREEN5)
    {
        updateCalibrationFingers();
    }
    if(screenChoice == SCREEN6)
    {
        updateCalibrationMarker();
    }
}

float testApp::averageDepth(ofRectangle bounding)
{
    std::vector<float> pts;
    
    //collect depth values for points inside the ofRectangle
    for(int x= bounding.getCenter().x - bounding.width/2;
        x < bounding.getCenter().x + bounding.width/2&& x< kinect.width;
        x++)
    {
        for(int y= bounding.getCenter().y - bounding.height/2;
            y < bounding.getCenter().y + bounding.height/2 && y < kinect.height;
            y++)
        {
            ofVec3f p =  kinect.getWorldCoordinateAt(x,y,kinect.getDistanceAt(x, y));
            if(p.z!=0)//ignore infinte depth values
            {
                pts.push_back(p.z);
            }
        }
    }
    
    float result;
    //Sum all depth values and divide to achieve an average
    for(int i= 0; i< pts.size(); i++)
    {
        result += pts[i];
    }
    return result/pts.size();
}

float testApp::averageDepth(float xorig, float yorig)
{
    int radius = 4;
    std::vector<float> pts;
    //collect depth values for points inside circle mde by center xorig, yorig with a radius of 4
    for(int x= xorig - radius/2; x < xorig + radius/2; x++)
    {
        for(int y= yorig - radius/2; y < yorig + radius/2; y++)
        {
            if( (x< kinect.width && x >0) && (y< kinect.height && y >0))//check if point is outside kinect resolution
            {
                ofVec3f p =  kinect.getWorldCoordinateAt(x,y,kinect.getDistanceAt(x, y));
                if(p.z!=0)//ignore infinite depth values
                {
                    pts.push_back(p.z);
                }
            }
        }
    }
    float result = 0;
    //Sum all depth values
    for(int i= 0; i< pts.size(); i++)
    {
        result += pts[i];
    }
    if(pts.size() == 0)//no points with admissable depth values were inside the circle
    {
        return 0;
    }
    return result/pts.size();//divide by number of points to achieve average
}

float testApp::averageDepthCorrected(float xorig, float yorig)
{
    int radius =4;
    //collect depth values for points inside circle mde by center xorig, yorig with a radius of 4
    std::vector<float> pts;
    for(int x= xorig - radius/2; x < xorig + radius/2; x++)
    {
        for(int y= yorig - radius/2; y < yorig + radius/2; y++)
        {
            if( (x< kinect.width && x >0) && (y< kinect.height && y >0))//check if point is outside kinect resolution
            {
                //instead of asking the Kinect for depth values, we ask the corrected depth values matrix
                int pix = (y*kinect.width) + x;
                if(correctedDepthValues[pix]!=0)//ignore infinite depth values
                {
                    pts.push_back(correctedDepthValues[pix]);
                }
            }
        }
    }
    float result = 0;
    //Sum all depth values
    for(int i= 0; i< pts.size(); i++)
    {
        result += pts[i];
    }
    if(pts.size() == 0)//no points with admissable depth values were inside the circle
    {
        return 0;
    }
    return result/pts.size();//divide by number of points to achieve average
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y )
{
}
void testApp::mouseDragged(int x, int y, int button)
{
}
void testApp::mousePressed(int x, int y, int button)
{
}
void testApp::mouseReleased()
{
}
//--------------------------------------------------------------
void testApp::keyReleased(int key)
{
}
void testApp::keyPressed (int key)
{
    if (key == 'r')//reset
    {
        //reset tokens
        onTableCCToken = false;
        onTableCCToken = NULL;
        onTokenCCToken = false;
        onTokenCCToken = NULL;
        inAirCCToken = false;
        inAirCCToken = NULL;
        useAlternativeManipulationUI = false;
        calibratingFingers = false;
        calibratingMarker = false;
        hasShredder1= false;
        shredder1 = NULL;
        hasShredder2 = false;
        shredder2 = NULL;
        hasOrbiter = false;
        orbiter = NULL;
        containers = LockableVector<ContainerToken*>();
        //reset tokens and fingers
        tokensData = LockableVector< TokenData *>();
        fingers = LockableVector<Finger *>();
        tokens = LockableVector<Token *>();
        shapes = LockableVector<Shape3D *>();
        selectionMovements = LockableVector<SelectionMovement*>();
        //reset 3D Scene
        cameraScene.clearFingersPickers();
        cameraScene.clearFingers();
        cameraScene.reset();
        cameraScene.addRotationTween(0,45,0,1);
        shapeEntryPoint = ofVec3f(0,0,0);
        //reset GUI
        unsetGUI();
        kinectScanning = false;
        showKinectGUI = false;
        drawAreaForContour.active = false;
        drawAreaForContour.clearDrawArea();
    }
    else if (key == '1')//change to Screen 1
    {
        unsetGUI();
        setGUISETUP();
        calibratingFingers = false;
        calibratingMarker = false;
        calibrationFinger->calibrating = false;
        calibrationMarker->calibrating = false;
        screenChoice = SCREEN1;
    }
    else if (key == '2')//change to Screen 2
    {
        unsetGUI();
        setGUISETUP();
        calibratingFingers = false;
        calibratingMarker = false;
        calibrationFinger->calibrating = false;
        calibrationMarker->calibrating = false;
        screenChoice = SCREEN2;
    }
    else if (key == '3')//change to Screen 3
    {
        unsetGUI();
        setGUISETUP();
        calibratingFingers = false;
        calibratingMarker = false;
        calibrationFinger->calibrating = false;
        calibrationMarker->calibrating = false;
        screenChoice = SCREEN3;
    }
    else if (key == '4')//change to Screen 4
    {
        unsetGUI();
        calibratingFingers = false;
        calibratingFingers = false;
        calibratingMarker = false;
        calibrationFinger->calibrating = false;
        calibrationMarker->calibrating = false;
        screenChoice = SCREEN4;
    }
    else if (key == '5')//change to Screen 5
    {
        unsetGUI();
        calibratingFingers = true;
        calibrationFinger->calibrating = true;
        calibratingMarker = false;
        calibrationMarker->calibrating = false;
        screenChoice = SCREEN5;
    }
    else if (key == '6')//change to Screen 6
    {
        unsetGUI();
        calibratingFingers = false;
        calibratingMarker = true;
        calibrationFinger->calibrating = false;
        calibrationMarker->calibrating = true;
        screenChoice = SCREEN6;
    }
    else if (key == 'f')//FullScreen
    {
        ofToggleFullscreen();
    }
    else if (key == 'g')//Add a cube; for testing purposes
    {
        ofVec3f point = shapeEntryPoint;
        point.x = point.x + ofRandom(-20,20);
        point.y = point.y + ofRandom(-20,20);
        point.z = point.z + ofRandom(-20,20);
        Basic3DObjectFromSTL * tempShape = new Basic3DObjectFromSTL("stls/cube.stl",point);
        ofColor * tempColor = new ofColor(0,0,0);
        tempColor->setHsb(ofRandom(0,255), ofRandom(180,255), ofRandom(180,255));//random color
        tempShape->changeColor(tempColor);
        shapes.lockVector();
        shapes.addElement(tempShape);//add to 3D Scene
        shapes.unlockVector();
    }
    else if(key == 'a')//Alternate Manipulation UI for container tokens
    {
        useAlternativeManipulationUI = !useAlternativeManipulationUI;
        vector<ContainerToken*> temp = containers.getObjects();
        containers.lockVector();
        for (std::vector<ContainerToken*>::iterator it=temp.begin() ; it < temp.end(); it++ )
        {
            (*it)->alternative = !(*it)->alternative;
            (*it)->update();
        }
        containers.unlockVector();
    }
}
//--------------------------------------------------------------
void testApp::draw()
{
    if(screenChoice==SCREEN1)
    {
        camForKinectPointCloud.begin();//start camera
        drawKinectPointCloud();//Kinect point cloud
        camForKinectPointCloud.end();//close camera
    }
    else if(screenChoice==SCREEN2)
    {
        int offset = kinect.width/2;//leave space for GUI
        
        kinect.drawDepth(0 + offset, 0, kinect.width/2 , kinect.height/2);//Kinect depth image
        kinect.draw(kinect.width/2+ offset, 0, kinect.width/2 , kinect.height/2);//kinect real color image
        showCorners(offset, 0);//draw corners on depth image
        showCorners(kinect.width/2+offset, 0);//draw corners on real color image
        maskedDepthImgForSurfacePlane.draw(0+ offset, kinect.height/2, kinect.width/2 , kinect.height/2);//corrected and masked depth image for plane above surface
        maskedDepthImgForPlaneAboveObjects.draw(kinect.width/2+offset,kinect.height/2, kinect.width/2 , kinect.height/2);//corrected and masked depth image for plane above objects
        
        ofPushStyle();
        ofSetColor(0, 255, 0);//green
        ofFill();
        //draw detected fingers (green dots) over the corrected and masked depth image for plane above surface
        for(vector<DetectedFinger *>::iterator it = detectedFingers.begin(); it != detectedFingers.end(); it++)
        {
            cv::Point2i p = (*it)->getExtremity();
            ofCircle( p.x /2 + offset , kinect.height/2 + p.y/2, 4);
        }
        ofPopStyle();
        
        ofPushStyle();
        ofSetColor(0, 255, 255);
        ofCircle(kinect.width/2 + centerTabletop->x /2  + offset, centerTabletop->y/2, radiusForPlaneDetection);//pixels used in the plane detection are represented on the real color image
        ofPopStyle();
        
        maskedDepthImgForSurfacePlane.draw(0+ offset, kinect.height, kinect.width/2 , kinect.height/2);//base
        blobfinder.draw(0+ offset, kinect.height, kinect.width/2 , kinect.height/2);//blobs over base
    }
    else if(screenChoice == SCREEN3)
    {
        ofPushMatrix();
        vector<ofxCvBlob> blobs;
        list<TuioObject*> objs;
        int treedepth = 50;
        int iterations = 20;//limitation of iterations
     
        perspectiveDepthImgForPlaneAboveObjects.draw(0,0,kinect.getWidth(), kinect.getHeight());//draw base
        
        blobfinder.draw(0, 0, kinect.getWidth(), kinect.getHeight());//draw blobs
        blobs = blobfinder.blobs;
        
        kinect.draw(300+ kinect.getWidth()/2,kinect.getHeight(), kinect.width/2 , kinect.height/2);//draw real Kinect image
        
        std::list<TuioObject*> objectList = client.client->getTuioObjects();//TUIO objects
        list<TuioObject*>::iterator tobj;
        client.client->lockObjectList();
        //Draw TUIO objects
        for (tobj=objectList.begin(); tobj != objectList.end(); tobj++)
        {
            TuioObject *obj = (*tobj);
            glColor3f(1.0,0.0,0.0);
            glPushMatrix();
            glTranslatef(kinect.getWidth() - obj->getX()*kinect.getWidth(), kinect.getHeight()- (obj->getY()* kinect.getHeight()), 0.0);
            glRotatef(obj->getAngleDegrees(), 0.0, 0.0, 1.0);
            ofRect(-10.0, -10.0, 20.0, 20.0);//oriented rectangle
            glColor3f(1.0,1.0,1.0);
            ofLine(0, 0, 0, -10);//line showing orientation
            glPopMatrix();
            string str = "SymbolId: "+ofToString((int)(obj->getSymbolID()));
            ofDrawBitmapString(str,kinect.getWidth() -  obj->getX()*kinect.getWidth()-10.0, kinect.getHeight()-obj->getY()* kinect.getHeight()+25.0);
            str = "SessionId: "+ofToString((int)(obj->getSessionID()));
            ofDrawBitmapString(str, kinect.getWidth() - obj->getX()*kinect.getWidth()-10.0, kinect.getHeight()-obj->getY()* kinect.getHeight()+40.0);
            vector< TokenData*> temp = tokensData.getObjects();
            //find token data
            for(vector< TokenData *>::iterator ito=temp.begin() ; ito < temp.end(); ito++ )
            {
                if((*ito)->getID()==obj->getSymbolID())
                {
                    str = "DepthMin: "+ofToString((int)((*ito)->zmin));
                    ofDrawBitmapString(str, kinect.getWidth() - obj->getX()*kinect.getWidth()-10.0, kinect.getHeight()-obj->getY()* kinect.getHeight()+60.0);
                    str = "DepthCurrent: "+ofToString((int)((*ito)->zcurrent));
                    ofDrawBitmapString(str, kinect.getWidth() -  obj->getX()*kinect.getWidth()-10.0, kinect.getHeight()-obj->getY()* kinect.getHeight()+80.0);
                    str = "DepthMax: "+ofToString((int)((*ito)->zmax));
                    ofDrawBitmapString(str,kinect.getWidth() -  obj->getX()*kinect.getWidth()-10.0, kinect.getHeight()-obj->getY()* kinect.getHeight()+100.0);
                }
            }
        }
        client.client->unlockObjectList();
        //draw connections between blobs and TUIO objects
        objs = client.client->getTuioObjects();
        for (list<TuioObject*>::iterator tobj=objs.begin(); tobj != objs.end(); tobj++)
        {
            TuioObject *obj = (*tobj);
            for(vector<ofxCvBlob>::iterator it = blobs.begin(); it != blobs.end(); it++)
            {
                ofRectangle bounds = (*it).boundingRect;//get blob bounds
                bounds.set(bounds.x - 15, bounds.y - 15, bounds.width + 30, bounds.height +30);//increase size of bounds
                if(bounds.inside(kinect.getWidth() - obj->getX()*kinect.getWidth(), kinect.getHeight()- (obj->getY()* kinect.getHeight())))//is TUIO object inside blob?
                {
                    ofPushStyle();
                    ofSetColor(0,255,255);
                    ofSetLineWidth(3);
                    ofLine((*it).centroid.x, (*it).centroid.y, kinect.getWidth() - obj->getX()*kinect.getWidth(), kinect.getHeight()- (obj->getY()* kinect.getHeight()));//connection
                    ofPopStyle();
                }
            }
        }
        
        ofPopMatrix;
    }
   
    else if (screenChoice==SCREEN4)
    {
        drawtCAD();//draw 3D Scene, tokens UIs, etc.
    }
    else if (screenChoice==SCREEN5)
    {
        drawCalibrationFingers();
    }
    else if (screenChoice==SCREEN6)
    {
        drawCalibrationMarker();
    }
}
//--------------------------------------------------------------
void testApp::exit()
{
    kinect.close();
}

//--------------------------------------------------------------
//GUIs

void testApp::guiSetupEvent(ofxUIEventArgs &e)
{
    string name = e.widget->getName();
    int kind = e.widget->getKind();
    
    if(name == "RADIUS")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        radiusForPlaneDetection = slider->getScaledValue();
    }
    else if(name == "DIST FOR OBJECT PLANE")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        distanceForPlaneAboveSurface = slider->getScaledValue();
    }
    else if(name == "DIST FOR SURFACE PLANE")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        distanceForPlaneAboveObjects = slider->getScaledValue();
    }
    else if(name == "LOCK PLANE")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        lockPlane = true;
    }
    else if(name == "LOCK CORNERS")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        lockCorners = true;
    }
    else if(name == "RECALIBRATE")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        findingPlanes = true;
        distanceForPlaneAboveSurface=31.00;
        distanceForPlaneAboveObjects=52.00;
        addNormalX = 0;
        addNormalY = 0;
        addNormalZ = 0;
        unsetGUI();
        setGUISETUP();
    }
    else if(name == "minBlobSize")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        minBlobSize = slider->getScaledValue();
    }
    else if(name == "maxBlobSize")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        maxBlobSize = slider->getScaledValue();
    }
    else if(name == "numBlobs")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        numBlobs = slider->getScaledValue();
    }
    else if(name == "LOAD XML")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        if(loadXML())//load setup configuration from XML file
        {
            findingPlanes = false;//has a plane, so turn this false
            lockPlane = false;
            unsetGUI();
            setGUISETUP();//reload GUI
        }
        else
        {
        };
    }
    else if(name == "SAVE XML")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        saveXML();
    }
    else if(name == "SAVE XML")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        saveXML();
    }
    else if(name == "-X")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        addNormalX = addNormalX - 0.01;
    }
    else if(name == "+X")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        addNormalX = addNormalX +0.01;
    }
    else if(name == "-Y")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        addNormalY = addNormalY -0.01;
    }
    else if(name == "-Y")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        addNormalY = addNormalY +0.01;
    }
    else if(name == "-Z")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        addNormalZ = addNormalZ -0.01;
    }
    else if(name == "-Z")
    {
        ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
        addNormalZ = addNormalZ +0.01;
    }
}

void testApp::setGUISETUP()
{
    float dim = 16;
    float xInit = OFX_UI_GLOBAL_WIDGET_SPACING;
    float length = 255-xInit;
    gui = new ofxUICanvas(0, 0, length+xInit, ofGetHeight());
    if(findingPlanes)//if does not have a plane?
    {
        gui->addWidgetDown(new ofxUILabel("PANEL 1: LOAD", OFX_UI_FONT_LARGE));
        gui->addWidgetDown(new ofxUISpacer(length-xInit, 2));
        gui->addWidgetDown(new ofxUILabelButton(false, "LOAD XML", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabel("PANEL 2: PLANE", OFX_UI_FONT_LARGE));
        gui->addWidgetDown(new ofxUISpacer(length-xInit, 2));
        gui->addWidgetDown(new ofxUISlider(length-xInit,dim,5, 60, radiusForPlaneDetection, "RADIUS"));
        gui->addWidgetDown(new ofxUILabelButton(false, "LOCK PLANE", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabelButton(false, "LOCK CORNERS", OFX_UI_FONT_MEDIUM));
    }
    else
    {
        gui->addWidgetDown(new ofxUILabel("PANEL 1: SAVE", OFX_UI_FONT_LARGE));
        gui->addWidgetDown(new ofxUILabelButton(false, "LOAD XML", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUISpacer(length-xInit, 2));
        gui->addWidgetDown(new ofxUILabelButton(false, "SAVE XML", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabel("PANEL 2: PLANE", OFX_UI_FONT_LARGE));
        gui->addWidgetDown(new ofxUISpacer(length-xInit, 2));
        gui->addWidgetDown(new ofxUILabelButton(false, "RECALIBRATE", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabelButton(false, "LOCK CORNERS", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUISlider(length-xInit,dim,- 500, 500, distanceForPlaneAboveObjects, "DIST FOR SURFACE PLANE"));
        gui->addWidgetDown(new ofxUISlider(length-xInit,dim,- 500, 500, distanceForPlaneAboveSurface, "DIST FOR OBJECT PLANE"));
        gui->addWidgetDown(new ofxUILabelButton(true, "-X", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabelButton(true, "+X", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabelButton(true, "-Y", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabelButton(true, "+Y", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabelButton(true, "-Z", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabelButton(true, "+Z", OFX_UI_FONT_MEDIUM));
        gui->addWidgetDown(new ofxUILabel("PANEL 3: BLOBS", OFX_UI_FONT_LARGE));
        gui->addWidgetDown(new ofxUISpacer(length-xInit, 2));
        gui->addWidgetDown(new ofxUISpacer(length-xInit, 2));
        gui->addWidgetDown(new ofxUISlider(length-xInit,dim,1, 5000, minBlobSize, "minBlobSize"));
        gui->addWidgetDown(new ofxUISlider(length-xInit,dim,5000, (640*480)/3, maxBlobSize, "maxBlobSize"));
        gui->addWidgetDown(new ofxUISlider(length-xInit,dim,2, 100, numBlobs, "numBlobs"));
    }
    ofAddListener(gui->newGUIEvent,this,&testApp::guiSetupEvent);//set event handler
    guiChoice = GUISETUP;//set GUI type
}

void testApp::setGUIKINECT()
{
    kinectScan->kinect_DrawVariables = kinectDraw_Variables;//set Kinect Scanning Mode variables
    float dim = 16;
    float xInit = OFX_UI_GLOBAL_WIDGET_SPACING;
    float length = 255-xInit;
    gui = new ofxUICanvas(0, 0, length+xInit, ofGetHeight());
    gui->addWidgetDown(new ofxUILabel("PANEL 1: Settings", OFX_UI_FONT_LARGE));
    gui->addWidgetDown(new ofxUISpacer(length-xInit, 2));
    gui->addWidgetDown(new ofxUISlider(length-xInit,dim,20, 1000, kinectDraw_Variables.zCutoff, "zCutoff"));
    gui->addWidgetDown(new ofxUISlider(length-xInit,dim,0,1, kinectDraw_Variables.fovWidth, "fovWidth"));
    gui->addWidgetDown(new ofxUISlider(length-xInit,dim,0,1, kinectDraw_Variables.fovHeight, "fovHeight"));
    gui->addWidgetDown(new ofxUIToggle((length-xInit)/5,dim, kinectDraw_Variables.useSmoothing, "useSmoothing"));
    if(kinectDraw_Variables.useSmoothing)
    {
        gui->addWidgetDown(new ofxUISlider(length-xInit,dim,0, 10, kinectDraw_Variables.smoothingAmount, "smoothingAmount"));
        gui->addWidgetDown(new ofxUISlider(length-xInit,dim,0, 4, kinectDraw_Variables.backOffset, "backOffset"));
    }
    ofAddListener(gui->newGUIEvent,this,&testApp::guiKinectEvent);
    guiChoice = GUIKINECT;
}

void testApp::guiKinectEvent(ofxUIEventArgs &e)
{
    string name = e.widget->getName();
    int kind = e.widget->getKind();
    if(name == "zCutoff")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        kinectDraw_Variables.zCutoff = slider->getScaledValue();
    }
    else if(name == "fovWidth")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        kinectDraw_Variables.fovWidth = slider->getScaledValue();
    }
    else if(name == "fovHeight")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        kinectDraw_Variables.fovHeight = slider->getScaledValue();
    }
    else if(name == "smoothingAmount")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        kinectDraw_Variables.smoothingAmount = slider->getScaledValue();
    }
    else if(name == "backOffset")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        kinectDraw_Variables.backOffset = slider->getScaledValue();
    }
    else if(name == "useSmoothing")
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        kinectDraw_Variables.useSmoothing = toggle->getValue();
        unsetGUI();
        showKinectGUI = true;
        kinectScan->changeColor(new ofColor(255,255,255));
    }
    kinectScan->kinect_DrawVariables = kinectDraw_Variables ;
}

void testApp::unsetGUI()
{
    if(gui!=NULL)
    {
        gui->disable();
        guiChoice = NOGUI;
    }
}
//--------------------------------------------------------------
//Lines
void testApp::resetLines()
{
    //reset lines into Kinect depth image edges
    cv::Point pt1,pt2;
    pt1.x = 0;
    pt1.y = kinect.getWidth();
    
    pt2.x = kinect.getWidth();
    pt2.y = kinect.getHeight();
    
    topLine = new lineGeom(kinect.getHeight(),(float) 90*PI/180, pt1, pt2);
    
    pt1.x = 0;
    pt1.y = 0;
    
    pt2.x = 0;
    pt2.y = kinect.getHeight();
    
    leftLine = new lineGeom(0,0, pt1, pt2);
    
    pt1.x = 0;
    pt1.y = 0;
    
    pt2.x = kinect.getWidth();
    pt2.y = 0;
    
    bottomLine = new lineGeom(0,(float) 90*PI/180, pt1, pt2);
    
    pt1.x = kinect.getWidth();
    pt1.y = 0;
    
    pt2.x = kinect.getWidth();
    pt2.y = kinect.getHeight();
    
    rightLine = new lineGeom(kinect.getWidth(),0, pt1, pt2);
    
    centerTabletop->x=320;
    centerTabletop->y=240;
}
void testApp::findLines(IplImage* asrc)
{
    //receives Kinect depth image
    IplImage* src = new IplImage(*asrc);
    IplImage* dst = cvCreateImage( cvGetSize(src), 8, 1 );
    IplImage* color_dst = cvCreateImage( cvGetSize(src), 8, 3 );
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* lines = 0;
    
    cvCanny( src, dst, 50, 200, 3 );//canny filter
    cvCvtColor( dst, color_dst, CV_GRAY2BGR );
    
    lines = cvHoughLines2( dst,
                          storage,
                          CV_HOUGH_STANDARD,
                          1,
                          CV_PI/180,
                          100,
                          0,
                          0 );//Hough Lines filter
    
    //store previous lines
    lineGeom *previousLeftLine = leftLine;
    lineGeom *previousRightLine = rightLine;
    lineGeom *previousTopLine = topLine;
    lineGeom *previousBottomLine = bottomLine;
    
    resetLines();//reset lines
    
    CvPoint pt1,pt2;
    pt1.x = 0;
    pt1.y = kinect.getHeight()/2;
    pt2.x = kinect.getWidth();
    pt2.y =  kinect.getHeight()/2;
    lineGeom *horizontalCenter= new lineGeom(kinect.getHeight()/2,(float) 90*PI/180, pt1, pt2);//horizontal line passing through center
    pt1.x = kinect.getWidth()/2;
    pt1.y = 0;
    pt2.x = kinect.getWidth()/2;
    pt2.y = kinect.getHeight();
    lineGeom *verticalCenter = new lineGeom(kinect.getWidth()/2,0, pt1, pt2);//vertical line passing through center

    
    bool bottomChanged = false;
    bool topChanged = false;
    bool leftChanged = false;
    bool rightChanged = false;
    
    //passthrough lines from Hough lines filter, searching for tabletop edge lines
    for(int i = 0; i < MIN(lines->total,100); i++ )
    {
        float* line = (float*)cvGetSeqElem(lines,i);
        float rho = line[0];
        float theta = line[1];
        CvPoint pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cvLine( color_dst, pt1, pt2, CV_RGB(255,255,255), 3, 8 );
        lineGeom * lg = new lineGeom(rho, theta, pt1, pt2);
        cv::Point *ptHorizontalIntersect = new cv::Point();
        cv::Point *ptVerticalIntersect = new cv::Point();
        if(lg->intersection(horizontalCenter, ptHorizontalIntersect))//find intersection of line with horizontal line
        {
            if(ptHorizontalIntersect->x < kinect.getWidth()/2)
            {
                if(leftLine->resultFromY(kinect.getHeight()/2) < ptHorizontalIntersect->x)//line is in the left of the image
                {
                    leftLine=lg;
                    leftChanged = true;
                }
                
            }
            else
            {
                
                if(rightLine->resultFromY(kinect.getHeight()/2) > ptHorizontalIntersect->x)//line is in the right of the image
                {
                    rightLine = lg;
                    rightChanged = true;
                }
                
            }
        }
        else
        {
            //top or bottom
        }
        
        if(lg->intersection(verticalCenter, ptVerticalIntersect))//find intersection of line with vertical line 
        {
            if(ptVerticalIntersect->y > kinect.getHeight()/2)
            {
                if(topLine->resultFromX(kinect.getWidth()/2) > ptVerticalIntersect->y)//line is in the top of the image
                {
                    topLine=lg;
                    topChanged = true;
                }
            }
            else
            {
                if(bottomLine->resultFromX(kinect.getWidth()/2) < ptVerticalIntersect->y)//line is in the bottom of the image
                {
                    bottomLine=lg;
                    bottomChanged=true;
                }
            }
        }
        else
        {
            //left or right
        }
    }
    
    if(!rightChanged)//right line was not found
    {
        rightLine = previousRightLine;
    }
    if(!leftChanged)//left line was not found
    {
        leftLine = previousLeftLine;
    }
    if(!bottomChanged)//bottom line was not found
    {
        bottomLine = previousBottomLine;
    }
    if(!topChanged)//top line was not found
    {
        topLine = previousTopLine;
    }
    
    updateCorners();//intersect lines to find corners
    
    cvReleaseImage(&color_dst);
    cvReleaseMemStorage(&storage);
}
//--------------------------------------------------------------
void testApp::updateCorners()
{
    // Update ltCorner,lbCorner, rTCorner, rbCorner;
    if(leftLine->intersection(topLine,ltCorner))
    {
        //updated left top corner
        *ltCorner = ltCornerKalman->correct(*ltCorner);
    }
    if(leftLine->intersection(bottomLine,lbCorner))
    {
        //updated left bottom corner
        *lbCorner = lbCornerKalman->correct(*lbCorner);
    }
    if(rightLine->intersection(topLine,rtCorner))
    {
        //updated right top corner
        *rtCorner = rtCornerKalman->correct(*rtCorner);
    }
    if(rightLine->intersection(bottomLine,rbCorner))
    {
        //updated right top corner
        *rbCorner = rbCornerKalman->correct(*rbCorner);
    }
    ofVec2f pta = ofVec2f(ltCorner->x,ltCorner->y);
    ofVec2f ptb = ofVec2f(rbCorner->x, rbCorner->y);
    ofVec2f ptc = ofVec2f(rtCorner->x,rtCorner->y);
    ofVec2f ptd = ofVec2f(lbCorner->x,lbCorner->y);
    LineSegment cross1 = LineSegment(pta,ptb);
    LineSegment cross2 = LineSegment(ptc,ptd);
    ofVec2f intersect;
    if( cross1.Intersect(cross2, intersect)==LineSegment::INTERESECTING)
    {
        centerTabletop->x=intersect.x;
        centerTabletop->y=intersect.y;
    };
}
void testApp::showCorners(int offsetX, int offsetY)
{
    ofPushStyle();
    ofSetColor(255, 0, 0);
    ofSetLineWidth(3);
    ofLine(ltCorner->x/2+ offsetX,ltCorner->y/2+ offsetY, rtCorner->x/2+ offsetX,  rtCorner->y/2+ offsetY);
    ofLine(rtCorner->x/2+ offsetX, rtCorner->y/2+ offsetY, rbCorner->x/2+ offsetX, rbCorner->y/2+ offsetY);
    ofLine(rbCorner->x/2+ offsetX, rbCorner->y/2+ offsetY, lbCorner->x/2+ offsetX, lbCorner->y/2+ offsetY);
    ofLine(lbCorner->x/2+ offsetX, lbCorner->y/2+ offsetY, ltCorner->x/2+ offsetX, ltCorner->y/2+ offsetY);
    ofSetColor(0, 255, 0);
    ofSetLineWidth(1);
    ofLine(ltCorner->x/2+ offsetX,ltCorner->y/2+ offsetY, rbCorner->x/2+ offsetX,  rbCorner->y/2+ offsetY);
    ofLine(rtCorner->x/2+ offsetX, rtCorner->y/2+ offsetY, lbCorner->x/2+ offsetX, lbCorner->y/2+ offsetY);
    ofSetColor(0, 0, 255);
    ofCircle(centerTabletop->x/2+ offsetX, centerTabletop->y/2+ offsetY, 2);
    ofPopStyle();
}
//--------------------------------------------------------------
void testApp::getDepthImgForPlanes()
{
    ofxCvGrayscaleImage temp1 = originalDepthImage;//for surface plane
    ofxCvGrayscaleImage temp2 = originalDepthImage;//for plane above the objects
    
    unsigned char * pixels1 = temp1.getPixels();
    unsigned char * pixels2 = temp2.getPixels();
    for (int i = 0; i < kinect.getHeight(); i++)
    {
        for (int j = 0; j < kinect.getWidth(); j++)
        {
            int pix = (i*kinect.width) + j;//location on pixel matrix
            ofVec3f p =  kinect.getWorldCoordinateAt(j,i,kinect.getDistanceAt(j,i));//get depth value
            Vector3d pt = Vector3d(p.x,p.y,p.z);
            if(kinect.getDistanceAt(j,i) > 0)//ignore infinite depth values
            {
                //Mask surface plane
                if(planeAboveSurface.ClassifyPoint(pt)== CP_BACK)
                {
                    pixels1[pix]=255;
                }
                else if(planeAboveSurface.ClassifyPoint(pt)== CP_ONPLANE)
                {
                    pixels1[pix]=0;
                }
                else if(planeAboveSurface.ClassifyPoint(pt)== CP_FRONT)
                {
                    pixels1[pix]=0;
                }
                
                //Mask plane above objects
                if(planeAboveObjects.ClassifyPoint(pt)== CP_BACK)
                {
                    pixels2[pix]=255;
                }
                else if(planeAboveObjects.ClassifyPoint(pt)== CP_ONPLANE)
                {
                    pixels2[pix]=0;
                }
                else if(planeAboveObjects.ClassifyPoint(pt)== CP_FRONT)
                {
                    pixels2[pix]=0;
                }
            }
            else
            {
                pixels1[pix]=0;
                pixels2[pix]=0;
            }
        }
    }
    
    //update images with masked pixels
    depthImgForSurfacePlane.setFromPixels(pixels1, kinect.getWidth(), kinect.getHeight());
    depthImgForSurfacePlane.updateTexture();
    
    depthImgForPlaneAboveObjects.setFromPixels(pixels2, kinect.getWidth(), kinect.getHeight());
    depthImgForPlaneAboveObjects.updateTexture();
}
void testApp::getDepthImgForPlanesAlternative()
{
    ofxCvGrayscaleImage temp1 = originalDepthImage;//for surface plane
    ofxCvGrayscaleImage temp2 = originalDepthImage;//for plane above the objects
    unsigned char * pixels1 = temp1.getPixels();
    unsigned char * pixels2 = temp2.getPixels();
    for (int i = 0; i < kinect.getHeight(); i++)
    {
        for (int j = 0; j < kinect.getWidth(); j++)
        {
            int pix = (i*kinect.width) + j;//location on pixel matrix
            ofVec3f p =  kinect.getWorldCoordinateAt(j,i,kinect.getDistanceAt(j,i));//get depth value
            Vector3d pt = Vector3d(p.x,p.y,p.z);
            if(kinect.getDistanceAt(j,i) > 0)//ignore infinite depth values
            {
                 //Mask surface plane
                if(planeAboveSurface.ClassifyPoint(pt)== CP_BACK)
                {
                    pixels1[pix]=0;
                }
                else if(planeAboveSurface.ClassifyPoint(pt)== CP_ONPLANE)
                {
                    pixels1[pix]=0;
                }
                else if(planeAboveSurface.ClassifyPoint(pt)== CP_FRONT)
                {
                    pixels1[pix]=255;
                }
                
                //Mask plane above objects
                if(planeAboveObjects.ClassifyPoint(pt)== CP_BACK)
                {
                    pixels2[pix]=0;
                }
                else if(planeAboveObjects.ClassifyPoint(pt)== CP_ONPLANE)
                {
                    pixels2[pix]=0;
                }
                else if(planeAboveObjects.ClassifyPoint(pt)== CP_FRONT)
                {
                    pixels2[pix]=255;
                }
            }
            else
            {
                pixels1[pix]=0;
                pixels2[pix]=0;
            }
        }
    }
     //update images with masked pixels
    depthImgForSurfacePlane.setFromPixels(pixels1, kinect.getWidth(), kinect.getHeight());
    depthImgForSurfacePlane.updateTexture();
    
    depthImgForPlaneAboveObjects.setFromPixels(pixels2, kinect.getWidth(), kinect.getHeight());
    depthImgForPlaneAboveObjects.updateTexture();
}

void testApp::drawKinectPointCloud()
{
    int w = kinect.width;
    int h = kinect.height;
    
    ofMesh mesh;//kinect point cloud
    mesh.setMode(OF_PRIMITIVE_POINTS);
    ofMesh mesh2;
    mesh2.setMode(OF_PRIMITIVE_POINTS);
    
    int step = 3;//draw a pixel in three pixels
    
    for(int y = 0; y < h; y += step)
    {
        for(int x = 0; x < w; x += step)
        {
            if(kinect.getDistanceAt(x, y) > 0)
            {
                if(centerTabletop->distance(ofPoint(x,y))<= radiusForPlaneDetection)
                {
                    mesh.addColor(ofFloatColor(255,0,0));//Red circle
                }
                else
                {
                    mesh.addColor(kinect.getColorAt(x,y));//Kinect point cloud
                }
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    glEnable(GL_DEPTH_TEST);
    mesh.drawVertices();
    glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
    
    ofPushMatrix();
    for(int y = -h; y < h; y += step)
    {
        for(int x = -w; x < w; x += step)
        {
            mesh2.addColor(ofFloatColor(0,255,0));//green
            float z = -(distancePlane +normalUnitVec[0]*x +normalUnitVec[1]*y)/normalUnitVec[2];//convert normal and distance to z
            ofVec3f v;
            v.x = x;
            v.y = y;
            v.z= z;
            mesh2.addVertex(v);
            
            if(!findingPlanes)//if the plane above the surface and the plane above the objects ha been found then
            {
                mesh2.addColor(ofFloatColor(0,255,255));//cyan
                z = -(distancePlane + distanceForPlaneAboveSurface + (normalUnitVec[0]+addNormalX)*x +(normalUnitVec[1]+addNormalY)*y)/(normalUnitVec[2]+addNormalZ);//convert normal and distance to z

                v.x = x;
                v.y = y;
                v.z = z;
                mesh2.addVertex(v);
                
                mesh2.addColor(ofFloatColor(255,0,255));//magenta
                z = -(distancePlane + distanceForPlaneAboveObjects + (normalUnitVec[0]+addNormalX)*x +(normalUnitVec[1]+addNormalY)*y)/(normalUnitVec[2]+addNormalZ);//convert normal and distance to z

                v.x = x;
                v.y = y;
                v.z = z;
                mesh2.addVertex(v);
            }
        }
    }
    
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1,-1);
    ofTranslate(0, 0, -1000); // center the points a bit
    glEnable(GL_DEPTH_TEST);
    mesh2.drawVertices();
    glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
};

bool testApp::getBestFitPlane(unsigned int vcount,
                              const float *points,
                              unsigned int vstride,
                              const float *weights,
                              unsigned int wstride,
                              float *plane)
{
    bool ret = false;
    ofVec3f kOrigin(0,0,0);
    float wtotal = 0;
    if ( 1 )
    {
        const char *source  = (const char *) points;
        const char *wsource = (const char *) weights;
        for (unsigned int i=0; i<vcount; i++)
        {
            const float *p = (const float *) source;
            float w = 1;
            if ( wsource )
            {
                const float *ws = (const float *) wsource;
                w = *ws; //
                wsource+=wstride;
            }
            kOrigin.x+=p[0]*w;
            kOrigin.y+=p[1]*w;
            kOrigin.z+=p[2]*w;
            wtotal+=w;
            source+=vstride;
        }
    }
    float recip = 1.0f / wtotal; // reciprocol of total weighting
    kOrigin.x*=recip;
    kOrigin.y*=recip;
    kOrigin.z*=recip;
    float fSumXX=0;
    float fSumXY=0;
    float fSumXZ=0;
    float fSumYY=0;
    float fSumYZ=0;
    float fSumZZ=0;
    if ( 1 )
    {
        const char *source  = (const char *) points;
        const char *wsource = (const char *) weights;
        for (unsigned int i=0; i<vcount; i++)
        {
            const float *p = (const float *) source;
            float w = 1;
            if ( wsource )
            {
                const float *ws = (const float *) wsource;
                w = *ws;
                wsource+=wstride;
            }
            ofVec3f kDiff;
            kDiff.x = w*(p[0] - kOrigin.x); // apply vertex weighting!
            kDiff.y = w*(p[1] - kOrigin.y);
            kDiff.z = w*(p[2] - kOrigin.z);
            fSumXX+= kDiff.x * kDiff.x; // sume of the squares of the differences.
            fSumXY+= kDiff.x * kDiff.y; // sume of the squares of the differences.
            fSumXZ+= kDiff.x * kDiff.z; // sume of the squares of the differences.
            fSumYY+= kDiff.y * kDiff.y;
            fSumYZ+= kDiff.y * kDiff.z;
            fSumZZ+= kDiff.z * kDiff.z;
            source+=vstride;
        }
    }
    fSumXX *= recip;
    fSumXY *= recip;
    fSumXZ *= recip;
    fSumYY *= recip;
    fSumYZ *= recip;
    fSumZZ *= recip;
    // setup the eigensolver
    Eigen kES;
    kES.mElement[0][0] = fSumXX;
    kES.mElement[0][1] = fSumXY;
    kES.mElement[0][2] = fSumXZ;
    kES.mElement[1][0] = fSumXY;
    kES.mElement[1][1] = fSumYY;
    kES.mElement[1][2] = fSumYZ;
    kES.mElement[2][0] = fSumXZ;
    kES.mElement[2][1] = fSumYZ;
    kES.mElement[2][2] = fSumZZ;
    // compute eigenstuff, smallest eigenvalue is in last position
    kES.DecrSortEigenStuff();
    ofVec3f kNormal;
    kNormal.x = kES.mElement[0][2];
    kNormal.y = kES.mElement[1][2];
    kNormal.z = kES.mElement[2][2];
    // the minimum energy
    plane[0] = kNormal.x;
    plane[1] = kNormal.y;
    plane[2] = kNormal.z;
    plane[3] = 0 - kNormal.dot(kOrigin);
    return ret;
}

void testApp::perspectiveCorrectionDepthValues()
{
    ofxCvGrayscaleImage temp = depthImgForSurfacePlane;
    unsigned char * pixels = temp.getPixels();
    cv::Mat src(cv::Size(640,480), CV_32FC1);
    
    //fill matrix with depth values
    for (int i = 0; i < kinect.getHeight(); i++)
    {
        for (int j = 0; j < kinect.getWidth(); j++)
        {
            src.at<float>(i,j) = kinect.getDistanceAt(j,i);
        }
    }
    
    cv::Mat outmat(cv::Size(640,480), CV_32FC1);//result of perspective correction
    
    cv::Point2f *c1 = new cv::Point2f[4];//pixels destinations
    cv::Point2f *c2 = new cv::Point2f[4];//pixels origin
    
    c1[0].x = lbCorner->x;
    c1[0].y = lbCorner->y;
    c1[1].x = rbCorner->x;
    c1[1].y = rbCorner->y;
    c1[2].x = ltCorner->x;
    c1[2].y = ltCorner->y;
    c1[3].x = rtCorner->x;
    c1[3].y = rtCorner->y;
    c2[0].x = 0;
    c2[0].y = 0;
    c2[1].x = 640;
    c2[1].y = 0;
    c2[2].x = 0;
    c2[2].y = 480;
    c2[3].x = 640;
    c2[3].y = 480;
    
    cv::Size size (640,480);
    cv:: Mat mmat(cv::Size(3,3),CV_32FC1);
    
    mmat = getPerspectiveTransform(c1, c2);//matrix with perspective transform
    
    warpPerspective(src, outmat, mmat, size);//apply perspective transform to src matrix
    
    //fill matrix with corrected depth values
    for (int i = 0; i < kinect.getHeight(); i++)
    {
        for (int j = 0; j < kinect.getWidth(); j++)
        {
            int pix = (i*kinect.width) + j;
            correctedDepthValues[pix]= outmat.at<float>(i,j);
        }
    }
}

void testApp::perspectiveCorrectionSurfacePlane()
{
    ofxCvGrayscaleImage temp = depthImgForSurfacePlane;
    unsigned char * pixels = temp.getPixels();
    cv::Mat src(cv::Size(640,480), CV_8UC1);
    //fill matrix with pixels values
    for (int i = 0; i < kinect.getHeight(); i++)
    {
        for (int j = 0; j < kinect.getWidth(); j++)
        {
            int pix = (i*kinect.width) + j;
            src.at<uchar>(i,j) = pixels[pix];
        }
    }
    
    cv::Mat outmat(cv::Size(640,480), CV_8UC1);//result of perspective correction
    
    cv::Point2f *c1 = new cv::Point2f[4];//pixels destinations
    cv::Point2f *c2 = new cv::Point2f[4];//pixels origin
    
    c1[0].x = lbCorner->x;
    c1[0].y = lbCorner->y;
    c1[1].x = rbCorner->x;
    c1[1].y = rbCorner->y;
    c1[2].x = ltCorner->x;
    c1[2].y = ltCorner->y;
    c1[3].x = rtCorner->x;
    c1[3].y = rtCorner->y;
    c2[0].x = 0;
    c2[0].y = 0;
    c2[1].x = 640;
    c2[1].y = 0;
    c2[2].x = 0;
    c2[2].y = 480;
    c2[3].x = 640;
    c2[3].y = 480;
    cv::Size size (640,480);
    
    cv:: Mat mmat(cv::Size(3,3),CV_32FC1);
    mmat = getPerspectiveTransform(c1, c2);//matrix with perspective transform
    
    warpPerspective(src, outmat, mmat, size);//apply perspective transform to src matrix
    
    IplImage * ip = new IplImage (outmat);
    perspectiveDepthImgForSurfacePlane = ip;//reload image with new pixels
    perspectiveDepthImgForSurfacePlane.updateTexture();
}

void testApp::perspectiveCorrectionPlaneAboveObjects()
{
    ofxCvGrayscaleImage temp = depthImgForPlaneAboveObjects;
    unsigned char * pixels = temp.getPixels();
    cv::Mat src(cv::Size(640,480), CV_8UC1);
    
    //fill matrix with pixels values
    for (int i = 0; i < kinect.getHeight(); i++)
    {
        for (int j = 0; j < kinect.getWidth(); j++)
        {
            int pix = (i*kinect.width) + j;
            src.at<uchar>(i,j) = pixels[pix];
        }
    }
    
    cv::Mat outmat(cv::Size(640,480), CV_8UC1);//result of perspective correction
    
    cv::Point2f *c1 = new cv::Point2f[4];//pixels destinations
    cv::Point2f *c2 = new cv::Point2f[4];//pixels origin
    
    c1[0].x = lbCorner->x;
    c1[0].y = lbCorner->y;
    c1[1].x = rbCorner->x;
    c1[1].y = rbCorner->y;
    c1[2].x = ltCorner->x;
    c1[2].y = ltCorner->y;
    c1[3].x = rtCorner->x;
    c1[3].y = rtCorner->y;
    c2[0].x = 0;
    c2[0].y = 0;
    c2[1].x = 640;
    c2[1].y = 0;
    c2[2].x = 0;
    c2[2].y = 480;
    c2[3].x = 640;
    c2[3].y = 480;
    
    cv::Size size (640,480);
    cv:: Mat mmat(cv::Size(3,3),CV_32FC1);
    
    mmat = getPerspectiveTransform(c1, c2);//matrix with perspective transform
    
    warpPerspective(src, outmat, mmat, size);//apply perspective transform to src matrix
    
    IplImage * ip = new IplImage (outmat);
    perspectiveDepthImgForPlaneAboveObjects = ip;//reload image with new pixels
    perspectiveDepthImgForPlaneAboveObjects.updateTexture();
}

void testApp::setupKalmanPlane()
{
    int dynam_params = 4;
    int measure_params = 4;
    m_pKalmanFilter = cvCreateKalman(dynam_params, measure_params);
    state = cvCreateMat( dynam_params, 1, CV_32FC1 );
    // random number generator stuff
    cvRandInit( &rng, 0, 1, -1, CV_RAND_UNI );
    cvRandSetRange( &rng, 0, 1, 0 );
    rng.disttype = CV_RAND_NORMAL;
    cvRand( &rng, state );
    process_noise = cvCreateMat( dynam_params, 1, CV_32FC1 ); // (w_k)
    measurement = cvCreateMat( measure_params, 1, CV_32FC1 ); // two parameters for x,y  (z_k)
    measurement_noise = cvCreateMat( measure_params, 1, CV_32FC1 ); // two parameters for x,y (v_k)
    cvZero(measurement);
    // F matrix data
    // F is transition matrix.  It relates how the states interact
    // For single input fixed velocity the new value
    // depends on the previous value and velocity- hence 1 0 1 0
    // on top line. New velocity is independent of previous
    // value, and only depends on previous velocity- hence 0 1 0 1 on second row
    const float F[] =
    {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    memcpy( m_pKalmanFilter->transition_matrix->data.fl, F, sizeof(F));
    cvSetIdentity( m_pKalmanFilter->measurement_matrix, cvRealScalar(1) ); // (H)
    cvSetIdentity( m_pKalmanFilter->process_noise_cov, cvRealScalar(1e-5) ); // (Q)
    cvSetIdentity( m_pKalmanFilter->measurement_noise_cov, cvRealScalar(1e-1) ); // (R)
    cvSetIdentity( m_pKalmanFilter->error_cov_post, cvRealScalar(1));
    // choose random initial state
    cvRand( &rng, m_pKalmanFilter->state_post );
}

vector<DetectedFinger *> testApp::detectFingers()
{
    //TABLE
    ofxCvGrayscaleImage temp = perspectiveDepthImgForPlaneAboveObjects;
    temp.invert();
    unsigned char * pixels = temp.getPixels();
    cv::Mat hand(cv::Size(640,480),CV_8UC1);
    for (int i = 0; i < kinect.getHeight(); i++)
    {
        for (int j = 0; j < kinect.getWidth(); j++)
        {
            int pix = (i*kinect.getWidth()) + j;
            hand.at<uchar>(i,j)= pixels[pix];
        }
    }
    vector<DetectedFinger *> detectedFingers;
    hand = hand < 225;
    IplImage * ip = new IplImage(hand);
    maskedDepthImgForSurfacePlane = ip;
    std::vector<std::vector<cv::Point > > contours;
    cv::findContours(hand, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // we are cloning here since method will destruct the image
    if (contours.size())
    {
        for (int i=0; i<contours.size(); i++)
        {
            vector<cv::Point> contour = contours[i];
            vector<ofPoint> contourforPoly = vector<ofPoint>();
            for(int i = 0; i< contour.size(); i++)
            {
                contourforPoly.push_back(ofVec2f(contour[i].x, contour[i].y));
            }
            ofPolyline poly = ofPolyline(contourforPoly);
            cv::Mat contourMat = cv::Mat(contour);
            double area = contourArea(contourMat);
            if (area > 400)    // possible hand
            {
                cv::Scalar center = mean(contourMat);
                cv::Point centerPoint = cv::Point(center.val[0], center.val[1]);
                vector<cv::Point> approxCurve;
                approxPolyDP(contourMat, approxCurve, 20, true);
                vector<int> hull;
                convexHull(cv::Mat(approxCurve), hull);
                // find upper and lower bounds of the hand and define cutoff threshold (don't consider upper vertices as fingers)
                int upper = 640, lower = 0;
                for (int j=0; j<hull.size(); j++)
                {
                    int idx = hull[j]; // corner index
                    if (approxCurve[idx].y < upper) upper = approxCurve[idx].y;
                    if (approxCurve[idx].y > lower) lower = approxCurve[idx].y;
                }
                float cutoff = upper - (upper - lower) * 0.1f;
                float cutoff2 = lower - (lower - upper) * 0.1f;
                // find interior angles of hull corners
                for (int j=0; j<hull.size(); j++)
                {
                    int idx = hull[j]; // corner index
                    int pdx = idx == 0 ? approxCurve.size() - 1 : idx - 1; //  predecessor of idx
                    int sdx = idx == approxCurve.size() - 1 ? 0 : idx + 1; // successor of idx
                    cv::Point v1 = approxCurve[sdx] - approxCurve[idx];
                    cv::Point v2 = approxCurve[pdx] - approxCurve[idx];
                    float angle = acos( (v1.x*v2.x + v1.y*v2.y) / (normCV(v1) * normCV(v2) ) );
                    if(approxCurve[idx].y > 10 && approxCurve[idx].x > 0 && approxCurve[idx].x < 630 && approxCurve[idx].y < 470)
                    {
                        // low interior angle + within upper 90% of region -> we got a finger
                        if (angle < 1 && approxCurve[idx].y > cutoff)
                        {
                            int u = approxCurve[idx].x;
                            int v = approxCurve[idx].y;
                            DetectedFinger * temp = new DetectedFinger();
                            temp->setExtremeity(cv::Point2i(u,v));
                            temp->getLimb()->setCenterLimb(centerPoint);
                            temp->getLimb()->setApproxCountour(contour);
                            temp->setFingerBase(cv::Point2i((approxCurve[pdx].x + approxCurve[sdx].x)/2, (approxCurve[pdx].y + approxCurve[sdx].y)/2));
                            int radius = 20;
                            std::vector<float> pts;
                            std::vector<float> pts2;
                            for(int x =  temp->getExtremity().x - radius/2; x <  temp->getExtremity().x + radius/2; x++)
                            {
                                for(int y = temp->getExtremity().y - radius/2; y <  temp->getExtremity().y + radius/2; y++)
                                {
                                    if( (x< kinect.width && x >= 0) && (y< kinect.height && y >= 0))
                                    {
                                        if(inside(x, y, poly))
                                        {
                                            unsigned char * pixels = maskedDepthImgForSurfacePlane.getPixels();
                                            int pix = (y*kinect.getWidth()) + x;
                                            if(pixels[pix] == 255)
                                            {
                                                if(correctedDepthValues[pix]!=0)
                                                {
                                                    ofVec3f p =  kinect.getWorldCoordinateAt(x,y,correctedDepthValues[pix]);
                                                    Vector3d pt = Vector3d(p.x,p.y,p.z);
                                                    pts.push_back(planeAboveObjects.DistanceTo(pt));//abs
                                                    pts2.push_back(planeAboveSurface.DistanceTo(pt));//abs
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            RollingAverage avg = RollingAverage(pts.size());
                            RollingAverage avg2 = RollingAverage(pts2.size());
                            for(int i= 0; i< pts.size(); i++)
                            {
                                avg.addSample(pts[i]);
                                avg2.addSample(pts[i]);
                            }
                            temp->setFingerDepthForSurfacePLane(avg.Average());
                            temp->setFingerDepthForObjectPlane(avg2.Average());
                            detectedFingers.push_back(temp);
                        }
                        else if (angle < 1 && approxCurve[idx].y < cutoff2)
                        {
                            int u = approxCurve[idx].x;
                            int v = approxCurve[idx].y;
                            DetectedFinger * temp = new DetectedFinger();
                            temp->setExtremeity(cv::Point2i(u,v));
                            temp->getLimb()->setCenterLimb(centerPoint);
                            temp->getLimb()->setApproxCountour(contour);
                            temp->setFingerBase(cv::Point2i((approxCurve[pdx].x + approxCurve[sdx].x)/2, (approxCurve[pdx].y + approxCurve[sdx].y)/2));
                            int radius = 20;
                            std::vector<float> pts;
                            std::vector<float> pts2;
                            for(int x =  temp->getExtremity().x - radius/2; x <  temp->getExtremity().x + radius/2; x++)
                            {
                                for(int y = temp->getExtremity().y - radius/2; y <  temp->getExtremity().y + radius/2; y++)
                                {
                                    if( (x< kinect.width && x >= 0) && (y< kinect.height && y >= 0))
                                    {
                                        if(inside(x, y, poly))
                                        {
                                            unsigned char * pixels = maskedDepthImgForSurfacePlane.getPixels();
                                            int pix = (y*kinect.getWidth()) + x;
                                            if(pixels[pix] == 255)
                                            {
                                                if(correctedDepthValues[pix]!=0)
                                                {
                                                    ofVec3f p =  kinect.getWorldCoordinateAt(x,y,correctedDepthValues[pix]);
                                                    Vector3d pt = Vector3d(p.x,p.y,p.z);
                                                    pts.push_back(planeAboveObjects.DistanceTo(pt));//abs
                                                    pts2.push_back(planeAboveSurface.DistanceTo(pt));//abs
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            RollingAverage avg = RollingAverage(pts.size());
                            RollingAverage avg2 = RollingAverage(pts2.size());
                            for(int i= 0; i< pts.size(); i++)
                            {
                                avg.addSample(pts[i]);
                                avg2.addSample(pts[i]);
                            }
                            temp->setFingerDepthForSurfacePLane(avg.Average());
                            temp->setFingerDepthForObjectPlane(avg2.Average());
                            detectedFingers.push_back(temp);
                        }
                    }
                }
            }
        }
    }
    //AIR
    ofxCvGrayscaleImage temp2 = perspectiveDepthImgForSurfacePlane;
    temp2.invert();
    unsigned char * pixels2 = temp2.getPixels();
    cv::Mat hand2(cv::Size(640,480),CV_8UC1);
    for (int i = 0; i < kinect.getHeight(); i++)
    {
        for (int j = 0; j < kinect.getWidth(); j++)
        {
            int pix = (i*kinect.getWidth()) + j;
            hand2.at<uchar>(i,j)= pixels2[pix];
        }
    }
    vector<DetectedFinger *> detectedFingers2;
    hand2 = hand2 < 225;
    IplImage * ip2 = new IplImage(hand2);
    maskedDepthImgForPlaneAboveObjects = ip2;
    //
    std::vector<std::vector<cv::Point> > contours2;
    cv::findContours(hand2, contours2, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE); // we are cloning here since method will destruct the image
    if (contours2.size())
    {
        for (int i=0; i<contours2.size(); i++)
        {
            vector<cv::Point> contour = contours2[i];
            vector<ofPoint> contourforPoly = vector<ofPoint>();
            for(int i = 0; i< contour.size(); i++)
            {
                contourforPoly.push_back(ofVec2f(contour[i].x, contour[i].y));
            }
            ofPolyline poly = ofPolyline(contourforPoly);
            cv::Mat contourMat = cv::Mat(contour);
            double area = contourArea(contourMat);
            if (area > 200)    // possible hand
            {
                cv::Scalar center = mean(contourMat);
                cv::Point centerPoint = cv::Point(center.val[0], center.val[1]);
                vector<cv::Point> approxCurve;
                approxPolyDP(contourMat, approxCurve, 20, true);
                vector<int> hull;
                convexHull(cv::Mat(approxCurve), hull);
                // find upper and lower bounds of the hand and define cutoff threshold (don't consider upper vertices as fingers)
                int upper = 640, lower = 0;
                for (int j=0; j<hull.size(); j++)
                {
                    int idx = hull[j]; // corner index
                    if (approxCurve[idx].y < upper) upper = approxCurve[idx].y;
                    if (approxCurve[idx].y > lower) lower = approxCurve[idx].y;
                }
                float cutoff = upper - (upper - lower) * 0.1f;
                float cutoff2 = lower - (lower - upper) * 0.1f;
                // find interior angles of hull corners
                for (int j=0; j<hull.size(); j++)
                {
                    int idx = hull[j]; // corner index
                    int pdx = idx == 0 ? approxCurve.size() - 1 : idx - 1; //  predecessor of idx
                    int sdx = idx == approxCurve.size() - 1 ? 0 : idx + 1; // successor of idx
                    cv::Point v1 = approxCurve[sdx] - approxCurve[idx];
                    cv::Point v2 = approxCurve[pdx] - approxCurve[idx];
                    float angle = acos( (v1.x*v2.x + v1.y*v2.y) / (normCV(v1) * normCV(v2) ) );
                    if(approxCurve[idx].y > 10 && approxCurve[idx].x > 0 && approxCurve[idx].x < 630 && approxCurve[idx].y < 470)
                    {
                        // low interior angle + within upper 90% of region -> we got a finger
                        if (angle < 1 && approxCurve[idx].y > cutoff)
                        {
                            int u = approxCurve[idx].x;
                            int v = approxCurve[idx].y;
                            DetectedFinger * temp = new DetectedFinger();
                            temp->setExtremeity(cv::Point2i(u,v));
                            temp->getLimb()->setCenterLimb(centerPoint);
                            temp->getLimb()->setApproxCountour(contour);
                            temp->setFingerBase(cv::Point2i((approxCurve[pdx].x + approxCurve[sdx].x)/2, (approxCurve[pdx].y + approxCurve[sdx].y)/2));
                            int radius = 20;
                            std::vector<float> pts;
                            std::vector<float> pts2;
                            for(int x =  temp->getExtremity().x - radius/2; x <  temp->getExtremity().x + radius/2; x++)
                            {
                                for(int y = temp->getExtremity().y - radius/2; y <  temp->getExtremity().y + radius/2; y++)
                                {
                                    if((x< kinect.width && x >0) && (y< kinect.height && y >0))
                                    {
                                        if(inside(x, y, poly))
                                        {
                                            unsigned char * pixels2 = maskedDepthImgForPlaneAboveObjects.getPixels();
                                            int pix = (y*kinect.getWidth()) + x;
                                            if(pixels2[pix] == 255)
                                            {
                                                if(correctedDepthValues[pix]!=0)
                                                {
                                                    ofVec3f p =  kinect.getWorldCoordinateAt(x,y,correctedDepthValues[pix]);
                                                    Vector3d pt = Vector3d(p.x,p.y,p.z);
                                                    pts.push_back(planeAboveObjects.DistanceTo(pt));//abs
                                                    pts2.push_back(planeAboveSurface.DistanceTo(pt));//abs
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            RollingAverage avg = RollingAverage(pts.size());
                            RollingAverage avg2 = RollingAverage(pts2.size());
                            for(int i= 0; i< pts.size(); i++)
                            {
                                avg.addSample(pts[i]);
                                avg2.addSample(pts[i]);
                            }
                            temp->setFingerDepthForSurfacePLane(avg.Average());
                            temp->setFingerDepthForObjectPlane(avg2.Average());
                            detectedFingers2.push_back(temp);
                        }
                        else if (angle < 1 && approxCurve[idx].y < cutoff2)
                        {
                            int u = approxCurve[idx].x;
                            int v = approxCurve[idx].y;
                            DetectedFinger * temp = new DetectedFinger();
                            temp->setExtremeity(cv::Point2i(u,v));
                            temp->getLimb()->setCenterLimb(centerPoint);
                            temp->getLimb()->setApproxCountour(contour);
                            temp->setFingerBase(cv::Point2i((approxCurve[pdx].x + approxCurve[sdx].x)/2, (approxCurve[pdx].y + approxCurve[sdx].y)/2));
                            int radius = 20;
                            std::vector<float> pts;
                            std::vector<float> pts2;
                            for(int x =  temp->getExtremity().x - radius/2; x <  temp->getExtremity().x + radius/2; x++)
                            {
                                for(int y = temp->getExtremity().y - radius/2; y <  temp->getExtremity().y + radius/2; y++)
                                {
                                    if((x< kinect.width && x >0) && (y< kinect.height && y >0))
                                    {
                                        if(inside(x, y, poly))
                                        {
                                            unsigned char * pixels2 = maskedDepthImgForPlaneAboveObjects.getPixels();
                                            int pix = (y*kinect.getWidth()) + x;
                                            if(pixels2[pix] == 255)
                                            {
                                                if(correctedDepthValues[pix]!=0)
                                                {
                                                    ofVec3f p =  kinect.getWorldCoordinateAt(x,y,correctedDepthValues[pix]);
                                                    Vector3d pt = Vector3d(p.x,p.y,p.z);
                                                    pts.push_back(planeAboveObjects.DistanceTo(pt));//abs
                                                    pts2.push_back(planeAboveSurface.DistanceTo(pt));//abs
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            RollingAverage avg = RollingAverage(pts.size());
                            RollingAverage avg2 = RollingAverage(pts2.size());
                            for(int i= 0; i< pts.size(); i++)
                            {
                                avg.addSample(pts[i]);
                                avg2.addSample(pts[i]);
                            }
                            temp->setFingerDepthForSurfacePLane(avg.Average());
                            temp->setFingerDepthForObjectPlane(avg2.Average());
                            detectedFingers2.push_back(temp);
                        }
                    }
                }
            }
        }
    }
    //MIX TABLE AND AIR
    vector<DetectedFinger *> detectedFingersFinal = vector<DetectedFinger *>();
    for(std::vector<DetectedFinger*>::iterator it = detectedFingers2.begin() ; it < detectedFingers2.end() ; it++)
    {
        DetectedFinger * temp = (*it);
        ofVec2f centertemp = ofVec2f(temp->getExtremity().x,temp->getExtremity().y);
        bool found = false;
        for(std::vector<DetectedFinger*>::iterator it2 = detectedFingers.begin() ; it2 < detectedFingers.end() && !found; it2++)
        {
            DetectedFinger * temp2 = (*it2);
            ofVec2f centertemp2 = ofVec2f(temp2->getExtremity().x,temp2->getExtremity().y);
            if(centertemp.distance(centertemp2) <= 50)
            {
                detectedFingers.erase(it2);
                detectedFingersFinal.push_back(temp2);
                found = true;
                break;
            }
        }
        if(!found)
        {
            detectedFingersFinal.push_back(temp);//fingertip above object
        }
    }
    for(std::vector<DetectedFinger*>::iterator it = detectedFingers.begin() ; it < detectedFingers.end() ; it++)
    {
        detectedFingersFinal.push_back((*it));
    }
    return detectedFingersFinal;
}

double testApp::normCV(cv::Point pt)
{
    return std::sqrt((double)pt.x*pt.x + (double)pt.y*pt.y);
}

void testApp::setuptCAD()
{
    ofEnableAlphaBlending();
    ofSetVerticalSync(true);
    //Set background color to grey
    ofBackground(COLORSCHEME_BACKGROUND_GREY);
    ofEnableSmoothing();
    //Setup tokens
    onTableCCToken = false;
    onTokenCCToken = false;
    inAirCCToken = false;
    saveToken = false;
    useAlternativeManipulationUI = false;
    calibratingFingers = false;
    calibratingMarker = false;
    hasShredder1 = false;
    hasShredder2 = false;
    hasOrbiter = false;
    //Setup 3D scene origin
    shapeEntryPoint = ofVec3f(0,0,0);
    //No Axis selected
    axis = new AxisPlane();
    //Link camera to axis
    cameraScene.selectedPlane = axis;
    cameraScene.cacheMatrices();
    //Setup viewport for camera
    viewport.x = 0;
    viewport.y = 0;
    viewport.width = 1280;//initial measurements, will be updated when application goes fullscreen
    viewport.height = 800;
    //Setup Lighting conditions
    ofSetSmoothLighting(true);
    light.setAmbientColor(ofColor(255, 255, 255));
    light.setDiffuseColor(ofColor(.0, .0, .0));
    light.setSpecularColor(ofColor(255, .1, .1));
    light.enable();
    //Initiate 3D shapes vector
    shapes = LockableVector<Shape3D*>();
    cameraScene.shapes = &shapes;
    //Setup Kinect Scanning parameters
    kinectDraw_Variables = kinectDrawVariables();
    //Setup Contour Mode area
    drawAreaForContour = ContourDrawArea(&shapeEntryPoint, &shapes, 0, 0, ofGetWidth(), ofGetHeight());
    //Add event listeners for TUIO client
    ofAddListener(client.objectAdded,this,&testApp::objectAdded);
    ofAddListener(client.objectRemoved,this,&testApp::objectRemoved);
    ofAddListener(client.objectUpdated,this,&testApp::objectUpdated);
    //Setup Kinect Scanning Mode
    kinectScan = new Basic3DObjectFromKinect(&shapeEntryPoint, &kinect);
    kinectScanning = false;
    showKinectGUI = false;
    //Load Images
    logo.loadImage("images/greylogo.png");
    cameraImg.loadImage("images/camera.png");
    font.loadFont("fonts/Helvetica.ttf", 14);
    //Set camera to starting position
    cameraScene.addRotationTween(0,45,0,1);
}

void testApp::drawtCAD()
{
    ofPushStyle();
    cameraScene.begin(viewport);
    drawScene();
    cameraScene.checkSelectable();
    if(kinectScanning)
    {
        kinectScan->update();
        kinectScan->draw();
    }
    cameraScene.end();
    ofPopStyle();
    ofPushStyle();
    shapeEntryPoint = cameraScene.cursor;
    ofPopStyle();
    ofPushStyle();
    vector<ContainerToken*> temp = containers.getObjects();
    containers.lockVector();
    bool found = false;
    for (std::vector<ContainerToken*>::iterator it=temp.begin() ; it < temp.end(); it++ )
    {
        (*it)->draw();
    }
    containers.unlockVector();
    ofPopStyle();
    ofPushStyle();
    vector<SelectionMovement*> temp3 = selectionMovements.getObjects();
    selectionMovements.lockVector();
    for (std::vector<SelectionMovement*>::iterator it=temp3.begin() ; it < temp3.end(); it++ )
    {
        (*it)->draw();
    }
    selectionMovements.unlockVector();
    ofPopStyle();
    ofPushStyle();
    if(hasShredder1 && hasShredder2)
    {
        vector<ofPoint> verts;
        verts.push_back(ofPoint(shredder1->getToken()->getX()*ofGetWidth(), shredder1->getToken()->getY()*ofGetHeight()+20));
        verts.push_back(ofPoint(shredder1->getToken()->getX()*ofGetWidth(), shredder1->getToken()->getY()*ofGetHeight()-20));
        verts.push_back(ofPoint(shredder2->getToken()->getX()*ofGetWidth(), shredder2->getToken()->getY()*ofGetHeight()-20));
        verts.push_back(ofPoint(shredder2->getToken()->getX()*ofGetWidth(), shredder2->getToken()->getY()*ofGetHeight()+20));
        ofPolyline p;
        p.addVertexes(verts);
        p.close();
        ofSetColor(255, 0, 0);
        ofFill();
        p.getSmoothed(5,1);
        p.simplify();
        p.draw();
    }
    if(drawAreaForContour.active)
    {
        drawAreaForContour.draw();
        if(hasShredder1)
        {
            ofSetColor(COLORSCHEME_LIGHTGREY);
            ofCircle(shredder1->getToken()->getX()*ofGetWidth(), shredder1->getToken()->getY()*ofGetHeight(),100);
        }
        if(hasShredder2)
        {
            ofSetColor(COLORSCHEME_LIGHTGREY);
            ofCircle(shredder2->getToken()->getX()*ofGetWidth(), shredder2->getToken()->getY()*ofGetHeight(),100);
        }
    }
    ofPopStyle();
    ofPushStyle();
    
    if(onTableCCToken != NULL)
    {
        if(onTableCCToken->getIsOnTable())
        {
            onTableCCToken->draw();
        }
    }
    if(onTokenCCToken != NULL)
    {
        if(onTokenCCToken->getIsOnTable())
        {
            onTokenCCToken->draw();
        }
    }
    if(inAirCCToken != NULL)
    {
        if(inAirCCToken->getIsOnTable())
        {
            inAirCCToken->draw();
        }
    }
    if(saveToken != NULL)
    {
        if(saveToken->getIsOnTable())
        {
            saveToken->draw();
        }
    }
    
    std::vector<Finger *> copyfingers = fingers.getObjects();
    for (std::vector<Finger *>::iterator it = copyfingers.begin() ; it < copyfingers.end(); it++)
    {
        ofPushStyle();
        ofSetColor(0, 0, 0);
        vector<cv::Point> approxCurve = (*it)->limbApproxCountour;
        if( approxCurve.size() >0)
        {
            ofPolyline p2;
            ofPath pa;
            pa.clear();
            for(int temp = 0; temp < approxCurve.size(); temp++)
            {
                p2.addVertex(ofPoint(ofGetWidth() - approxCurve[temp].x/(float)kinect.width*ofGetWidth() , ofGetHeight() - approxCurve[temp].y/(float)kinect.height*ofGetHeight()));
            }
            p2.close();
            p2 = p2.getSmoothed(5,1);
            p2.simplify();
            vector<ofPoint> points = p2.getVertices();
            for(int temp = 0; temp < points.size(); temp++)
            {
                pa.curveTo(points.at(temp));
            }
            pa.close();
            pa.setColor(COLORSCHEME_LIGHTGREY_TRANS);
            pa.translate(ofVec2f(-(p2.getBoundingBox().getCenter().x - (*it)->getLimbCenterX()*ofGetWidth()),-(p2.getBoundingBox().getCenter().y - (*it)->getLimbCenterY()*ofGetHeight())));
            pa.draw();
            vector<ofPolyline> polypa = pa.getOutline();
            ofSetColor(COLORSCHEME_DARKGREY);
            for(int i = 0; i < polypa.size(); i++)
            {
                polypa.at(i).draw();
            }
        }
        ofPopStyle();
        ofPushStyle();
        (*it)->draw();
        ofPopStyle();
    }
    ofPopStyle();
    if(showKinectGUI)
    {
        setGUIKINECT();
        showKinectGUI = false;
    }
    ofPushStyle();
    calibrationMarker->drawBox(COLORSCHEME_LIGHTGREY);
    ofPopStyle();
    ofPushStyle();
    logo.draw(ofGetWidth()-90, 10, 80, 52);
    ofPopStyle();
    ofPushStyle();
    ofSetColor(COLORSCHEME_TEXT_BLACK);
    switch (cameraScene.currentState)
    {
        case ofEasyFingerCam::ROTATING:
        {
            cameraImg.draw(0,0, 40, 40);
            font.drawString(" is rotating...", 40, 30);
            break;
        }
        case ofEasyFingerCam::SCALING:
        {
            cameraImg.draw(0,0,40, 40);
            font.drawString(" is zooming...", 40, 30);
            break;
        }
        case ofEasyFingerCam::PANNING:
        {
            cameraImg.draw(0,0,40, 40);
            font.drawString(" is panning...", 40, 30);
            break;
        }
        case ofEasyFingerCam::TWEENING:
        {
            cameraImg.draw(0,0, 40, 40);
            font.drawString(" is tweening...", 40, 30);
            break;
        }
        case ofEasyFingerCam::STABLE:
        {
            break;
        }
        case ofEasyFingerCam::POINTING:
        {
            cameraImg.draw(0,0, 40, 40);
            font.drawString(" is setting a entry point for shapes...", 40, 30);
            ofVec2f cursorCamera = cameraScene.worldToScreen(shapeEntryPoint, viewport);
            ofSetColor(COLORSCHEME_TEXT_BLACK);
            ofNoFill();
            ofCircle(cursorCamera, ((ofGetElapsedTimeMillis()/20)%50));
            break;
        }
        case ofEasyFingerCam::RESET:
        {
            cameraImg.draw(0,0, 40, 40);
            font.drawString(" is reseting...", 40, 30);
            break;
        }
        default:
            break;
    }
    if(hasOrbiter)
    {
        orbiter->update();
        orbiter->draw();
    }
    ofPopStyle();
    //kinect.drawDepth(0, 0, kinect.width , kinect.height);
}

void testApp::stitchFingers()
{
    std::vector<Finger *> copyfingers = fingers.getObjects();
    for(vector<DetectedFinger *>::iterator it = detectedFingers.begin(); it != detectedFingers.end(); it++)
    {
        cv::Point2i p = (*it)->getExtremity();
        cv::Point p2 = (*it)->getLimb()->getCenterLimb();
        cv::Point2i p3 = (*it)->getFingerBase();
        Finger* closest;
        float distanceclosest = 9999;
        bool found = false;
        std::vector<Finger *>::iterator tempit;
        for (std::vector<Finger *>::iterator it2 = copyfingers.begin() ; it2 < copyfingers.end(); it2++ )
        {
            float distancetemp;
            if( !calibratingFingers)
            {
                ofVec2f calib1 = calibrationFinger->applyCalibration(kinect.width-p.x, kinect.height-p.y);
                if(calib1.x == 0 && calib1.y == 0)
                {
                    break;
                }
                distancetemp = ofVec2f(calib1.x*ofGetWidth(), calib1.y*ofGetHeight()).distance(ofVec2f((*it2)->getX()*ofGetWidth(), (*it2)->getY() *ofGetHeight()));
                
            }
            else
            {
                distancetemp = ofVec2f(p.x,p.y).distance(ofVec2f((1-(*it2)->getX())*kinect.width,  (1-(*it2)->getY()) *kinect.height));
            }
            if(distancetemp < distanceclosest)
            {
                closest = (*it2);
                tempit = it2;
                distanceclosest = distancetemp;
                found = true;
            }
        }
        if(found)
        {
            // cout << "closest " << distanceclosest << endl;
            if(distanceclosest < 250)
            {
                if(! calibratingFingers)
                {
                    ofVec2f calib1 = calibrationFinger->applyCalibration(kinect.width-p.x, kinect.height-p.y);
                    ofVec2f calib2 = calibrationFinger->applyCalibration(kinect.width-p2.x, kinect.height- p2.y);
                    ofVec2f calib3 = calibrationFinger->applyCalibration(kinect.width-p3.x, kinect.height- p3.y);
                    //   cout << "calib1" << calib1.x << endl;
                    //what if if(calib1.x != 0 && calib1.y != 0){ //MISSING
                    closest->checkFinger(calib1.x,calib1.y,  (*it)->getFingerDepthForSurfacePLane(), (*it)->getFingerDepthForObjectPlane() , calib2.x, calib2.y, calib3.x, calib3.y, (*it)->getLimb()->getApproxCountour());
                }
                else
                {
                    ofVec2f calib1 = ofVec2f(p.x/(float)kinect.width, p.y/(float)kinect.height);
                    ofVec2f calib2 = ofVec2f(p2.x/(float)kinect.width, p2.y/(float)kinect.height);
                    ofVec2f calib3 = ofVec2f(p3.x/(float)kinect.width, p3.y/(float)kinect.height);
                    closest->checkFinger(1-calib1.x,1-calib1.y,  (*it)->getFingerDepthForSurfacePLane(), (*it)->getFingerDepthForObjectPlane(), 1-calib2.x, 1-calib2.y,1-calib3.x, 1-calib3.y, (*it)->getLimb()->getApproxCountour());
                }
                closest->updateMoved();
                copyfingers.erase(tempit);
            }
            else
            {
                //drop fingertip
            }
        }
        else
        {
            fingers.lockVector();
            if( !calibratingFingers)
            {
                ofVec2f calib1 = calibrationFinger->applyCalibration(kinect.width-p.x, kinect.height-p.y);
                ofVec2f calib2 = calibrationFinger->applyCalibration(kinect.width-p2.x, kinect.height- p2.y);
                ofVec2f calib3 = calibrationFinger->applyCalibration(kinect.width-p3.x, kinect.height- p3.y);
                if(calib1.x != 0 && calib1.y != 0)
                {
                    fingers.addElement(new Finger( calib1.x, calib1.y, (*it)->getFingerDepthForSurfacePLane(), (*it)->getFingerDepthForObjectPlane() , newFingerIDCounter,  calib2.x, calib2.y, calib3.x, calib3.y, (*it)->getLimb()->getApproxCountour()));
                }
            }
            else
            {
                ofVec2f calib1 = ofVec2f(p.x/(float)kinect.width, p.y/(float)kinect.height);
                ofVec2f calib2 = ofVec2f(p2.x/(float)kinect.width, p2.y/(float)kinect.height);
                ofVec2f calib3 = ofVec2f(p3.x/(float)kinect.width, p3.y/(float)kinect.height);
                fingers.addElement(new Finger( 1-calib1.x, 1-calib1.y,  (*it)->getFingerDepthForSurfacePLane(), (*it)->getFingerDepthForObjectPlane(), newFingerIDCounter,  1-calib2.x, 1-calib2.y,1-calib3.x, 1-calib3.y,  (*it)->getLimb()->getApproxCountour()));
            }
            newFingerIDCounter++;
            fingers.unlockVector();
        }
    }
    for (std::vector<Finger *>::iterator it = copyfingers.begin() ; it < copyfingers.end(); it++)
    {
        //if(!(*it)->fake){//erase this
        if(!(*it)->hasBeenAdded)
        {
            (*it)->markForRemoval = true;
        }
        else
        {
            (*it)->hasBeenRemoved = true;
        }
        //}
    }
}

void testApp::updateLevel(Finger * finger)
{
    if( finger->getZForSurfacePlane() > 260)
    {
        finger->previousTypeLevel = finger->typeLevel;
        finger->typeLevel = Finger::CEILING;
        return;
    }
    //check to see if it hits a container
    vector<ContainerToken*> temp = containers.getObjects();
    bool hitcontainer = false;
    for (std::vector<ContainerToken*>::iterator it2=temp.begin() ; it2 < temp.end() && !hitcontainer; it2++ )
    {
        if((*it2)->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight(),finger->getZForObjectPlane(),false))
        {
            hitcontainer = true;
        };
    }
    if(hitcontainer)
    {
        if( finger->getZForObjectPlane() > 34)
        {
            finger->previousTypeLevel = finger->typeLevel;
            finger->typeLevel = Finger::ABOVEOBJECT;
            return;
        }
        else
        {
            finger->previousTypeLevel = finger->typeLevel;
            finger->typeLevel = Finger::OBJECT;
            return;
        }
    }
    if(hasShredder1)
    {
        if(shredder1->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
        {
            if( finger->getZForObjectPlane() > 34)
            {
                finger->previousTypeLevel = finger->typeLevel;
                finger->typeLevel = Finger::ABOVEOBJECT;
                return;
            }
            else
            {
                finger->previousTypeLevel = finger->typeLevel;
                finger->typeLevel = Finger::OBJECT;
                return;
            }
        }
        else
        {
            //carry on
        }
    }
    if(hasShredder2)
    {
        if(shredder2->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
        {
            if( finger->getZForObjectPlane() > 34)
            {
                finger->previousTypeLevel = finger->typeLevel;
                finger->typeLevel = Finger::ABOVEOBJECT;
                return;
            }
            else
            {
                finger->previousTypeLevel = finger->typeLevel;
                finger->typeLevel = Finger::OBJECT;
                return;
            }
        }
        else
        {
            //carry on
        }
    }
    if(hasOrbiter)
    {
        if(orbiter->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
        {
            if(finger->getFingerBaseX()*ofGetWidth() !=0 &&
               finger->getFingerBaseY()*ofGetHeight()!=0)
            {
                orbiter->store(ofVec2f(
                                       finger->getFingerBaseX()*ofGetWidth(),
                                       finger->getFingerBaseY()*ofGetHeight()),
                               ofVec2f(finger->getX()*ofGetWidth(),
                                       finger->getY()*ofGetHeight())
                               );
                orbiter->action();
                if( finger->getZForObjectPlane() > 34)
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::ABOVEOBJECT;
                    return;
                }
                else
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::OBJECT;
                    return;
                }
            }
            else if(finger->getLimbCenterX()*ofGetWidth() !=0 &&
                    finger->getLimbCenterY()*ofGetHeight()!=0)
            {
                orbiter->store(ofVec2f(
                                       finger->getLimbCenterX()*ofGetWidth(),
                                       finger->getLimbCenterY()*ofGetHeight()),
                               ofVec2f(finger->getX()*ofGetWidth(),
                                       finger->getY()*ofGetHeight())
                               );
                orbiter->action();
                if( finger->getZForObjectPlane() > 34)
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::ABOVEOBJECT;
                    return;
                }
                else
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::OBJECT;
                    return;
                }
            }
        }
        else
        {
            //carry on
        }
    }
    if(onTokenCCToken != NULL)
    {
        if(onTokenCCToken->getIsOnTable())
        {
            if(onTokenCCToken->insideToken(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
            {
                if( finger->getZForObjectPlane() > 34)
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::ABOVEOBJECT;
                    return;
                }
                else
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::OBJECT;
                    return;
                }
            }
        }
    }
    if(saveToken != NULL)
    {
        if(saveToken->getIsOnTable())
        {
            if(saveToken->insideToken(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
            {
                if( finger->getZForObjectPlane() > 34)
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::ABOVEOBJECT;
                    return;
                }
                else
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::OBJECT;
                    return;
                }
            }
        }
    }
    if(inAirCCToken != NULL)
    {
        if(inAirCCToken->getIsOnTable())
        {
            if(inAirCCToken->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
            {
                if( finger->getZForObjectPlane() > 34)
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::ABOVEOBJECT;
                    return;
                }
                else
                {
                    finger->previousTypeLevel = finger->typeLevel;
                    finger->typeLevel = Finger::OBJECT;
                    return;
                }
            }
        }
    }
    if(finger->getZForSurfacePlane() > 32)
    {
        finger->previousTypeLevel = finger->typeLevel;
        finger->typeLevel = Finger::ABOVESURFACE;
        return;
    }
    else
    {
        finger->previousTypeLevel = finger->typeLevel;
        finger->typeLevel = Finger::SURFACE;
        return;
    }
}

void testApp::onSurface(Finger * finger)
{
    bool hitcircle = false;
    if(onTableCCToken != NULL)
    {
        if(onTableCCToken->getIsOnTable())
        {
            std::pair<bool, OnScreenOption *> temppair = onTableCCToken->checkHit(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight());
            if(temppair.first)
            {
                hitcircle = true;
                finger->typeFinger = Finger::ONTABLEOPTION;
                if(temppair.second != NULL)
                {
                    KinectOption *d1 = dynamic_cast<KinectOption*>(temppair.second);
                    if((bool)d1)
                    {
                        unsetGUI();
                        showKinectGUI = true;
                        kinectScanning = true;
                        kinectScan->changeColor(new ofColor(255,255,255));
                    }
                    else
                    {
                        ContourOption *d2 = dynamic_cast<ContourOption*>(temppair.second);
                        if((bool)d2)
                        {
                            drawAreaForContour.active = true;
                            drawAreaForContour.width = ofGetWidth();
                            drawAreaForContour.height = ofGetHeight();
                        }
                        else
                        {
                            BackOption *d3 = dynamic_cast<BackOption*>(temppair.second);
                            if((bool)d3)
                            {
                                if(kinectScanning)
                                {
                                    unsetGUI();
                                    kinectScanning = false;
                                }
                                else if(drawAreaForContour.active)
                                {
                                    drawAreaForContour.active = false;
                                    drawAreaForContour.clearDrawArea();
                                }
                            }
                            else
                            {
                                SaveKinectOption *d4 = dynamic_cast<SaveKinectOption*>(temppair.second);
                                if((bool)d4)
                                {
                                    unsetGUI();
                                    Basic3DObjectFromCopy * temp = new Basic3DObjectFromCopy(shapeEntryPoint, kinectScan->mesh) ;
                                    ofColor * tempColor = new ofColor(0,0,0);
                                    tempColor->setHsb(ofRandom(0,255), ofRandom(180,255), ofRandom(180,255));
                                    temp->changeColor(tempColor);
                                    shapes.lockVector();
                                    shapes.addElement(temp);
                                    shapes.unlockVector();
                                    unsetGUI();
                                    kinectScanning = false;
                                    showKinectGUI = false;
                                }
                                else
                                {
                                    SaveContourOption *d5 = dynamic_cast<SaveContourOption*>(temppair.second);
                                    if((bool)d5)
                                    {
                                        //drawAreaForContour.active = false;
                                        drawAreaForContour.save();
                                        ///MAKE SAVE FUNCTION
                                    }
                                    else
                                    {
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if(!hitcircle)
    {
        bool hitmenu = false;
        if(onTokenCCToken != NULL)
        {
            if(onTokenCCToken->getIsOnTable())
            {
                std::pair<bool, OnScreenOption *> temppair = onTokenCCToken->checkHit(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight());
                if(temppair.first)
                {
                    hitmenu = true;
                    finger->typeFinger = Finger::ONTABLEOPTION;
                    if(temppair.second != NULL)
                    {
                        SaveKinectOption *d1 = dynamic_cast<SaveKinectOption*>(temppair.second);
                        if((bool)d1)
                        {
                            unsetGUI();
                            Basic3DObjectFromCopy * temp = new Basic3DObjectFromCopy(shapeEntryPoint, kinectScan->mesh) ;
                            ofColor * tempColor = new ofColor(0,0,0);
                            tempColor->setHsb(ofRandom(0,255), ofRandom(180,255), ofRandom(180,255));
                            temp->changeColor(tempColor);
                            shapes.lockVector();
                            shapes.addElement(temp);
                            shapes.unlockVector();
                            unsetGUI();
                            kinectScanning = false;
                            showKinectGUI = false;
                        }
                        else
                        {
                            SaveContourOption *d2 = dynamic_cast<SaveContourOption*>(temppair.second);
                            if((bool)d2)
                            {
                                //drawAreaForContour.active = false;
                                drawAreaForContour.save();
                                ///MAKE SAVE FUNCTION
                            }
                            else
                            {
                                BackOption *d3 = dynamic_cast<BackOption*>(temppair.second);
                                if((bool)d3)
                                {
                                    if(kinectScanning)
                                    {
                                        unsetGUI();
                                        kinectScanning = false;
                                    }
                                    else if(drawAreaForContour.active)
                                    {
                                        drawAreaForContour.active = false;
                                        drawAreaForContour.clearDrawArea();
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if(!hitmenu)
        {
            if(drawAreaForContour.active)
            {
                bool inMenuCircle = false;
                if(onTableCCToken != NULL)
                {
                    if(onTableCCToken->getIsOnTable())
                    {
                        if(onTableCCToken->insideProtectedOptions(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                        {
                            inMenuCircle = true;
                        }
                    }
                }
                if(onTokenCCToken != NULL)
                {
                    if(onTokenCCToken->getIsOnTable())
                    {
                        if(onTokenCCToken->insideProtectedOptions(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                        {
                            inMenuCircle = true;
                        }
                    }
                }
                if(inAirCCToken != NULL)
                {
                    if(inAirCCToken->getIsOnTable())
                    {
                        inMenuCircle = false;
                    }
                }
                if(!inMenuCircle)
                {
                    drawAreaForContour.addVert(finger);
                    finger->typeFinger = Finger::CONTOUR;
                }
            }
            else
            {
                // vector<ContainerToken*> temp2 = containers.getObjects();
                
                vector<ContainerToken*> temp3 = containers.getObjects();
                bool hitrightcontainer = false;
                for (std::vector<ContainerToken*>::iterator it3=temp3.begin() ; it3 < temp3.end() && !hitrightcontainer; it3++ )
                {
                    if((*it3)->checkSouthScale(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                    {
                        hitrightcontainer = true;
                        (*it3)->setHasSouthScale(true);
                        (*it3)->fingerOnSouthScale = finger;
                    };
                }
                if(hitrightcontainer)
                {
                    finger->typeFinger = Finger::SCALECONTAINERSOUTH;
                }
                else
                {
                    vector<ContainerToken*> temp4 = containers.getObjects();
                    bool hitleftcontainer = false;
                    for (std::vector<ContainerToken*>::iterator it4=temp4.begin() ; it4 < temp4.end() && !hitleftcontainer; it4++ )
                    {
                        if((*it4)->checkEastScale(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                        {
                            hitleftcontainer = true;
                            (*it4)->setHasEastScale(true);
                            (*it4)->fingerOnEastScale = finger;
                        };
                    }
                    if(hitleftcontainer)
                    {
                        finger->typeFinger = Finger::SCALECONTAINERNORTH;
                    }
                    else
                    {
                        finger->typeFinger = Finger::PICKER;
                        cameraScene.fingerPickerIn(finger);
                    }
                    
                }
            }
        }
    }
}

void testApp::changeLevel(Finger * finger)
{
    switch (finger->previousTypeLevel)
    {
        case Finger::SURFACE:
        {
            switch (finger->typeLevel)
            {
                case Finger::SURFACE:
                {
                    //NO CHANGE
                    break;
                }
                case Finger::ABOVESURFACE:
                {
                    if(!(finger->typeFinger == Finger::SHAPESELECTION || finger->typeFinger  == Finger::SCALECONTAINERNORTH || finger->typeFinger  == Finger::SCALECONTAINERSOUTH))
                    {
                        changeFingerType(finger);
                        finger->typeFinger = Finger::NOTYPE;
                    }
                    break;
                }
                case Finger::CEILING:
                {
                    changeFingerType(finger);
                    finger->typeFinger = Finger::CAMERA;
                    cameraScene.fingerIn(finger);
                    break;
                }
                case Finger::OBJECT:
                {
                    if(finger->typeFinger == Finger::SCALECONTAINERNORTH || finger->typeFinger  == Finger::SCALECONTAINERSOUTH)
                    {
                        changeFingerType(finger);
                        finger->typeFinger = Finger::NOTYPE;
                    }
                    
                    break;
                }
                case Finger::ABOVEOBJECT:
                {
                    if(finger->typeFinger == Finger::SCALECONTAINERNORTH || finger->typeFinger  == Finger::SCALECONTAINERSOUTH)
                    {
                        changeFingerType(finger);
                        finger->typeFinger = Finger::NOTYPE;
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case Finger::ABOVESURFACE:
        {
            switch (finger->typeLevel)
            {
                case Finger::SURFACE:
                {
                    //CHECK FOR PROBLEMS
                    changeFingerType(finger);
                    finger->typeFinger == Finger::NOTYPE;
                    onSurface(finger);
                    break;
                }
                case Finger::ABOVESURFACE:
                {
                    break;
                }
                case Finger::CEILING:
                {
                    changeFingerType(finger);
                    finger->typeFinger = Finger::CAMERA;
                    cameraScene.fingerIn(finger);
                    break;
                }
                case Finger::OBJECT:
                {
                    vector<ContainerToken*> temp = containers.getObjects();
                    bool hitcontainer = false;
                    for (std::vector<ContainerToken*>::iterator it2=temp.begin() ; it2 < temp.end() && !hitcontainer; it2++ )
                    {
                        if((*it2)->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight(),finger->getZForObjectPlane(), false))
                        {
                            hitcontainer = true;
                            finger->typeFinger = Finger::NOTYPE;
                            (*it2)->click(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight(),finger->getZForObjectPlane() );
                        };
                    }
                    if(!hitcontainer)
                    {
                        bool hitonTokenCCToken = false;
                        if(onTokenCCToken != NULL)
                        {
                            if(onTokenCCToken->getIsOnTable())
                            {
                                if(onTokenCCToken->insideToken(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                                {
                                    finger->typeFinger = Finger::NOTYPE;
                                    hitonTokenCCToken = true;
                                    OnTokenContentCreationUI::RETURNTYPE rt = onTokenCCToken->action(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight());
                                    switch(rt)
                                    {
                                        case OnTokenContentCreationUI::RETURNCONTOUR:
                                        {
                                            drawAreaForContour.active = true;
                                            drawAreaForContour.width = ofGetWidth();
                                            drawAreaForContour.height = ofGetHeight();
                                            break;
                                        }
                                        case OnTokenContentCreationUI::RETURNCOPY:
                                        {
                                            break;
                                        }
                                        case OnTokenContentCreationUI::RETURNKINECT:
                                        {
                                            unsetGUI();
                                            showKinectGUI = true;
                                            kinectScanning = true;
                                            kinectScan->changeColor(new ofColor(255,255,255));
                                            break;
                                        }
                                        case OnTokenContentCreationUI::RETURNSTL:
                                        {
                                            break;
                                        }
                                        case OnTokenContentCreationUI::NORETURN:
                                        {
                                            break;
                                        }
                                        default:
                                            break;
                                    }
                                }
                            }
                        }
                        if(!hitonTokenCCToken)
                        {
                            bool hitInAirCCToken = false;
                            if(inAirCCToken != NULL)
                            {
                                if(inAirCCToken->getIsOnTable())
                                {
                                    if(inAirCCToken->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                                    {
                                        hitonTokenCCToken = true;
                                        finger->typeFinger = Finger::NOTYPE;
                                        InAirContentCreationUI::RETURNTYPE rt = inAirCCToken->action(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight());
                                        switch(rt)
                                        {
                                            case InAirContentCreationUI::RETURNCONTOUR:
                                            {
                                                drawAreaForContour.active = true;
                                                drawAreaForContour.width = ofGetWidth();
                                                drawAreaForContour.height = ofGetHeight();
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNCOPY:
                                            {
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNKINECT:
                                            {
                                                unsetGUI();
                                                showKinectGUI = true;
                                                kinectScan->changeColor(new ofColor(255,255,255));
                                                kinectScanning = true;
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNSTL:
                                            {
                                                break;
                                            }
                                            case InAirContentCreationUI::NORETURN:
                                            {
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNDISCARDCONTOUR:
                                            {
                                                drawAreaForContour.active = false;
                                                drawAreaForContour.clearDrawArea();
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNDISCARDKINECT:
                                            {
                                                unsetGUI();
                                                showKinectGUI = false;
                                                kinectScanning = false;
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNSAVECONTOUR:
                                            {
                                                drawAreaForContour.save();
                                                //MISSING FUNCTION
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNSAVEKINECT:
                                            {
                                                unsetGUI();
                                                Basic3DObjectFromCopy * temp = new Basic3DObjectFromCopy(shapeEntryPoint, kinectScan->mesh) ;
                                                ofColor * tempColor = new ofColor(0,0,0);
                                                tempColor->setHsb(ofRandom(0,255), ofRandom(180,255), ofRandom(180,255));
                                                temp->changeColor(tempColor);
                                                shapes.lockVector();
                                                shapes.addElement(temp);
                                                shapes.unlockVector();
                                                unsetGUI();
                                                kinectScanning = false;
                                                showKinectGUI = false;
                                                break;
                                            }
                                            default:
                                                break;
                                        }
                                    }
                                }
                            }
                            //LAST ADD
                            if(!hitInAirCCToken)
                            {
                                if(saveToken != NULL)
                                {
                                    if(saveToken->getIsOnTable())
                                    {
                                        if(saveToken->insideToken(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                                        {
                                            finger->typeFinger = Finger::NOTYPE;
                                            saveToken->action();
                                        }
                                    }
                                }
                            }
                        }
                    }
                    break;
                }
                case Finger::ABOVEOBJECT:
                {
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case Finger::CEILING:
        {
            switch (finger->typeLevel)
            {
                case Finger::SURFACE:
                {
                    changeFingerType(finger);
                    finger->typeFinger == Finger::NOTYPE;
                    onSurface(finger);
                    break;
                }
                case Finger::ABOVESURFACE:
                {
                    changeFingerType(finger);
                    finger->typeFinger == Finger::NOTYPE;
                    break;
                }
                case Finger::CEILING:
                {
                    break;
                }
                case Finger::OBJECT:
                {
                    changeFingerType(finger);
                    finger->typeFinger == Finger::NOTYPE;
                    break;
                }
                case Finger::ABOVEOBJECT:
                {
                    changeFingerType(finger);
                    finger->typeFinger == Finger::NOTYPE;
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case Finger::OBJECT:
        {
            switch (finger->typeLevel)
            {
                case Finger::SURFACE:
                {
                    changeFingerType(finger);
                    finger->typeFinger == Finger::NOTYPE;
                    onSurface(finger);
                    break;
                }
                case Finger::ABOVESURFACE:
                {
                    break;
                }
                case Finger::CEILING:
                {
                    changeFingerType(finger);
                    finger->typeFinger = Finger::CAMERA;
                    cameraScene.fingerIn(finger);
                    break;
                }
                case Finger::OBJECT:
                {
                    if(inAirCCToken != NULL)
                    {
                        if(inAirCCToken->getIsOnTable())
                        {
                            if(inAirCCToken->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                            {
                                finger->typeFinger = Finger::NOTYPE;
                                InAirContentCreationUI::RETURNTYPE rt = inAirCCToken->action(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight());
                                switch(rt)
                                {
                                    case InAirContentCreationUI::RETURNCONTOUR:
                                    {
                                        drawAreaForContour.active = true;
                                        drawAreaForContour.width = ofGetWidth();
                                        drawAreaForContour.height = ofGetHeight();
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNCOPY:
                                    {
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNKINECT:
                                    {
                                        unsetGUI();
                                        showKinectGUI = true;
                                        kinectScan->changeColor(new ofColor(255,255,255));
                                        kinectScanning = true;
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNSTL:
                                    {
                                        break;
                                    }
                                    case InAirContentCreationUI::NORETURN:
                                    {
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNDISCARDCONTOUR:
                                    {
                                        drawAreaForContour.active = false;
                                        drawAreaForContour.clearDrawArea();
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNDISCARDKINECT:
                                    {
                                        unsetGUI();
                                        showKinectGUI = false;
                                        kinectScanning = false;
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNSAVECONTOUR:
                                    {
                                        drawAreaForContour.save();
                                        //MISSING FUNCTION
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNSAVEKINECT:
                                    {
                                        unsetGUI();
                                        Basic3DObjectFromCopy * temp = new Basic3DObjectFromCopy(shapeEntryPoint, kinectScan->mesh) ;
                                        ofColor * tempColor = new ofColor(0,0,0);
                                        tempColor->setHsb(ofRandom(0,255), ofRandom(180,255), ofRandom(180,255));
                                        temp->changeColor(tempColor);
                                        shapes.lockVector();
                                        shapes.addElement(temp);
                                        shapes.unlockVector();
                                        unsetGUI();
                                        kinectScanning = false;
                                        showKinectGUI = false;
                                        break;
                                    }
                                    default:
                                        break;
                                }
                            }
                        }
                    }
                    break;
                }
                case Finger::ABOVEOBJECT:
                {
                    if(inAirCCToken != NULL)
                    {
                        if(inAirCCToken->getIsOnTable())
                        {
                            if(inAirCCToken->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                            {
                                finger->typeFinger = Finger::NOTYPE;
                                InAirContentCreationUI::RETURNTYPE rt = inAirCCToken->action(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight());
                                switch(rt)
                                {
                                    case InAirContentCreationUI::RETURNCONTOUR:
                                    {
                                        drawAreaForContour.active = true;
                                        drawAreaForContour.width = ofGetWidth();
                                        drawAreaForContour.height = ofGetHeight();
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNCOPY:
                                    {
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNKINECT:
                                    {
                                        unsetGUI();
                                        showKinectGUI = true;
                                        kinectScan->changeColor(new ofColor(255,255,255));
                                        kinectScanning = true;
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNSTL:
                                    {
                                        break;
                                    }
                                    case InAirContentCreationUI::NORETURN:
                                    {
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNDISCARDCONTOUR:
                                    {
                                        drawAreaForContour.active = false;
                                        drawAreaForContour.clearDrawArea();
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNDISCARDKINECT:
                                    {
                                        unsetGUI();
                                        showKinectGUI = false;
                                        kinectScanning = false;
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNSAVECONTOUR:
                                    {
                                        //MISSING FUNCTION
                                        drawAreaForContour.save();
                                        break;
                                    }
                                    case InAirContentCreationUI::RETURNSAVEKINECT:
                                    {
                                        unsetGUI();
                                        Basic3DObjectFromCopy * temp = new Basic3DObjectFromCopy(shapeEntryPoint, kinectScan->mesh) ;
                                        ofColor * tempColor = new ofColor(0,0,0);
                                        tempColor->setHsb(ofRandom(0,255), ofRandom(180,255), ofRandom(180,255));
                                        temp->changeColor(tempColor);
                                        shapes.lockVector();
                                        shapes.addElement(temp);
                                        shapes.unlockVector();
                                        unsetGUI();
                                        kinectScanning = false;
                                        showKinectGUI = false;
                                        break;
                                    }
                                    default:
                                        break;
                                }
                            }
                        }
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case Finger::ABOVEOBJECT:
        {
            switch (finger->typeLevel)
            {
                case Finger::SURFACE:
                {
                    changeFingerType(finger);
                    finger->typeFinger == Finger::NOTYPE;
                    onSurface(finger);
                    break;
                }
                case Finger::ABOVESURFACE:
                {
                    break;
                }
                case Finger::CEILING:
                {
                    changeFingerType(finger);
                    finger->typeFinger = Finger::CAMERA;
                    cameraScene.fingerIn(finger);
                    break;
                }
                case Finger::OBJECT:
                {
                    vector<ContainerToken*> temp = containers.getObjects();
                    bool hitcontainer = false;
                    for (std::vector<ContainerToken*>::iterator it2=temp.begin() ; it2 < temp.end() && !hitcontainer; it2++ )
                    {
                        if((*it2)->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight(),finger->getZForObjectPlane(),false ))
                        {
                            hitcontainer = true;
                            finger->typeFinger = Finger::NOTYPE;
                            (*it2)->click(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight(),finger->getZForObjectPlane() );
                        };
                    }
                    if(!hitcontainer)
                    {
                        bool hitonTokenCCToken = false;
                        if(onTokenCCToken != NULL)
                        {
                            if(onTokenCCToken->getIsOnTable())
                            {
                                if(onTokenCCToken->insideToken(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                                {
                                    finger->typeFinger = Finger::NOTYPE;
                                    OnTokenContentCreationUI::RETURNTYPE rt = onTokenCCToken->action(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight());
                                    hitonTokenCCToken = true;
                                    switch(rt)
                                    {
                                        case OnTokenContentCreationUI::RETURNCONTOUR:
                                        {
                                            drawAreaForContour.active = true;
                                            drawAreaForContour.width = ofGetWidth();
                                            drawAreaForContour.height = ofGetHeight();
                                            break;
                                        }
                                        case OnTokenContentCreationUI::RETURNCOPY:
                                        {
                                            break;
                                        }
                                        case OnTokenContentCreationUI::RETURNKINECT:
                                        {
                                            unsetGUI();
                                            showKinectGUI = true;
                                            kinectScan->changeColor(new ofColor(255,255,255));
                                            kinectScanning = true;
                                            break;
                                        }
                                        case OnTokenContentCreationUI::RETURNSTL:
                                        {
                                            break;
                                        }
                                        case OnTokenContentCreationUI::NORETURN:
                                        {
                                            break;
                                        }
                                        default:
                                            break;
                                    }
                                }
                            }
                        }
                        if(!hitonTokenCCToken)
                        {
                            bool hitInAirCCToken = false;
                            if(inAirCCToken != NULL)
                            {
                                if(inAirCCToken->getIsOnTable())
                                {
                                    if(inAirCCToken->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                                    {
                                        finger->typeFinger = Finger::NOTYPE;
                                        hitInAirCCToken = true;
                                        InAirContentCreationUI::RETURNTYPE rt = inAirCCToken->action(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight());
                                        switch(rt)
                                        {
                                            case InAirContentCreationUI::RETURNCONTOUR:
                                            {
                                                drawAreaForContour.active = true;
                                                drawAreaForContour.width = ofGetWidth();
                                                drawAreaForContour.height = ofGetHeight();
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNCOPY:
                                            {
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNKINECT:
                                            {
                                                unsetGUI();
                                                showKinectGUI = true;
                                                kinectScanning = true;
                                                kinectScan->changeColor(new ofColor(255,255,255));
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNSTL:
                                            {
                                                break;
                                            }
                                            case InAirContentCreationUI::NORETURN:
                                            {
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNDISCARDCONTOUR:
                                            {
                                                drawAreaForContour.active = false;
                                                drawAreaForContour.clearDrawArea();
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNDISCARDKINECT:
                                            {
                                                unsetGUI();
                                                showKinectGUI = false;
                                                kinectScanning = false;
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNSAVECONTOUR:
                                            {
                                                //MISSING FUNCTION
                                                drawAreaForContour.save();
                                                break;
                                            }
                                            case InAirContentCreationUI::RETURNSAVEKINECT:
                                            {
                                                unsetGUI();
                                                Basic3DObjectFromCopy * temp = new Basic3DObjectFromCopy(shapeEntryPoint, kinectScan->mesh);
                                                ofColor * tempColor = new ofColor(0,0,0);
                                                tempColor->setHsb(ofRandom(0,255), ofRandom(180,255), ofRandom(180,255));
                                                temp->changeColor(tempColor);
                                                shapes.lockVector();
                                                shapes.addElement(temp);
                                                shapes.unlockVector();
                                                unsetGUI();
                                                kinectScanning = false;
                                                showKinectGUI = false;
                                                break;
                                            }
                                            default:
                                                break;
                                        }
                                    }
                                }
                            }
                            //LAST ADD
                            if(!hitInAirCCToken)
                            {
                                if(saveToken != NULL)
                                {
                                    if(saveToken->getIsOnTable())
                                    {
                                        if(saveToken->insideToken(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                                        {
                                            finger->typeFinger = Finger::NOTYPE;
                                            saveToken->action();
                                        }
                                    }
                                }
                            }
                        }
                    }
                    break;
                }
                case Finger::ABOVEOBJECT:
                {
                    vector<ContainerToken*> temp = containers.getObjects();
                    bool hitcontainer = false;
                    for (std::vector<ContainerToken*>::iterator it2=temp.begin() ; it2 < temp.end() && !hitcontainer; it2++ )
                    {
                        if((*it2)->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight(),finger->getZForObjectPlane(),true))
                        {
                            hitcontainer = true;
                            finger->typeFinger = Finger::NOTYPE;
                            (*it2)->click(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight(),finger->getZForObjectPlane() );
                        };
                    }
                    if(!hitcontainer)
                    {
                        if(inAirCCToken != NULL)
                        {
                            if(inAirCCToken->getIsOnTable())
                            {
                                if(inAirCCToken->inside(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                                {
                                    finger->typeFinger = Finger::NOTYPE;
                                    InAirContentCreationUI::RETURNTYPE rt = inAirCCToken->action(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight());
                                    switch(rt)
                                    {
                                        case InAirContentCreationUI::RETURNCONTOUR:
                                        {
                                            drawAreaForContour.active = true;
                                            drawAreaForContour.width = ofGetWidth();
                                            drawAreaForContour.height = ofGetHeight();
                                            break;
                                        }
                                        case InAirContentCreationUI::RETURNCOPY:
                                        {
                                            break;
                                        }
                                        case InAirContentCreationUI::RETURNKINECT:
                                        {
                                            unsetGUI();
                                            showKinectGUI = true;
                                            kinectScan->changeColor(new ofColor(255,255,255));
                                            kinectScanning = true;
                                            break;
                                        }
                                        case InAirContentCreationUI::RETURNSTL:
                                        {
                                            break;
                                        }
                                        case InAirContentCreationUI::NORETURN:
                                        {
                                            break;
                                        }
                                        case InAirContentCreationUI::RETURNDISCARDCONTOUR:
                                        {
                                            drawAreaForContour.active = false;
                                            drawAreaForContour.clearDrawArea();
                                            break;
                                        }
                                        case InAirContentCreationUI::RETURNDISCARDKINECT:
                                        {
                                            unsetGUI();
                                            showKinectGUI = false;
                                            kinectScanning = false;
                                            break;
                                        }
                                        case InAirContentCreationUI::RETURNSAVECONTOUR:
                                        {
                                            drawAreaForContour.save();
                                            //MISSING FUNCTION
                                            break;
                                        }
                                        case InAirContentCreationUI::RETURNSAVEKINECT:
                                        {
                                            unsetGUI();
                                            Basic3DObjectFromCopy * temp = new Basic3DObjectFromCopy(shapeEntryPoint, kinectScan->mesh) ;
                                            ofColor * tempColor = new ofColor(0,0,0);
                                            tempColor->setHsb(ofRandom(0,255), ofRandom(180,255), ofRandom(180,255));
                                            temp->changeColor(tempColor);
                                            shapes.lockVector();
                                            shapes.addElement(temp);
                                            shapes.unlockVector();
                                            unsetGUI();
                                            kinectScanning = false;
                                            showKinectGUI = false;
                                            break;
                                        }
                                        default:
                                            break;
                                    }
                                }
                            }
                        }
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }
        default:
            break;
    }
}

void testApp::updatetCAD()
{
    stitchFingers();
    vector<Finger *> tempfingers = fingers.getObjects();
    if(tempfingers.size() == 0)
    {
        cameraScene.clearFingersPickers();
        cameraScene.clearFingers();
    }
    for (std::vector<Finger*>::iterator it=tempfingers.begin() ; it < tempfingers.end(); it++ )
    {
        (*it)->update();
        updateLevel((*it));
        //updateLevel
        if((*it)->markForAdd)
        {
            if((*it)->typeLevel == Finger::ABOVESURFACE)
            {
                //NOACTION
            }
            else if((*it)->typeLevel == Finger::CEILING)
            {
                (*it)->typeFinger = Finger::CAMERA;
                cameraScene.fingerIn((*it));
            }
            else if((*it)->typeLevel == Finger::OBJECT)
            {
                //MISSING
                if(inAirCCToken != NULL)
                {
                    if(inAirCCToken->getIsOnTable())
                    {
                        if(inAirCCToken->inside((*it)->getX()*ofGetWidth(), (*it)->getY()*ofGetHeight()))
                        {
                            (*it)->typeFinger = Finger::NOTYPE;
                            InAirContentCreationUI::RETURNTYPE rt = inAirCCToken->action((*it)->getX()*ofGetWidth(), (*it)->getY()*ofGetHeight());
                            switch(rt)
                            {
                                case InAirContentCreationUI::RETURNCONTOUR:
                                {
                                    drawAreaForContour.active = true;
                                    drawAreaForContour.width = ofGetWidth();
                                    drawAreaForContour.height = ofGetHeight();
                                    break;
                                }
                                case InAirContentCreationUI::RETURNCOPY:
                                {
                                    break;
                                }
                                case InAirContentCreationUI::RETURNKINECT:
                                {
                                    unsetGUI();
                                    showKinectGUI = true;
                                    kinectScanning = true;
                                    kinectScan->changeColor(new ofColor(255,255,255));
                                    break;
                                }
                                case InAirContentCreationUI::RETURNSTL:
                                {
                                    break;
                                }
                                case InAirContentCreationUI::NORETURN:
                                {
                                    break;
                                }
                                default:
                                    break;
                            }
                        }
                    }
                }
            }
            else if((*it)->typeLevel == Finger::ABOVEOBJECT)
            {
                //MISSING
                if(inAirCCToken != NULL)
                {
                    if(inAirCCToken->getIsOnTable())
                    {
                        if(inAirCCToken->inside((*it)->getX()*ofGetWidth(), (*it)->getY()*ofGetHeight()))
                        {
                            (*it)->typeFinger = Finger::NOTYPE;
                            InAirContentCreationUI::RETURNTYPE rt = inAirCCToken->action((*it)->getX()*ofGetWidth(), (*it)->getY()*ofGetHeight());
                            switch(rt)
                            {
                                case InAirContentCreationUI::RETURNCONTOUR:
                                {
                                    drawAreaForContour.active = true;
                                    drawAreaForContour.width = ofGetWidth();
                                    drawAreaForContour.height = ofGetHeight();
                                    break;
                                }
                                case InAirContentCreationUI::RETURNCOPY:
                                {
                                    break;
                                }
                                case InAirContentCreationUI::RETURNKINECT:
                                {
                                    unsetGUI();
                                    showKinectGUI = true;
                                    kinectScanning = true;
                                    kinectScan->changeColor(new ofColor(255,255,255));
                                    break;
                                }
                                case InAirContentCreationUI::RETURNSTL:
                                {
                                    break;
                                }
                                case InAirContentCreationUI::NORETURN:
                                {
                                    break;
                                }
                                default:
                                    break;
                            }
                        }
                    }
                }
            }
            if((*it)->typeLevel == Finger::SURFACE)
            {
                onSurface((*it));
            }
            (*it)->markForAdd = false;
        }
        else if((*it)->markForMoved)
        {
            if((*it)->typeFinger == Finger::PICKER)
            {
                if((*it)->pickedShapes3D.size() > 0)
                {
                    selectionMovements.lockVector();
                    selectionMovements.addElement(new SelectionMovement((*it)->pickedShapes3D , (*it), &cameraScene, viewport ));
                    selectionMovements.unlockVector();
                    changeFingerType((*it));
                    (*it)->typeFinger = Finger::SHAPESELECTION;
                }
            }
            changeLevel((*it));
            switch ((*it)->typeFinger)
            {
                case Finger::PICKER:
                {
                    vector<ContainerToken*> temp2 = containers.getObjects();
                    for (std::vector<ContainerToken*>::iterator it2=temp2.begin() ; it2 < temp2.end(); it2++ )
                    {
                        (*it2)->checkLinkCut((*it)->getX()*ofGetWidth(), (*it)->getY()*ofGetHeight(), (*it)->getStartX()*ofGetWidth(), (*it)->getStartY()*ofGetHeight());
                    };
                    break;
                }
                case Finger::CAMERA:
                    break;
                case Finger::ONTABLEOPTION:
                    break;
                case Finger::CONTOUR:
                {
                    bool inMenuCircle = false;
                    if(onTableCCToken != NULL)
                    {
                        if(onTableCCToken->getIsOnTable())
                        {
                            if(onTableCCToken->insideProtectedOptions((*it)->getX()*ofGetWidth(), (*it)->getY()*ofGetHeight()))
                            {
                                inMenuCircle = true;
                            }
                        }
                    }
                    if(onTokenCCToken != NULL)
                    {
                        if(onTokenCCToken->getIsOnTable())
                        {
                            if(onTokenCCToken->insideProtectedOptions((*it)->getX()*ofGetWidth(), (*it)->getY()*ofGetHeight()))
                            {
                                inMenuCircle = true;
                            }
                        }
                    }
                    if(inAirCCToken != NULL)
                    {
                        if(inAirCCToken->getIsOnTable())
                        {
                            inMenuCircle = false;
                        }
                    }
                    if(!inMenuCircle)
                    {
                        drawAreaForContour.addVert((*it));
                    }
                }
                    break;
                    //                case Finger::BOOLEANCONTAINER:
                    //
                    //                    break;
                case Finger::SCALECONTAINERNORTH:
                    break;
                case Finger::SCALECONTAINERSOUTH:
                    break;
                case Finger::SHAPESELECTION:
                {
                    vector<SelectionMovement*> temp = selectionMovements.getObjects();
                    for (std::vector<SelectionMovement*>::iterator it2=temp.begin() ; it2 < temp.end(); it2++ )
                    {
                        if((*it2)->finger->getFingerID() == (*it)->getFingerID())
                        {
                            //(*it2)->vert = ofVec2f((*it)->getX()*ofGetWidth(),(*it)->getY()*ofGetHeight());
                            vector<ContainerToken*> temp2 = containers.getObjects();
                            containers.lockVector();
                            bool found = false;
                            for (std::vector<ContainerToken*>::iterator it3=temp2.begin() ; it3 < temp2.end(); it3++ )
                            {
                                ofVec2f fingerCenter = ofVec2f(((*it)->getX())*ofGetWidth(), (*it)->getY()*ofGetHeight());
                                ofVec2f containerCenter = ofVec2f((*it3)->getToken()->getX() *ofGetWidth(), ((*it3)->getToken()->getY()) * ofGetHeight());
                                if(fingerCenter.distance(containerCenter) < 100)
                                {
                                    found = true;
                                    for( int i = 0; i < (*it2)->selectedShapes.size() ; i++)
                                    {
                                        (*it3)->addLink((*it2)->selectedShapes[i]);
                                    }
                                }
                            }
                            containers.unlockVector();
                            if(found)
                            {
                                changeFingerType((*it));
                                (*it)->typeFinger = Finger::UNUSABLE;
                            }
                        }
                    }
                    break;
                }
                case Finger::NOTYPE:
                    break;
                default:
                    break;
            }
            (*it)->markForMoved = false;
        }
        else if((*it)->markForRemoval)
        {
            changeFingerType((*it));
            fingers.removeElement((*it));
        }
    }
    vector<Token *> temptokens = tokens.getObjects();
    for (std::vector<Token*>::iterator it=temptokens.begin() ; it < temptokens.end(); it++ )
    {
        (*it)->update();
        if((*it)->markForAdd)
        {
            if((*it)->getSymbolID() == MARKER_ID_SAVE) //shredder
            {
                
                if(saveToken==NULL)
                {
                    saveToken = new SaveToken((*it), &shapes);
                    
                    saveToken->setIsOnTable(true);
                }
                else
                {
                    saveToken->setToken(*it);
                    saveToken->setIsOnTable(true);
                }
                saveToken->update();
                
                
                
                (*it)->typeobject = Token::SAVE;
            }
            else if((*it)->getSymbolID() == MARKER_ID_ORBIT) //shredder
            {
                orbiter = new OrbitToken((*it), &containers, axis);
                hasOrbiter = true;
                (*it)->typeobject = Token::ORBITER;
            }
            else if((*it)->getSymbolID() == MARKER_ID_SHREDDER1) //shredder
            {
                shredder1 = new ShredderToken((*it));
                hasShredder1 = true;
                (*it)->typeobject = Token::SHREDDER1;
            }
            else if((*it)->getSymbolID()== MARKER_ID_SHREDDER2)
            {
                shredder2 = new ShredderToken((*it));
                hasShredder2 = true;
                (*it)->typeobject = Token::SHREDDER2;
            }
            else if((*it)->getSymbolID()==MARKER_ID_X_Z)
            {
                cameraScene.addRotationTween(0,45,0,1);
                axis->changeAxis(AxisPlane::X_Z);
                logo.loadImage("images/magentalogo.png");
                (*it)->typeobject = Token::MARKERXZ;
            }
            else if((*it)->getSymbolID()==MARKER_ID_X_Y)
            {
                cameraScene.addRotationTween(0, -45, 0,1);
                axis->changeAxis(AxisPlane::X_Y);
                logo.loadImage("images/yellowlogo.png");
                (*it)->typeobject = Token::MARKERXY;
            }
            else if((*it)->getSymbolID()==MARKER_ID_Y_Z)
            {
                cameraScene.addRotationTween(90,-45,0,1);
                axis->changeAxis(AxisPlane::Y_Z);
                logo.loadImage("images/cyanlogo.png");
                (*it)->typeobject = Token::MARKERYZ;
            }
            else if((*it)->getSymbolID()==MARKER_ID_ONTABLECCTOKEN)
            {
                if(onTableCCToken==NULL)
                {
                    onTableCCToken = new OnTableContentCreationUI((*it), &shapeEntryPoint, &shapes, &containers);
                }
                else
                {
                    onTableCCToken->setToken(*it);
                    onTableCCToken->setIsOnTable(true);
                }
                onTableCCToken->update();
                (*it)->typeobject = Token::ONTABLECC;
            }
            else if((*it)->getSymbolID()==MARKER_ID_ONTOKENCCTOKEN)
            {
                if(onTokenCCToken==NULL)
                {
                    onTokenCCToken = new OnTokenContentCreationUI((*it), &shapeEntryPoint, &shapes, &containers);
                }
                else
                {
                    onTokenCCToken->setToken(*it);
                    onTokenCCToken->setIsOnTable(true);
                }
                onTokenCCToken->update();
                (*it)->typeobject = Token::ONTOKENCC;
            }
            else if((*it)->getSymbolID()==MARKER_ID_INAIRCCTOKEN)
            {
                if(inAirCCToken==NULL)
                {
                    inAirCCToken = new InAirContentCreationUI((*it), &shapeEntryPoint, &shapes, &containers);
                }
                else
                {
                    inAirCCToken->setToken(*it);
                    inAirCCToken->setIsOnTable(true);
                }
                inAirCCToken->update();
                (*it)->typeobject = Token::INAIRCC;
            }
            else
            {
                vector<ContainerToken*> temp = containers.getObjects();
                containers.lockVector();
                bool found = false;
                ContainerToken * tempCont;
                for (std::vector<ContainerToken*>::iterator it2=temp.begin() ; it2 < temp.end(); it2++ )
                {
                    if((*it2)->getToken()->getSymbolID()==(*it)->getSymbolID() || (*it2)->getID()==(*it)->getSymbolID())
                    {
                        found = true;
                        tempCont = (*it2);
                    }
                }
                if(!found)
                {
                    tempCont = new ContainerToken((*it), viewport, axis, &cameraScene, useAlternativeManipulationUI, &shapes);
                    containers.addElement(tempCont);
                }
                else
                {
                    tempCont->setToken(*it);
                    tempCont->setIsOnTable(true);
                }
                (*it)->typeobject = Token::CONTAINER;
                containers.unlockVector();
            }
            (*it)->markForAdd = false;
        }
        else if((*it)->markForMoved)
        {
            switch ( (*it)->typeobject)
            {
                case Token::CONTAINER:
                {
                    vector<ContainerToken*> temp = containers.getObjects();
                    containers.lockVector();
                    bool found = false;
                    ContainerToken * tempCont;
                    for (std::vector<ContainerToken*>::iterator it2=temp.begin() ; it2 < temp.end(); it2++ )
                    {
                        if((*it2)->getToken()->getSymbolID()==(*it)->getSymbolID())
                        {
                            found = true;
                            tempCont = (*it2);
                        }
                    }
                    if(found)
                    {
                        tempCont->update();
                        if(hasShredder1 && hasShredder2)
                        {
                            vector<ofPoint> verts;
                            verts.push_back(ofPoint(shredder1->getToken()->getX()*ofGetWidth(), shredder1->getToken()->getY()*ofGetHeight()+20));
                            verts.push_back(ofPoint(shredder1->getToken()->getX()*ofGetWidth(), shredder1->getToken()->getY()*ofGetHeight()-20));
                            verts.push_back(ofPoint(shredder2->getToken()->getX()*ofGetWidth(), shredder2->getToken()->getY()*ofGetHeight()-20));
                            verts.push_back(ofPoint(shredder2->getToken()->getX()*ofGetWidth(), shredder2->getToken()->getY()*ofGetHeight()+20));
                            ofPolyline p;
                            p.addVertexes(verts);
                            p.close();
                            if(inside((*it)->getX()*ofGetWidth(), (*it)->getY()*ofGetHeight(), p))
                            {
                                shapes.lockVector();
                                for (std::vector<Shape3D*>::iterator it3= tempCont->links.begin() ; it3 <  tempCont->links.end(); it3++ )
                                {
                                    (*it3)->deleteShape();
                                    shapes.removeElement(*it3);
                                }
                                shapes.unlockVector();
                                tempCont->clearLinks();
                            }
                            else
                            {
                            }
                        }
                    }
                    containers.unlockVector();
                    break;
                }
                case Token::MARKERXZ:
                    break;
                case Token::MARKERXY:
                    break;
                case Token::MARKERYZ:
                    break;
                case Token::ONTABLECC:
                    onTableCCToken->update();
                    break;
                case Token::ONTOKENCC:
                    onTokenCCToken->update();
                    break;
                case Token::INAIRCC:
                    inAirCCToken->update();
                    break;
                case Token::NOTYPE:
                    break;
                case Token::SHREDDER1:
                {
                    if(drawAreaForContour.active)
                    {
                        drawAreaForContour.erase(shredder1->getToken()->getX()*ofGetWidth(), shredder1->getToken()->getY()*ofGetHeight(), 100);
                    }
                    break;
                }
                case Token::SHREDDER2:
                {
                    if(drawAreaForContour.active)
                    {
                        drawAreaForContour.erase(shredder2->getToken()->getX()*ofGetWidth(), shredder2->getToken()->getY()*ofGetHeight(), 100);
                    }
                    break;
                }
                case Token::ORBITER:
                {
                    break;
                }
                case Token::SAVE:
                {
                    saveToken->update();
                    break;
                }
                default:
                    break;
            }
            (*it)->markForMoved = false;
        }
        else if((*it)->markForRemoval)
        {
            changeMarkerType((*it));
            tokens.removeElement((*it));
        }
    }
    vector<ContainerToken*> temp = containers.getObjects();
    for (std::vector<ContainerToken*>::iterator it=temp.begin() ; it < temp.end(); it++ )
    {
        if((*it)->getIsOnTable())
        {
            vector< TokenData*> temp = tokensData.getObjects();
            for(vector< TokenData *>::iterator ito=temp.begin() ; ito < temp.end(); ito++ )
            {
                if((*ito)->getID()==(*it)->getID())
                {
                    //                    cout << (*ito)->zmax << "  ---> " << (*ito)->zcurrent<<endl;
                    //                    cout << (*ito)->zcurrent + 2 << endl;
                    //                    cout << (*ito)->zcurrent - 2 << endl;
                    if(!(
                         (*ito)->zmax < (*ito)->zcurrent + 2 && (*ito)->zmax > (*ito)->zcurrent  - 2 ))
                    {
                        //cout << "hover" << endl;
                        (*it)->hovering();
                    }
                    else
                    {
                        (*it)->noHovering();
                        // cout << "nohover" << endl;
                    }
                    // tempCont->boolLock->markerdata = (*ito);
                }
            }
            (*it)->update();
            (*it)->updateScale();
        }
    }
}

void testApp::changeMarkerType(Token * marker)
{
    switch (marker->typeobject)
    {
        case Token::CONTAINER:
        {
            vector<ContainerToken*> temp = containers.getObjects();
            containers.lockVector();
            bool found = false;
            ContainerToken * tempCont;
            for (std::vector<ContainerToken*>::iterator it2=temp.begin() ; it2 < temp.end(); it2++ )
            {
                if((*it2)->getToken()->getSymbolID()==marker->getSymbolID())
                {
                    found = true;
                    tempCont =(*it2);
                    break;
                }
            }
            if(found)
            {
                tempCont->setIsOnTable(false);
            }
            containers.unlockVector();
            break;
        }
        case Token::MARKERXZ:
            axis->changeAxis(AxisPlane::NOAXIS);
            break;
        case Token::MARKERXY:
            axis->changeAxis(AxisPlane::NOAXIS);
            break;
        case Token::MARKERYZ:
            axis->changeAxis(AxisPlane::NOAXIS);
            break;
        case Token::ONTABLECC:
            onTableCCToken->setIsOnTable(false);
            break;
        case Token::ONTOKENCC:
            onTokenCCToken->setIsOnTable(false);
            break;
        case Token::INAIRCC:
            inAirCCToken->setIsOnTable(false);
            break;
        case Token::NOTYPE:
            break;
        case Token::SHREDDER1:
            hasShredder1 = false;
            break;
        case Token::SHREDDER2:
            hasShredder2 = false;
            break;
        case Token::CALIBRATION:
            calibrationMarker->objectRemoved(marker);
            break;
        case Token::ORBITER:
            hasOrbiter = false;
            break;
        case Token::SAVE:
            saveToken->setIsOnTable(false);
            break;
            break;
        default:
            break;
    }
}

void testApp::changeFingerType(Finger * finger)
{
    switch ( finger->typeFinger)
    {
        case Finger::PICKER:
            finger->pickedShapes3D.clear();
            cameraScene.fingerPickerOut(finger);
            break;
        case Finger::CAMERA:
            cameraScene.fingerOut(finger);
            break;
        case Finger::ONTABLEOPTION:
            break;
        case Finger::CONTOUR:
        {
            bool inMenuCircle = false;
            if(onTableCCToken != NULL)
            {
                if(onTableCCToken->getIsOnTable())
                {
                    if(onTableCCToken->insideProtectedOptions(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                    {
                        inMenuCircle = true;
                    }
                }
            }
            if(onTokenCCToken != NULL)
            {
                if(onTokenCCToken->getIsOnTable())
                {
                    if(onTokenCCToken->insideProtectedOptions(finger->getX()*ofGetWidth(), finger->getY()*ofGetHeight()))
                    {
                        inMenuCircle = true;
                    }
                }
            }
            if(inAirCCToken != NULL)
            {
                if(inAirCCToken->getIsOnTable())
                {
                    inMenuCircle = false;
                }
            }
            if(!inMenuCircle)
            {
                drawAreaForContour.addVert(finger);
            }
        }
            break;
        case Finger::SCALECONTAINERNORTH:
        {
            vector<ContainerToken*> temp2 = containers.getObjects();
            for (std::vector<ContainerToken*>::iterator it2=temp2.begin() ; it2 < temp2.end(); it2++ )
            {
                if((*it2)->fingerOnEastScale != NULL)
                {
                    if((*it2)->fingerOnEastScale->getFingerID() == finger->getFingerID())
                    {
                        (*it2)->setHasEastScale(false);
                    }
                }
            }
        }
            break;
        case Finger::SCALECONTAINERSOUTH:
        {
            vector<ContainerToken*> temp2 = containers.getObjects();
            for (std::vector<ContainerToken*>::iterator it2=temp2.begin() ; it2 < temp2.end(); it2++ )
            {
                if((*it2)->fingerOnSouthScale != NULL)
                {
                    if((*it2)->fingerOnSouthScale->getFingerID() == finger->getFingerID())
                    {
                        (*it2)->setHasSouthScale(false);
                    }
                }
            }
        }
            break;
        case Finger::SHAPESELECTION:
        {
            vector<SelectionMovement*> temp = selectionMovements.getObjects();
            selectionMovements.lockVector();
            for (std::vector<SelectionMovement *>::iterator it2=temp.begin() ; it2 < temp.end(); it2++ )
            {
                if((*it2)->finger->getFingerID() == finger->getFingerID())
                {
                    (*it2)->cancel();
                    selectionMovements.removeElement((*it2));
                }
            }
            selectionMovements.unlockVector();
            break;
        }
        case Finger::NOTYPE:
            break;
        default:
            break;
    }
}


bool Descending(const float& d1, const float& d2)
{
    return d1 > d2;
}
void testApp::drawScene()
{
    float gridsize = 100.0f;
    if(shapes.getObjects().size()==0)
    {
    }
    else
    {
        float xMin=999999,xMax=-999999, yMin=999999, yMax=-999999, zMin = 999999, zMax =-999999;
        for (int i=0; i<shapes.getObjects().size(); i++)
        {
            vector<float> pt = shapes.getObjects()[i]->mesh.ofLimits();
            if(pt.size()==6)
            {
                xMin = min(xMin,pt.at(0));
                xMax = max(xMax,pt.at(1));
                yMin = min(yMin,pt.at(2));
                yMax = max(yMax,pt.at(3));
                zMin = min(zMin,pt.at(4));
                zMax = max(zMax,pt.at(5));
            }
        }
        vector<float> distances;
        distances.push_back((int)abs(xMin));
        distances.push_back((int)abs(xMax));
        distances.push_back((int)abs(yMin));
        distances.push_back((int)abs(yMax));
        distances.push_back((int)abs(zMin));
        distances.push_back((int)abs(zMax));
        sort(distances.begin(), distances.end(),Descending);
        gridsize = distances.front();
        float rest = (int) fmod(gridsize,10);
        while(rest != 0)
        {
            gridsize++;
            rest = (int) fmod(gridsize,10);
        }
        if(gridsize <100.0f)
        {
            gridsize = 100.0f;
        }
    }
    float ticks = gridsize/20;
    Grids tempgrid = Grids();
    switch (axis->axis)
    {
            //        case AxisPlane::NOAXIS:
            //            tempgrid.ofDrawGrid(gridsize,ticks,false,true,true,true);
            //            break;
            //        case AxisPlane::XY:
            //            tempgrid.ofDrawGrid(gridsize,ticks,false,true,true,true);
            //            break;
            //        case AxisPlane::YZ:
            //            tempgrid.ofDrawGrid(gridsize,ticks,false,true,true,true);
            //            break;
            //        case AxisPlane::ZX:
            //            tempgrid.ofDrawGrid(gridsize,ticks,false,true,true,true);
            //            break;
        case AxisPlane::NOAXIS:
            tempgrid.ofDrawGrid(gridsize,ticks,false,true,true,true);
            break;
        case AxisPlane::X_Z:
            tempgrid.ofDrawGrid(gridsize,ticks,false,false,true,false);
            break;
        case AxisPlane::X_Y:
            tempgrid.ofDrawGrid(gridsize,ticks,false,false,false,true);
            break;
        case AxisPlane::Y_Z:
            tempgrid.ofDrawGrid(gridsize,ticks,false,true,false,false);
            break;
        default:
            break;
    }
    ofSetColor(COLORSCHEME_RED);
    ofSetLineWidth(30);
    ofLine(ofVec3f(-gridsize,0,0), ofVec3f(gridsize,0,0));
    ofSetColor(COLORSCHEME_GREEN);
    ofSetLineWidth(30);
    ofLine(ofVec3f(0,-gridsize,0), ofVec3f(0,gridsize,0));
    ofSetColor(COLORSCHEME_BLUE);
    ofSetLineWidth(30);
    ofLine(ofVec3f(0,0,-gridsize), ofVec3f(0,0,gridsize));
}


void testApp::objectAdded(TuioObject & obj)
{
    vector<Token *> temptokens = tokens.getObjects();
    bool found = false;
    for (std::vector<Token *>::iterator it=temptokens.begin() ; it < temptokens.end() && !found; it++ )
    {
        if(calibratingMarker)
        {
            if ((*it)->checkToken(&obj))
            {
                found = true;
            }
        }
        else
        {
            ofVec2f tempcalib = calibrationMarker->applyCalibration(obj.getX()*640, obj.getY()*480);
            if ((*it)->checkToken(&obj, tempcalib.x, tempcalib.y))
            {
                found = true;
            }
        }
    }
    if(!found)
    {
        tokens.lockVector();
        if(calibratingMarker)
        {
            tokens.addElement(new Token(&obj));
        }
        else
        {
            ofVec2f tempcalib = calibrationMarker->applyCalibration(obj.getX()*640, obj.getY()*480);
            tokens.addElement(new Token(&obj, tempcalib.x, tempcalib.y));
        }
        tokens.unlockVector();
    }
}

void testApp::objectUpdated(TuioObject & obj)
{
    vector<Token *> temptokens = tokens.getObjects();
    bool found = false;
    for (std::vector<Token*>::iterator it=temptokens.begin() ; it < temptokens.end() && !found; it++ )
    {
        if(calibratingMarker)
        {
            if ((*it)->checkToken(&obj))
            {
                found = true;
                (*it)->updateMoved();
            }
        }
        else
        {
            ofVec2f tempcalib = calibrationMarker->applyCalibration(obj.getX()*640, obj.getY()*480);
            if ((*it)->checkToken(&obj, tempcalib.x, tempcalib.y))
            {
                found = true;
                (*it)->updateMoved(tempcalib.x, tempcalib.y);
            }
        }
    }
}

void testApp::objectRemoved(TuioObject & obj)
{
    vector<Token *> temptokens = tokens.getObjects();
    bool found = false;
    for (std::vector<Token*>::iterator it = temptokens.begin() ; it < temptokens.end() && !found; it++ )
    {
        if ((*it)->getSymbolID() == obj.getSymbolID())
        {
            found = true;
            if(!(*it)->hasBeenAdded)
            {
                (*it)->markForRemoval = true;
            }
            else
            {
                (*it)->hasBeenRemoved = true;
            };
        }
    }
}

bool testApp::inside(float x, float y, ofPolyline polyline)
{
    int counter = 0;
    int i;
    double xinters;
    ofPoint p1,p2;
    int N = polyline.size();
    p1 = polyline[0];
    for (i=1; i<=N; i++)
    {
        p2 = polyline[i % N];
        if (y > MIN(p1.y,p2.y))
        {
            if (y <= MAX(p1.y,p2.y))
            {
                if (x <= MAX(p1.x,p2.x))
                {
                    if (p1.y != p2.y)
                    {
                        xinters = (y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                        if (p1.x == p2.x || x <= xinters)
                            counter++;
                    }
                }
            }
        }
        p1 = p2;
    }
    if (counter % 2 == 0) return false;
    else return true;
}

void testApp::updateCalibrationFingers()
{
    stitchFingers();
    
    vector<Finger *> tempfingers = fingers.getObjects();
    for (std::vector<Finger*>::iterator it=tempfingers.begin() ; it < tempfingers.end(); it++ )
    {
        (*it)->update();
        if((*it)->markForAdd)
        {
            if((*it)->getZForSurfacePlane() < 100 )
            {
                calibrationFinger->touchDown((*it));
                (*it)->typeFinger = Finger::CALIBRATION;
            }
            (*it)->markForAdd = false;
        }
        else if((*it)->markForMoved)
        {
            if((*it)->getZForSurfacePlane() > 100 && (*it)->typeFinger == Finger::CALIBRATION)
            {
                calibrationFinger->touchUp((*it));
                (*it)->typeFinger = Finger::NOTYPE;
            }
            else if((*it)->getZForSurfacePlane() < 100 && (*it)->typeFinger != Finger::CALIBRATION)
            {
                calibrationFinger->touchDown((*it));
                (*it)->typeFinger = Finger::CALIBRATION;
            }
            (*it)->markForMoved = false;
        }
        else if((*it)->markForRemoval)
        {
            if((*it)->typeFinger == Finger::CALIBRATION)
            {
                calibrationFinger->touchUp((*it));
            }
            fingers.removeElement((*it));
        }
    }
    calibrationFinger->update();
}

void testApp::drawCalibrationFingers()
{
    calibrationFinger->doCalibration();
}

void testApp::updateCalibrationMarker()
{
    vector<Token *> temptokens = tokens.getObjects();
    for (std::vector<Token*>::iterator it=temptokens.begin() ; it < temptokens.end(); it++ )
    {
        (*it)->update();
        if((*it)->markForAdd)
        {
            changeMarkerType((*it));
            calibrationMarker->objectAdded((*it));
            (*it)->typeobject= Token::CALIBRATION;
            (*it)->markForAdd = false;
        }
        else if((*it)->markForMoved)
        {
            if((*it)->typeobject!= Token::CALIBRATION)
            {
                changeMarkerType((*it));
                calibrationMarker->objectAdded((*it));
                (*it)->typeobject= Token::CALIBRATION;
            }
            (*it)->markForMoved = false;
        }
        else if((*it)->markForRemoval)
        {
            changeMarkerType((*it));
            if((*it)->typeobject == Token::CALIBRATION)
            {
                calibrationMarker->objectRemoved((*it));
            }
            tokens.removeElement((*it));
        }
    }
    calibrationMarker->update();
}

void testApp::drawCalibrationMarker()
{
    calibrationMarker->doCalibration();
}

bool testApp::loadXML()
{
    if( xmlfile.loadFile("planeSettings.xml")) {}
    else
        //FAIL!
        return false;
    normalUnitVec[0] = xmlfile.getValue("PLANE:NORMALX",  0.000000);
    normalUnitVec[1] = xmlfile.getValue("PLANE:NORMALY",  0.000000);
    normalUnitVec[2] = xmlfile.getValue("PLANE:NORMALZ",  0.000000);
    addNormalX = xmlfile.getValue("PLANE:ADDNORMALX",  0.000000);
    addNormalY = xmlfile.getValue("PLANE:ADDNORMALY",  0.000000);
    addNormalZ = xmlfile.getValue("PLANE:ADDNORMALZ",  0.000000);
    distancePlane = xmlfile.getValue("PLANE:DISTANCEPLANE",  0.000000);
    distanceForPlaneAboveSurface =  xmlfile.getValue("PLANE:distanceForPlaneAboveSurface",  0.000000);
    distanceForPlaneAboveObjects = xmlfile.getValue("PLANE:distanceForPlaneAboveObjects",  0.000000);
    ltCorner->x = xmlfile.getValue("CORNERS:LTCORNER:X",  0.000000);
    ltCorner->y = xmlfile.getValue("CORNERS:LTCORNER:Y",  0.000000);
    rtCorner->x = xmlfile.getValue("CORNERS:RTCORNER:X",  0.000000);
    rtCorner->y = xmlfile.getValue("CORNERS:RTCORNER:Y",  0.000000);
    lbCorner->x = xmlfile.getValue("CORNERS:LBCORNER:X",  0.000000);
    lbCorner->y = xmlfile.getValue("CORNERS:LBCORNER:Y",  0.000000);
    rbCorner->x = xmlfile.getValue("CORNERS:RBCORNER:X",  0.000000);
    rbCorner->y = xmlfile.getValue("CORNERS:RBCORNER:Y",  0.000000);
    return true;
}

void testApp::saveXML()
{
    xmlfile.setValue("PLANE:NORMALX", normalUnitVec[0]);
    xmlfile.setValue("PLANE:NORMALY", normalUnitVec[1]);
    xmlfile.setValue("PLANE:NORMALZ", normalUnitVec[2]);
    xmlfile.setValue("PLANE:ADDNORMALX", addNormalX);
    xmlfile.setValue("PLANE:ADDNORMALY", addNormalY);
    xmlfile.setValue("PLANE:ADDNORMALZ", addNormalZ);
    xmlfile.setValue("PLANE:DISTANCEPLANE", distancePlane);
    xmlfile.setValue("PLANE:distanceForPlaneAboveSurface", distanceForPlaneAboveSurface);
    xmlfile.setValue("PLANE:distanceForPlaneAboveObjects", distanceForPlaneAboveObjects);
    xmlfile.setValue("CORNERS:LTCORNER:X", ltCorner->x);
    xmlfile.setValue("CORNERS:LTCORNER:Y", ltCorner->y);
    xmlfile.setValue("CORNERS:RTCORNER:X",  rtCorner->x);
    xmlfile.setValue("CORNERS:RTCORNER:Y", rtCorner->y );
    xmlfile.setValue("CORNERS:LBCORNER:X",lbCorner->x);
    xmlfile.setValue("CORNERS:LBCORNER:Y",lbCorner->y);
    xmlfile.setValue("CORNERS:RBCORNER:X", rbCorner->x);
    xmlfile.setValue("CORNERS:RBCORNER:Y", rbCorner->y);
    xmlfile.saveFile("planeSettings.xml");
}