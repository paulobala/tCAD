#include "ofEasyFingerCam.h"
#include "ofMath.h"
#include "ofUtils.h"

// when an ofEasyCam is moving due to momentum, this keeps it
// from moving forever by assuming small values are zero.
float epsilonTransform = 1e-7;

// this is the default on windows os
unsigned long doubleclickTime = 500;

//----------------------------------------
ofEasyFingerCam::ofEasyFingerCam()
{
    selectedPlane = new AxisPlane();
    drag = 0.1f;
    zoomSpeed = 2.0f;
    bMouseInputEnabled = false;
    bFingerInputEnabled = false;
    mousePosViewPrev.set(0, 0);
    lastFrame = 0;
    bDistanceSet = false;
    lastDistance = 0;
    distanceScaleVelocity = 0;
    fingers = std::vector<Finger *>();
    target.setPosition(0, 0, 0);
    zooming = true;
    reset();
    setParent(target);
    prevDistance = 0;
    ofVec3f center = ofVec3f(0.0f, 0.0f, 0.0f);
    ofVec3f normal = ofVec3f(0.0f, 1.0f, 0.0f).normalize();
    ofVec2f scale = ofVec2f(1.0f, 1.0f);
    enableMouseInput();
    enableFingerInput();
}

//----------------------------------------
ofEasyFingerCam::~ofEasyFingerCam()
{
}


void ofEasyFingerCam::addRotationTween(int ax, int ay, int az, int speed)
{
    rTweens.push_back(new RotationTween(ax, ay, az, speed));
}

void ofEasyFingerCam::addScaleTween(float scale)
{
    sTweens.push_back(new ScalingTween(scale));
}

void ofEasyFingerCam::addPanningTween(ofVec3f value)
{
    pTweens.push_back(new PanningTween(value));
}
//----------------------------------------
void ofEasyFingerCam::begin(ofRectangle viewport)
{
    glEnable(GL_DEPTH_TEST);
    viewportRect = viewport;
    ofCamera::begin(viewport);
    ofPushMatrix();
    glGetDoublev(GL_PROJECTION_MATRIX, this->matP);
    glGetDoublev(GL_MODELVIEW_MATRIX, this->matM);
    glGetIntegerv(GL_VIEWPORT, this->viewport);
    bool hasTweens = false;
    if(rTweens.size()>0)
    {
        hasTweens = true;
        setAnglesFromOrientation();
        targetXRot = rTweens[0]->x;
        targetYRot = rTweens[0]->y;
        targetZRot = rTweens[0]->z;
        updateRotation();
        if(targetXRot - rotationX <1 && targetXRot - rotationX >-1 &&
           targetYRot - rotationY <1 && targetYRot - rotationY >-1 &&
           targetZRot - rotationZ <1 && targetZRot - rotationZ >-1)
        {
            rTweens.erase(rTweens.begin());
        }
        setAnglesFromOrientation();
    }
    if(sTweens.size()>0)
    {
        hasTweens = true;
        if(abs(sTweens.at(0)->scale - getDistance()) < 10)
        {
            float newvalue = (getDistance()-sTweens.at(0)->scale)*0.05/100;
            setDistance(getDistance()+newvalue);
        }
    }
    if(pTweens.size()>0)
    {
        hasTweens = true;
        if(target.getPosition().distance(pTweens.at(0)->pan) > 1)
        {
            ofVec3f newTranslation;
            newTranslation = pTweens.at(0)->pan - target.getPosition();
            translation = newTranslation/10;
            target.move(translation);
        }
        else
        {
            pTweens.erase(pTweens.begin());
        }
    }
    if(hasTweens)
    {
        currentState = TWEENING;
    }
    else
    {
        currentState = STABLE;
        if(bMouseInputEnabled||bFingerInputEnabled)
        {
            if(!bDistanceSet)
            {
                setDistance(getImagePlaneDistance(viewport), true);
            }
            if (fingers.size() > 0 )
            {
                // it's important to check whether we've already accounted for the mouse
                // just in case you use the camera multiple times in a single frame
                if (lastFrame != ofGetFrameNum())
                {
                    lastFrame = ofGetFrameNum();
                    currentState = STABLE;
                    if (fingers.size() == 1)
                    {
                        if(selectedPlane->axis == AxisPlane::NOAXIS)
                        {
                            currentState = ROTATING;
                            // if there is some smart way to use dt to scale the values drag, we should do it
                            // you can't simply multiply drag etc because the behavior is unstable at high framerates
                            // float dt = ofGetLastFrameTime();
                            ofVec2f mousePosScreen = ofVec3f(fingers[0]->getX()*ofGetWidth() - viewport.width/2 - viewport.x, viewport.height/2 - (fingers[0]->getY()*ofGetHeight() - viewport.y), 0);
                            ofVec2f mouseVelScreen = (mousePosScreen - mousePosScreenPrev).lengthSquared();
                            ofVec3f targetPos =  target.getGlobalPosition();
                            ofVec3f mousePosXYZ = ofVec3f(mousePosScreen.x, mousePosScreen.y, targetPos.z);
                            float sphereRadius = min(viewport.width, viewport.height)/2;
                            float diffSquared = sphereRadius * sphereRadius - (targetPos - mousePosXYZ).lengthSquared();
                            if(diffSquared <= 0)
                            {
                                mousePosXYZ.z = 0;
                            }
                            else
                            {
                                mousePosXYZ.z = sqrtf(diffSquared);
                            }
                            mousePosXYZ.z += targetPos.z;
                            ofVec3f mousePosView = ofMatrix4x4::getInverseOf(target.getGlobalTransformMatrix()) * mousePosXYZ;
                            //calc new rotation velocity
                            ofQuaternion newRotation;
                            if(fingerPressedPrev[0])
                            {
                                newRotation.makeRotate(mousePosViewPrev, mousePosView);
                            }
                            fingerPressedPrev[0] = true;
                            //apply drag towards new velocities
                            rotation.slerp(drag, rotation, newRotation); // TODO: add dt
                            mousePosViewPrev = ofMatrix4x4::getInverseOf(target.getGlobalTransformMatrix()) * mousePosXYZ;
                            // apply transforms if they're big enough
                            // TODO: these should be scaled by dt
                            if(translation.lengthSquared() > epsilonTransform)
                            {
                                // TODO: this isn't quite right, it needs to move wrt the rotation
                                target.move(translation);
                            }
                            if (rotation.asVec3().lengthSquared() > epsilonTransform)
                            {
                                target.rotate(rotation.conj());
                            }
                            if (abs(distanceScaleVelocity - 1.0f) > epsilonTransform)
                            {
                                setDistance(getDistance() * (1.0f + distanceScaleVelocity), false);
                            }
                            mousePosScreenPrev = mousePosScreen;
                            // targetFut.setPosition(target.getPosition());
                        }
                    }
                    if (fingers.size() == 2)
                    {
                        currentState = SCALING;
                        if(zooming)
                        {
                            ofVec2f pointa = ofVec2f(fingers[0]->getX()*ofGetWidth(),fingers[0]->getY()*ofGetHeight());
                            ofVec2f pointb = ofVec2f(fingers[1]->getX()*ofGetWidth(),fingers[1]->getY()*ofGetHeight());
                            float newDistance = pointa.distance(pointb);
                            float newDistanceScaleVelocity = 0.0f;
                            if(prevDistance == 0)
                            {
                                prevDistance = newDistance;
                            }
                            else
                            {
                                newDistanceScaleVelocity = zoomSpeed * ( prevDistance - newDistance) / ofVec2f(0,0).distance(ofVec2f(ofGetHeight(),ofGetWidth()));
                                distanceScaleVelocity = ofLerp(distanceScaleVelocity, newDistanceScaleVelocity, drag);
                                if (abs(distanceScaleVelocity - 1.0f) > epsilonTransform)
                                {
                                    setDistance(getDistance() * (1.0f + distanceScaleVelocity), false);
                                }
                                prevDistance = newDistance;
                            }
                        }
                    }
                    else
                    {
                        prevDistance = 0;
                    }
                    if (fingers.size() == 3)
                    {
                        currentState = POINTING;
                    }
                    if (fingers.size() == 5)
                    {
                        if(selectedPlane->axis == AxisPlane::NOAXIS)
                        {
                            currentState = PANNING;
                            //DECIDE LATER
                        }
                    }
                    if (fingers.size() == 10)
                    {
                        if(selectedPlane->axis == AxisPlane::NOAXIS)
                        {
                            currentState = RESET;
                            addPanningTween(ofVec3f(0,0,0));
                            addRotationTween(0, 0, 0, 1);
                        }
                    }
                }
            }
            else
            {
                //MOUSE
                fingerPressedPrev[0] = false;
                // it's important to check whether we've already accounted for the mouse
                // just in case you use the camera multiple times in a single frame
                if (lastFrame != ofGetFrameNum())
                {
                    lastFrame = ofGetFrameNum();
                    if(selectedPlane->axis == AxisPlane::NOAXIS)
                    {
                        // if there is some smart way to use dt to scale the values drag, we should do it
                        // you can't simply multiply drag etc because the behavior is unstable at high framerates
                        // float dt = ofGetLastFrameTime();
                        currentState = STABLE;
                        ofVec2f mousePosScreen = ofVec3f(ofGetMouseX() - viewport.width/2 - viewport.x, viewport.height/2 - (ofGetMouseY() - viewport.y), 0);
                        ofVec2f mouseVelScreen = (mousePosScreen - mousePosScreenPrev).lengthSquared();
                        ofVec3f targetPos =  target.getGlobalPosition();
                        ofVec3f mousePosXYZ = ofVec3f(mousePosScreen.x, mousePosScreen.y, targetPos.z);
                        float sphereRadius = min(viewport.width, viewport.height)/2;
                        float diffSquared = sphereRadius * sphereRadius - (targetPos - mousePosXYZ).lengthSquared();
                        if(diffSquared <= 0)
                        {
                            mousePosXYZ.z = 0;
                        }
                        else
                        {
                            mousePosXYZ.z = sqrtf(diffSquared);
                        }
                        mousePosXYZ.z += targetPos.z;
                        ofVec3f mousePosView = ofMatrix4x4::getInverseOf(target.getGlobalTransformMatrix()) * mousePosXYZ;
                        bool mousePressedCur[] = {ofGetMousePressed(0), ofGetMousePressed(2)};
                        //calc new rotation velocity
                        ofQuaternion newRotation;
                        if(mousePressedPrev[0] && mousePressedCur[0])
                        {
                            newRotation.makeRotate(mousePosViewPrev, mousePosView);
                        }
                        //calc new scale velocity
                        float newDistanceScaleVelocity = 0.0f;
                        if(mousePressedPrev[1] && mousePressedCur[1])
                        {
                            newDistanceScaleVelocity = zoomSpeed * (mousePosScreenPrev.y - mousePosScreen.y) / viewport.height;
                        }
                        mousePressedPrev[0] = mousePressedCur[0];
                        mousePressedPrev[1] = mousePressedCur[1];
                        ofVec3f newTranslation;
                        // TODO: this doesn't work at all. why not?
                        if(ofGetMousePressed() && ofGetKeyPressed(OF_KEY_SHIFT))
                        {
                            newTranslation = mousePosScreenPrev - mousePosScreen;
                        }
                        //apply drag towards new velocities
                        distanceScaleVelocity = ofLerp(distanceScaleVelocity, newDistanceScaleVelocity, drag); // TODO: add dt
                        rotation.slerp(drag, rotation, newRotation); // TODO: add dt
                        translation.interpolate(newTranslation, drag);
                        mousePosViewPrev = ofMatrix4x4::getInverseOf(target.getGlobalTransformMatrix()) * mousePosXYZ;
                        // apply transforms if they're big enough
                        // TODO: these should be scaled by dt
                        if(translation.lengthSquared() > epsilonTransform)
                        {
                            // TODO: this isn't quite right, it needs to move wrt the rotation
                            target.move(translation);
                        }
                        if (rotation.asVec3().lengthSquared() > epsilonTransform)
                        {
                            target.rotate(rotation.conj());
                        }
                        if (abs(distanceScaleVelocity - 1.0f) > epsilonTransform)
                        {
                            setDistance(getDistance() * (1.0f + distanceScaleVelocity), false);
                        }
                        mousePosScreenPrev = mousePosScreen;
                        //targetFut.setPosition(target.getPosition());
                    }
                }
            }
        }
        setAnglesFromOrientation();
    }
    ofCamera::begin(viewport);
}
void ofEasyFingerCam::end()
{
    findCursor();
    // this has to happen after all drawing + findCursor()
    // but before camera.end()
    ofPushStyle();
    ofSetColor(ofColor::black);
    ofSphere(cursor.x, cursor.y, cursor.z, 1);
    ofSphere(cursor.x-1, cursor.y, cursor.z, 1);
    ofSphere(cursor.x+1, cursor.y, cursor.z, 1);
    ofSphere(cursor.x, cursor.y-1, cursor.z, 1);
    ofSphere(cursor.x, cursor.y+1, cursor.z, 1);
    ofSphere(cursor.x, cursor.y, cursor.z-1, 1);
    ofSphere(cursor.x, cursor.y, cursor.z+1, 1);
    ofPopStyle();
    //optimistically, we presume there's no stray push/pops
    ofPopMatrix();
    ofCamera::end();
    glDisable(GL_DEPTH_TEST);
    if(fingers.size()==3)
    {
        ofVec2f mouseP;
        mouseP.x = (fingers[0]->getX()*ofGetWidth()+ fingers[1]->getX()*ofGetWidth() +fingers[2]->getX()*ofGetWidth())/3;
        mouseP.y = (fingers[0]->getY()*ofGetHeight()+ fingers[1]->getY()*ofGetHeight()+ fingers[2]->getY()*ofGetHeight())/3;
        if (viewportRect.inside(mouseP))
        {
            ofPushStyle();
            ofFill();
            ofSetColor(50, 10, 10);
            ofRect(mouseP.x + 20, mouseP.y + 20, 80, 40);
            stringstream ss;
            ss << "x: " << ofToString(cursor.x, 2) << endl;
            ss << "y: " << ofToString(cursor.y, 2) << endl;
            ss << "z: " << ofToString(cursor.z, 2) << endl;
            ofSetColor(255, 255, 255);
            ofDrawBitmapString(ss.str(), mouseP.x + 30, mouseP.y + 30);
            ofPopStyle();
        }
    }
}

//----------------------------------------
void ofEasyFingerCam::reset()
{
    rotationX = 0.0f;
    rotationY = 0.0f;
    rotationZ = 0.0f;
    targetXRot = 0.0f;
    targetYRot = 0.0f;
    targetZRot = 0.0f;
    target.resetTransform();
    setDistance(lastDistance, false);
    rotation = ofQuaternion(0,0,0,1);
    distanceScaleVelocity = 0;
}

//----------------------------------------
void ofEasyFingerCam::setTarget(const ofVec3f& targetPoint)
{
    target.setPosition(targetPoint);
}

//----------------------------------------
void ofEasyFingerCam::setTarget(ofNode& targetNode)
{
    target.setPosition(ofVec3f(0, 0, 0));
    target.setParent(targetNode);
}

//----------------------------------------
ofNode& ofEasyFingerCam::getTarget()
{
    return target;
}

//----------------------------------------
void ofEasyFingerCam::setDistance(float distance)
{
    setDistance(distance, true);
}

//----------------------------------------
void ofEasyFingerCam::setDistance(float distance, bool save)
{
    if (distance > 0.0f)
    {
        if(save)
        {
            this->lastDistance = distance;
        }
        setPosition(0, 0, distance);
        bDistanceSet = true;
    }
}

//----------------------------------------
float ofEasyFingerCam::getDistance() const
{
    return getPosition().z;
}

//----------------------------------------
void ofEasyFingerCam::setDrag(float drag)
{
    this->drag = drag;
}

//----------------------------------------
float ofEasyFingerCam::getDrag() const
{
    return drag;
}

//----------------------------------------
void ofEasyFingerCam::enableMouseInput()
{
    if(!bMouseInputEnabled)
    {
        bMouseInputEnabled = true;
        ofRegisterMouseEvents(this);
    }
}


//----------------------------------------
void ofEasyFingerCam::disableMouseInput()
{
    if(bMouseInputEnabled)
    {
        bMouseInputEnabled = false;
        ofUnregisterMouseEvents(this);
    }
}

//----------------------------------------
bool ofEasyFingerCam::getMouseInputEnabled()
{
    return bMouseInputEnabled;
}


//----------------------------------------
void ofEasyFingerCam::mouseDragged(ofMouseEventArgs& mouse)
{
}

//----------------------------------------
void ofEasyFingerCam::mouseMoved(ofMouseEventArgs& mouse)
{
}

//----------------------------------------
void ofEasyFingerCam::mousePressed(ofMouseEventArgs& mouse)
{
}

//----------------------------------------
void ofEasyFingerCam::mouseReleased(ofMouseEventArgs& mouse)
{
}

//----------------------------------------
void ofEasyFingerCam::enableFingerInput()
{
    if(!bFingerInputEnabled)
    {
        bFingerInputEnabled = true;
    }
}


//----------------------------------------
void ofEasyFingerCam::disableFingerInput()
{
    if(bFingerInputEnabled)
    {
        bFingerInputEnabled = false;
    }
}

//----------------------------------------
bool ofEasyFingerCam::getFingerInputEnabled()
{
    return bFingerInputEnabled;
}

//----------------------------------------
void ofEasyFingerCam::fingerIn(Finger* finger)
{
    fingers.push_back(finger);
}
//----------------------------------------
void ofEasyFingerCam::fingerMoved(Finger* finger)
{
}

void ofEasyFingerCam::fingerOut(Finger * finger)
{
    bool found = false;
    for ( vector<Finger *>::iterator it=fingers.begin() ; it < fingers.end() && !found; it++ )
    {
        if((*it)->getFingerID()==finger->getFingerID())
        {
            found = true;
            fingers.erase(it);
        }
    }
}


void ofEasyFingerCam::fingerPickerIn(Finger* finger)
{
    fingersPickers.push_back(finger);
}
void ofEasyFingerCam::fingerPickerMoved(Finger* finger)
{
}
void ofEasyFingerCam::fingerPickerOut(Finger * finger)
{
    bool found = false;
    for ( vector<Finger *>::iterator it=fingersPickers.begin() ; it < fingersPickers.end() && !found; it++ )
    {
        if((*it)->getFingerID()==finger->getFingerID())
        {
            found = true;
            fingersPickers.erase(it);
        }
    }
}


#define ofEasyFingerCam_SEARCH_WIDTH 0.3f
#define ofEasyFingerCam_SEARCH_MAX_ITERATIONS 300
#define ofEasyFingerCam_SEARCH_WINDINGS 3.0f
void ofEasyFingerCam::findCursor()
{
    //read z value from depth buffer at mouse coords
    if(fingers.size()==3)
    {
        ofVec3f mouseP;
        mouseP.x = (fingers[0]->getX()*ofGetWidth()+fingers[1]->getX()*ofGetWidth()+fingers[2]->getX()*ofGetWidth())/3;
        mouseP.y = (fingers[0]->getY()*ofGetHeight()+fingers[1]->getY()*ofGetHeight()+fingers[2]->getY()*ofGetHeight())/3;
        if (mouseP.z == 1.0f)
            mouseP.z = 0.5f;
        glReadPixels(mouseP.x, ofGetHeight()-1-mouseP.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &mouseP.z);
        //if we get nothing, scatter until we get something
        //we search in a spiral until we hit something
        if (mouseP.z == 1.0f)
        {
            float sx, sy; // search this spot in screen space
            float r, theta; // search is in polar coords
            for (int iteration=0; iteration < ofEasyFingerCam_SEARCH_MAX_ITERATIONS; iteration++)
            {
                r = ofEasyFingerCam_SEARCH_WIDTH * float(iteration) / float(ofEasyFingerCam_SEARCH_MAX_ITERATIONS);
                theta = ofEasyFingerCam_SEARCH_WINDINGS * 2 * PI * float(iteration) / float(ofEasyFingerCam_SEARCH_MAX_ITERATIONS);
                sx = ofGetWidth() * r * cos(theta) + mouseP.x;
                sy = ofGetHeight() * r * sin(theta) + mouseP.y;
                if (!viewportRect.inside(sx, sy))
                    continue;
                glReadPixels(sx, ofGetViewportHeight()-1-sy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &mouseP.z);
                if (mouseP.z != 1.0f)
                    break;
            }
        }
        if (mouseP.z == 1.0f)
            return;
        GLdouble c[3];
        gluUnProject(mouseP.x, ofGetHeight()-1-mouseP.y, mouseP.z, matM, matP, viewport, c, c+1, c+2);
        cursor.x = c[0];
        cursor.y = c[1];
        cursor.z = c[2];
    }
}

std::vector<Shape3D*> ofEasyFingerCam::glSelect(int x, int y)
{
    std::vector<Shape3D*> result = std::vector<Shape3D*>();
    GLuint buff[512] = {0};
    GLint hits, view[4];
    GLfloat proj_matrix[16];
    /*
     This choose the buffer where store the values for the selection data
     */
    glSelectBuffer(256, buff);
    /*
     This retrieves info about the viewport
     */
    glGetIntegerv(GL_VIEWPORT, view);
    glGetFloatv(GL_PROJECTION_MATRIX, proj_matrix);
    /*
     Switching in selecton mode
     */
    glRenderMode(GL_SELECT);
    /*
     Clearing the names' stack
     This stack contains all the info about the objects
     */
    glInitNames();
    /*
     Now modify the viewing volume, restricting selection area around the cursor
     */
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    /*
     restrict the draw to an area around the cursor
     */
    gluPickMatrix(x, ofGetHeight() - y, 15.0, 15.0, view);

    float width = ofGetWidth();
    float height = ofGetHeight();
    float viewW = ofGetViewportWidth();
    float viewH = ofGetViewportHeight();
    float fov = 60;
    float eyeX = viewW / 2;
    float eyeY = viewH / 2;
    float halfFov = PI * fov / 360;
    float theTan = tanf(halfFov);
    float dist = eyeY / theTan;
    float aspect = (float) viewW / viewH;
    float nearDist = dist / 10.0f;
    float farDist = dist * 15.0f;
  
    gluPerspective(fov, aspect, nearDist, farDist);
    /*
     Draw the objects onto the screen
     */
    glMatrixMode(GL_MODELVIEW);
    /*
     draw only the names in the stack, and fill the array
     */
    draw();
    
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    /*
     get number of objects drawed in that area
     and return to render mode
     */
    hits = glRenderMode(GL_RENDER);
    /*
     Print a list of the objects
     */
    result = list_hits(hits, buff);
    glMatrixMode(GL_MODELVIEW);
    return result;
}

void ofEasyFingerCam::draw()
{
    vector<Shape3D*> temp = shapes->getObjects();
    shapes->lockVector();
    int i = 0;
    for (std::vector<Shape3D*>::iterator it=temp.begin() ; it < temp.end(); it++ )
    {
        if((*it)!=NULL)
        {
            glPushName(i);
            (*it)->draw();
            glPopName();
            i++;
        }
    }
    shapes->unlockVector();
}

void ofEasyFingerCam::checkSelectable()
{
    if(fingersPickers.size()>=1)
    {
        if(shapes!=NULL)
        {
            draw();
            if (shapes->getObjects().size() > 0)
            {
                for(int i = 0; i< fingersPickers.size(); i++)
                {
                    std::vector<Shape3D*> result = glSelect(fingersPickers[i]->getX()*ofGetWidth(), fingersPickers[i]->getY()*ofGetHeight());
                    for( std::vector<Shape3D*>::iterator it = result.begin(); it < result.end(); it++)
                    {
                        bool found = true;
                        for( std::vector<Shape3D*>::iterator it2 = fingersPickers[i]->pickedShapes3D.begin(); it2 < fingersPickers[i]->pickedShapes3D.end() && !found; it2++)
                        {
                            if((*it)==(*it2))
                            {
                                found = true;
                            }
                        }
                        if(found)
                        {
                            fingersPickers[i]->pickedShapes3D.push_back((*it));
                        }
                    }
                }
            }
        }
    }
    else
    {
        if(shapes!=NULL)
        {
            draw();
        }
    }
}

std::vector<Shape3D*> ofEasyFingerCam::list_hits(GLint hits, GLuint *buffer)
{
    unsigned int j;
    std::vector<Shape3D*> result = std::vector<Shape3D*>();
    GLuint *ptr, minZ, minminZ, nearestId, *ptrNames, numberOfNames;
    ptr = (GLuint *) buffer;
    minminZ = 0xffffffff;
    for (int i = 0; i < hits; i++)
    {
        numberOfNames = *ptr;
        ptr++;
        minZ = *ptr;
        ptrNames = ptr+2;
        if(minminZ>minZ && numberOfNames>0)
        {
            minminZ = minZ;
            nearestId = ptrNames[0];
        }
    
        for (j = 0; j < numberOfNames; j++,ptrNames++)
        {
            result.push_back(shapes->getObjects()[*ptrNames]);
        }
     
        ptr += numberOfNames+2;
    }
    return result;
}

void ofEasyFingerCam::setAnglesFromOrientation()
{
    ofVec3f rotation = target.getOrientationEuler();
    rotationX = targetXRot = -rotation.y;
    rotationY = targetYRot = -rotation.z;
    rotationZ = targetZRot = -rotation.x;
}

void ofEasyFingerCam::updateRotation()
{
    rotationX += (targetXRot - rotationX) *.1;
    rotationY += (targetYRot - rotationY) *.1;
    rotationZ += (targetZRot - rotationZ) *.1;
    target.setOrientation(ofQuaternion(0,0,0,1)); //reset
    ofQuaternion p = ((getOrientationQuat() * ofQuaternion(-rotationY, getXAxis())) * ofQuaternion(-rotationX, getYAxis())) * ofQuaternion(-rotationZ, getZAxis());
    target.setOrientation(p);
}

void ofEasyFingerCam::clearFingersPickers()
{
    fingersPickers.clear();
}

void ofEasyFingerCam::clearFingers()
{
    fingers.clear();
}