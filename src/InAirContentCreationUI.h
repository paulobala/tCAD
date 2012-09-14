#ifndef tCAD_ThirdMenu_h
#define tCAD_ThirdMenu_h

#include "OnScreenOption.h"
#include "lockableVector.h"
#include "Shape3D.h"
#include "STLSubOptions.h"
#include "Token.h"
#include "OnTokenOption.h"
#include "InAirOption.h"
#include "ColorScheme.h"

/*
 In air Content Creation token
 */
class InAirContentCreationUI
{
public:
    enum OPTION
    {
        LEFT,RIGHT, UP, DOWN, CENTER, NOOPTION
    };//rectangles around and on token
    
    enum CURRENTMENU
    {
        STL, KINECT, COPY, CONTOUR, MAINMENU
    };//current mode
    
private:
    Token * tuioObject;  //token
    float radius;
    LockableVector<Shape3D* > * repo;//shapes in 3D Scene
    LockableVector<ContainerToken* > * containers;//container tokens on tabletop
    ofVec3f * entryPoint;//entry point for new shapes
    
    ofVec2f nw, ne, se, sw, center;//token edges
    
    //Options
    InAirOption * up;
    InAirOption * left;
    InAirOption * right;
    InAirOption * centerop;
    
    vector<OnScreenOption *> optionsSTL;
    OnScreenOption * optionSTL;
    
    vector<OnScreenOption *> optionsCopy;
    OnScreenOption * optionCopy;
    
    OPTION previousOption;
    OPTION currentOption;
    
    bool isOnTable;
    int mainMenuOptionNumber;
    bool visible;
    unsigned long lastUsed;
    ofTrueTypeFont font;
    CURRENTMENU currentMenu;
public:
    void setToken(Token * value)
    {
        tuioObject = value;
    }
    enum RETURNTYPE
    {
        RETURNKINECT,  RETURNSTL,  RETURNCOPY,  RETURNCONTOUR, RETURNSAVEKINECT, RETURNSAVECONTOUR, RETURNDISCARDKINECT, RETURNDISCARDCONTOUR , NORETURN
    };
    bool getIsOnTable()
    {
        return isOnTable;
    }
    void setIsOnTable(bool value)
    {
        isOnTable = value;
    }
    
    
    /*
     Constructor
     */
    InAirContentCreationUI(Token * to, ofVec3f * point, LockableVector<Shape3D* > * arepo, LockableVector<ContainerToken* > * acontainers)
    {
        entryPoint = point;
        tuioObject = to;
        repo = arepo;
        containers = acontainers;
        radius = 200;
        mainMenuOptionNumber = 0;
        currentMenu = MAINMENU;
        previousOption = NOOPTION;
        currentOption = NOOPTION;
        left = new InAirOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), tuioObject->getAngle());
        right = new InAirOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0),tuioObject->getAngle());
        up = new InAirOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0),tuioObject->getAngle());
        centerop = new InAirOption(ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0), ofVec2f(0,0),tuioObject->getAngle());
        isOnTable = true;
        font.loadFont("fonts/HelveticaNeue.ttf", 18);
        setup2(6);
        assignFunctions();
        lastUsed = ofGetElapsedTimeMillis();
        visible = true;
    }
    
    /*
     Update onscreen options
     */
    void updateOptions(float x, float y, float angle)
    {
        ofVec2f centerpoint = ofVec2f(x*ofGetWidth(), y*ofGetHeight());
        ofVec2f point1 = ofVec2f(x*ofGetWidth() +cos(angle)*70, y*ofGetHeight()+sin(angle)*70);
        point1.rotate(-45, centerpoint);
        ofVec2f point2 = ofVec2f(x*ofGetWidth()+cos(angle+PI/2)*70,
                                 y*ofGetHeight()+sin(angle+PI/2)*70);
        point2.rotate(-45, centerpoint);
        ofVec2f point3 =  ofVec2f(x*ofGetWidth() +cos(angle+PI)*70, y*ofGetHeight()+sin(angle+PI)*70);
        point3.rotate(-45, centerpoint);
        ofVec2f point4 = ofVec2f(x*ofGetWidth() +cos(angle+PI+PI/2)*70, y*ofGetHeight()+sin(angle+PI+PI/2)*70);
        point4.rotate(-45, centerpoint);
        ofVec2f point1l = point4;
        ofVec2f point2l = point1;
        point1l.rotate(90, point2);
        point2l.rotate(90, point2);
        ofVec2f point3r = point4;
        ofVec2f point4r = point1;
        point3r.rotate(-90, point3);
        point4r.rotate(-90, point3);
        ofVec2f point4u = point1;
        ofVec2f point1u = point2;
        point1u.rotate(-90, point4);
        point4u.rotate(-90, point4);
        ofVec2f point2d = point1;
        ofVec2f point3d = point2;
        point2d.rotate(90, point3);
        point3d.rotate(90, point3);
        left->update(point1, point1l, point2l, point2,angle);
        right->update(point4r, point4, point3, point3r,angle);
        up->update(point4u, point1u, point1, point4, angle);
        centerop->update(point4, point1, point2, point3,angle);
    }
    /*
     Options for STL mode
     */
    float setup2(int numOptions)
    {
        float center = (numOptions* 100)/2;
        float x1 = tuioObject->getX()*ofGetWidth() - center;
        float y1 = tuioObject->getY()*ofGetHeight() - 300;
        ofImage img;
        img.loadImage("images/cube.png");
        optionsSTL.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Cube",entryPoint, repo, "stls/cube.stl",ofColor(233,29,45),img));
        img.loadImage("images/cone.png");
        optionsSTL.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Cone",entryPoint, repo, "stls/cone.stl",ofColor(245,131,31),img));
        img.loadImage("images/cylinder.png");
        optionsSTL.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Cylinder",entryPoint, repo, "stls/cylinder.stl",ofColor(70,183,73),img));
        img.loadImage("images/pyramid.png");
        optionsSTL.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Pyramid",entryPoint, repo, "stls/pyramid.stl",ofColor(245,131,31),img));
        img.loadImage("images/sphere.png");
        optionsSTL.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Sphere",entryPoint, repo, "stls/sphere.stl",ofColor(117,206,219),img));
        img.loadImage("images/tube.png");
        optionsSTL.push_back(new STLSubOptions(ofVec2f(x1,y1),40, "Tube",entryPoint, repo, "stls/tube.stl",ofColor(0,159,215),img));
        return 2;
    }
    /*
     Options for copy mode
     */
    void setup3()
    {
        optionsCopy.clear();
        int numOptions = 0;
        vector<ContainerToken*> temp = containers->getObjects();
        for (std::vector<ContainerToken*>::iterator it=temp.begin() ; it < temp.end(); it++ )
        {
            if((*it)->links.size()>0)
            {
                numOptions ++;
            }
        }
        float center = (numOptions* 100)/2;
        float x1 = tuioObject->getX()*ofGetWidth() - center;
        float y1 = tuioObject->getY()*ofGetHeight() - 300;
        for (std::vector<ContainerToken*>::iterator it=temp.begin() ; it < temp.end(); it++ )
        {
            if((*it)->getIsOnTable())
            {
                if((*it)->links.size()>0)
                {
                    optionsCopy.push_back(new CopyContainerOption(ofVec2f(x1,y1),40, "Copy",entryPoint, repo, ofColor(233,29,45), (*it) ));
                }
            }
        }
    }
    /*
     Update token edges
     */
    void update()
    {
        if(isOnTable)
        {
            updateOptions(tuioObject->getX(), tuioObject->getY(), -tuioObject->getAngle());
        }
    }
    /*
     Draw onscreen options
     */
    void draw()
    {
        if(isOnTable)
        {
            if(visible)
            {
                if(ofGetElapsedTimeMillis() - lastUsed < 6000)
                {
                    centerop->draw();
                    left->draw();
                    right->draw();
                    up->draw();
                    ofVec2f point = (centerop->sw + centerop->se)/2;
                    ofPushMatrix();
                    ofTranslate(point.x, point.y);
                    ofRotate(ofRadToDeg(-tuioObject->getAngle()));
                    //draw information of remaing time of visibility
                    char fpsStr[255]; // an array of chars
                    sprintf(fpsStr, "Disappearing in %lu seconds.", (6000-(ofGetElapsedTimeMillis() - lastUsed)) /1000 );
                    ofSetColor(COLORSCHEME_TEXT_WHITE);
                    font.drawString(fpsStr, -80, 20);
                    ofPopMatrix();
                }
                else
                {
                    visible = false;
                    previousOption = NOOPTION;
                    currentOption = NOOPTION;
                }
            }
        }
    }
    /*
     Is point over token or surrounding areas
     */
    bool inside(float x, float y)
    {
        if(visible)
        {
            if(right->inside(x, y))
            {
                return true;
            }
            else if(left->inside(x, y))
            {
                return true;
            }
            else  if(centerop->inside(x, y))
            {
                return true;
            }
            else if(up->inside(x, y))
            {
                return true;
            }
        }
        else
        {
            if(centerop->inside(x, y))
            {
                visible = true;//make it visible
                lastUsed = ofGetElapsedTimeMillis();
                return false;
            }
        }
        return false;
    }
    /*
     Find in which area the point is
     */
    void updateOption(float x, float y)
    {
        previousOption = currentOption;
        if(right->inside(x, y))
        {
            currentOption = RIGHT;
        }
        else if(left->inside(x, y))
        {
            currentOption = LEFT;
        }
        else if(centerop->inside(x, y))
        {
            currentOption = CENTER;
        }
        else if(up->inside(x, y))
        {
            currentOption = UP;
        }
        else
        {
            currentOption = NOOPTION;
        }
    }
    
    /*
     Caroussel logic
     */
    void assignFunctions()
    {
        switch(currentMenu)
        {
            case STL:
            {
                if(optionsSTL.size() == 1)
                {
                    left->function = InAirOption::BACK;
                    right->function = InAirOption::BACK;
                }
                else if(optionSTL == (*optionsSTL.begin()))
                {
                    left->function = InAirOption::LEFTARROWSTL;
                    right->function = InAirOption::BACK;
                }
                else if(optionSTL == optionsSTL.back())
                {
                    left->function = InAirOption::BACK;
                    right->function = InAirOption::RIGHTARROWSTL;
                }
                else
                {
                    left->function = InAirOption::LEFTARROWSTL;
                    right->function = InAirOption::RIGHTARROWSTL;
                }
                up->function = InAirOption::STLENTER;
                up->name = optionSTL->name;
                up->color = optionSTL->color;
                STLSubOptions *d1 = dynamic_cast<STLSubOptions*>(optionSTL);
                if((bool)d1)
                {
                    up->shapeImg = d1->img;
                }
                break;
            }
            case KINECT:
            {
                left->function = InAirOption::KINECTDISCARD;
                right->function = InAirOption::KINECTSAVE;
                break;
            }
            case CONTOUR:
            {
                left->function = InAirOption::CONTOURDISCARD;
                right->function = InAirOption::CONTOURSAVE;
                break;
            }
            case COPY:
            {
                if(optionsCopy.size() == 0)
                {
                    left->function = InAirOption::BACK;
                    right->function = InAirOption::BACK;
                    up->function = InAirOption::BACK;
                }
                else if(optionsCopy.size() == 1)
                {
                    left->function = InAirOption::BACK;
                    right->function = InAirOption::BACK;
                    up->function = InAirOption::COPYENTER;
                    CopyContainerOption *d1 = dynamic_cast<CopyContainerOption*>(optionCopy);
                    if((bool)d1)
                    {
                        up->copyCont = d1;
                    }
                    else
                    {
                        up->copyCont = NULL;
                    }
                }
                else if(optionCopy == (*optionsCopy.begin()))
                {
                    left->function = InAirOption::LEFTARROWCOPY;
                    right->function = InAirOption::BACK;
                    up->function = InAirOption::COPYENTER;
                    CopyContainerOption *d1 = dynamic_cast<CopyContainerOption*>(optionCopy);
                    if((bool)d1)
                    {
                        up->copyCont = d1;
                    }
                    else
                    {
                        up->copyCont = NULL;
                    }
                }
                else if(optionCopy == optionsCopy.back())
                {
                    left->function = InAirOption::BACK;
                    right->function = InAirOption::RIGHTARROWCOPY;
                    up->function = InAirOption::COPYENTER;
                    CopyContainerOption *d1 = dynamic_cast<CopyContainerOption*>(optionCopy);
                    if((bool)d1)
                    {
                        up->copyCont = d1;
                    }
                    else
                    {
                        up->copyCont = NULL;
                    }
                }
                else
                {
                    left->function = InAirOption::LEFTARROWCOPY;
                    right->function = InAirOption::RIGHTARROWCOPY;
                    up->function = InAirOption::COPYENTER;
                    CopyContainerOption *d1 = dynamic_cast<CopyContainerOption*>(optionCopy);
                    if((bool)d1)
                    {
                        up->copyCont = d1;
                    }
                    else
                    {
                        up->copyCont = NULL;
                    }
                }
                break;
            }
            case MAINMENU:
            {
                left->function = InAirOption::LEFTARROWMAINMENU;
                right->function = InAirOption::RIGHTARROWMAINMENU;
                up->function = InAirOption::MAINMENUENTER;
                switch (mainMenuOptionNumber)
                {
                    case 0:
                    {
                        up->name = "STL";
                        break;
                    }
                    case 1:
                    {
                        up->name = "KINECT";
                        break;
                    }
                    case 2:
                    {
                        up->name = "CONTOUR";
                        break;
                    }
                    case 3:
                    {
                        up->name = "COPY";
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            default:
                ;
        }
    }
    
    /*
     Cause an action conforming movement direction (eg center square to right square
     */
    RETURNTYPE action(float x, float y)
    {
        updateOption(x,y);
        switch (previousOption)
        {
            case LEFT:
            {
                break;
            }
            case RIGHT:
            {
                break;
            }
            case UP:
            {
                break;
            }
            case DOWN:
            {
                break;
            }
            case CENTER:
            {
                switch (currentOption)
                {
                    case LEFT:
                    {
                        lastUsed = ofGetElapsedTimeMillis();
                        return leftHit();
                        
                        break;
                    }
                    case RIGHT:
                    {
                        lastUsed = ofGetElapsedTimeMillis();
                        return rightHit();
                       
                        break;
                    }
                    case UP:
                    {
                        lastUsed = ofGetElapsedTimeMillis();
                        return  upHit();
                       
                        break;
                    }
                    case DOWN:
                    {
                        break;
                    }
                    case CENTER:
                    {
                        lastUsed = ofGetElapsedTimeMillis();
                        break;
                    }
                    case NOOPTION:
                    {
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case NOOPTION:
            {
                break;
            }
            default:
                break;
        }
        return NORETURN;
    }
    
    /*
     trigger left area
     */
    RETURNTYPE leftHit()
    {
        left->action();
        hit(left->function);
    }
    /*
     trigger right area
     */
    RETURNTYPE rightHit()
    {
        right->action();
        hit(right->function);
    }
    /*
     trigger up area
     */
    RETURNTYPE upHit()
    {
        up->action();
        hit(up->function);
    }
    
    /*
     Caroussel logic
     */
    RETURNTYPE hit(InAirOption::FUNCTION fc)
    {
        switch(fc)
        {
            case InAirOption::LEFTARROWMAINMENU:
            {
                if(mainMenuOptionNumber >= 3)
                {
                    mainMenuOptionNumber = 0;
                }
                else
                {
                    mainMenuOptionNumber++;
                }
                assignFunctions();
                break;
            }
            case InAirOption::RIGHTARROWMAINMENU:
            {
                if(mainMenuOptionNumber <= 0)
                {
                    mainMenuOptionNumber = 3;
                }
                else
                {
                    mainMenuOptionNumber--;
                }
                assignFunctions();
                break;
            }
            case InAirOption::MAINMENUENTER:
            {
                switch (mainMenuOptionNumber)
                {
                    case 0:
                    {
                        optionSTL = (*optionsSTL.begin());
                        currentMenu = STL;
                        assignFunctions();
                        return  RETURNSTL;
                        break;
                    }
                    case 1:
                    {
                        currentMenu = KINECT;
                        assignFunctions();
                        return RETURNKINECT;
                        break;
                    }
                    case 2:
                    {
                        currentMenu = CONTOUR;
                        assignFunctions();
                        return RETURNCONTOUR;
                        break;
                    }
                    case 3:
                    {
                        setup3();
                        if(optionsCopy.size()>0)
                        {
                            optionCopy = (*optionsCopy.begin());
                        }
                        currentMenu = COPY;
                        assignFunctions();
                        return RETURNCOPY;
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case InAirOption::LEFTARROWSTL:
            {
                for(int i = 0; i < optionsSTL.size(); i++)
                {
                    if(optionSTL == optionsSTL.at(i))
                    {
                        optionSTL = optionsSTL.at(i+1);
                        break;
                    }
                }
                assignFunctions();
                break;
            }
            case InAirOption::RIGHTARROWSTL:
            {
                for(int i = 0; i < optionsSTL.size(); i++)
                {
                    if(optionSTL == optionsSTL.at(i))
                    {
                        optionSTL = optionsSTL.at(i-1);
                        break;
                    }
                }
                assignFunctions();
                break;
            }
            case InAirOption::STLENTER:
            {
                optionSTL->action();
                assignFunctions();
                break;
            }
            case  InAirOption::LEFTARROWCOPY:
            {
                for(int i = 0; i < optionsCopy.size(); i++)
                {
                    if(optionCopy== optionsCopy.at(i))
                    {
                        optionCopy = optionsCopy.at(i+1);
                        break;
                    }
                }
                assignFunctions();
                break;
            }
            case  InAirOption::RIGHTARROWCOPY:
            {
                for(int i = 0; i < optionsCopy.size(); i++)
                {
                    if(optionCopy == optionsCopy.at(i))
                    {
                        optionCopy = optionsCopy.at(i-1);
                        break;
                    }
                }
                assignFunctions();
                break;
            }
            case InAirOption::COPYENTER:
            {
                optionCopy->action();
                assignFunctions();
                break;
            }
            case InAirOption::KINECTSAVE:
            {
                currentMenu = MAINMENU;
                assignFunctions();
                return RETURNSAVEKINECT;
                break;
            }
            case  InAirOption::KINECTDISCARD:
            {
                currentMenu = MAINMENU;
                assignFunctions();
                return RETURNDISCARDKINECT;
                break;
            }
            case  InAirOption::CONTOURSAVE:
            {
                currentMenu = MAINMENU;
                assignFunctions();
                return RETURNSAVECONTOUR;
                break;
            }
            case  InAirOption::CONTOURDISCARD:
            {
                currentMenu = MAINMENU;
                assignFunctions();
                return RETURNDISCARDCONTOUR;
                break;
            }
            case  InAirOption::BACK:
            {
                currentMenu = MAINMENU;
                assignFunctions();
                break;
            }
            case  InAirOption::NOFUNCTION:
            {
                break;
            }
            default:
                break;
        }
    }
    
};

#endif
