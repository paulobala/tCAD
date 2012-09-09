#ifndef Carver_OperationContainer_h
#define Carver_OperationContainer_h
#include "Shape3D.h"
#include "AxisPlane.h"
#include "EastContainer.h"
#include "WestContainer.h"
#include "NorthContainer.h"
#include "SouthContainer.h"
#include "CenterContainer.h"
#include "Composite3DObject.h"
#include "lineIntersection.h"


class OperationContainer {
    std::vector<Shape3D*> * links;
    LockableVector<Shape3D* > * shapes;
    
    AxisPlane * selectedPlane;  
    ofImage translateImg, scaleImg, rotateImg, minusImg, plusImg, rightscaleImg, leftscaleImg, leftArrowImg, rightArrowImg, unionImg, noBoolImg, symDifImg, diffABImg, diffBAImg, intersectImg, scaledown, scaleup,rotateleft,rotateright,rotatedown,rotateup,godown,goup;
    ofVec2f nw, ne, sw, se;
    void makePreviews(Shape3D* objA, Shape3D * obj);
    
    unsigned long leftArrowTime;
    unsigned long rightArrowTime;
  
    ofTrueTypeFont font;
public:
    
    enum OPERATIONTYPE{
        SCALE, TRANSLATE, ROTATE, BOOLEANS, NOOPERATION
    };
    
    enum BOOLEANOPERATION {
		BOOLEAN_INTERSECTION, 
		BOOLEAN_UNION, 
		BOOLEAN_DIFFERENCE_A_MINUS_B,
        BOOLEAN_DIFFERENCE_B_MINUS_A,
        BOOLEAN_SYMMETRIC_DIFFERENCE,
        NOBOOLEAN
	};
    vector<BOOLEANOPERATION> possibleBooleans;
    BOOLEANOPERATION chosen;
    OPERATIONTYPE opType;
    
    EastContainer * east;
    WestContainer * west;
    NorthContainer * north;
    SouthContainer * south;
    CenterContainer * center;
    
    Composite3DObject * newComposite;
    
        
    OperationContainer(std::vector<Shape3D*> * links_,  AxisPlane * selectedPlane_, LockableVector<Shape3D*> * shapes_);
    
    void update(float x, float y, float angle);
    bool inside(float x, float y, float z);
    bool insideDepth(float x, float y, float z);
    
    void draw();
    
    void action(float z, float y, float z);
    
    void eastHit();
    void westHit();
    void northHit();
    void southHit();
    void centerHit();
    
    void changedLinks();
    void changeAxis();
};

#endif
