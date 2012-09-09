#ifndef Carver_OperationContainerAlternative_h
#define Carver_OperationContainerAlternative_h

#include "Shape3D.h"
#include "AxisPlane.h"
#include "OperationAlternativeOption.h"
#include "CenterContainer.h"
#include "Composite3DObject.h"
#include "lineIntersection.h"

class OperationContainerAlternative {
    std::vector<Shape3D*> * links;
    LockableVector<Shape3D* > * shapes;
    
    AxisPlane * selectedPlane;
    ofImage translateImg, scaleImg, rotateLeftImg,rotateUpImg, minusImg, plusImg, rightscaleImg, leftscaleImg, leftArrowImg, rightArrowImg, unionImg, noBoolImg, symDifImg, diffABImg, diffBAImg, intersectImg;
    ofVec2f nw, ne, sw, se;
     void makePreviews(Shape3D* objA, Shape3D * obj);
    unsigned long leftArrowTime;
    unsigned long rightArrowTime;
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
    
    OPERATIONTYPE opType;
    
    OperationAlternativeOption * east;
    OperationAlternativeOption * west;
    OperationAlternativeOption * north;
    OperationAlternativeOption * south;
    OperationAlternativeOption * center;
    
    Composite3DObject * newComposite;
    
    OperationContainerAlternative(std::vector<Shape3D*> * links_,  AxisPlane * selectedPlane_,  LockableVector<Shape3D* > * shapes_);
    

    vector<BOOLEANOPERATION> possiblebooleans;
    BOOLEANOPERATION chosen;
  
    
    void update(float x, float y, float angle);
    bool inside(float x, float y, float z);
    bool insidedepth(float x, float y, float z);
    void draw();
    
    void action(float z, float y, float z);
    
    void eastHit(bool direction);
    void westHit(bool direction);
    void northHit(bool direction);
    void southHit(bool direction);
    
    void centerHit();
    
    void changedLinks();
    
    void changeAxis();
};

#endif
