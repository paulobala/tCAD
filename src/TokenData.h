

#ifndef Carver_markerData_h
#define Carver_markerData_h

class TokenData
{
    int idToken;
    void updateMinDepth(float newdepth){
        if(newdepth < zmin)
        {
            zmin = newdepth;
        }
    }
public:
    int getID(){return idToken;}
    
    float zmin;
    float zmax;
    float zcurrent;
    ofPoint centroidBlob;
    
    TokenData(TuioObject * tuioObject,float averageDepth, ofPoint centroidBlob_){
        idToken = tuioObject->getSymbolID();
        centroidBlob = centroidBlob_;
        zcurrent = 0;
        zmin = 1000;
        zmax = 0;
        updateMaxDepth(averageDepth);
    }
    
    void updateCurrentDepth(float newDepth){
        zcurrent = newDepth;
        updateMinDepth(newDepth);
    }
    void updateMaxDepth(float newDepth){
        zmax = newDepth;
        updateCurrentDepth(newDepth);
    }
};

#endif
