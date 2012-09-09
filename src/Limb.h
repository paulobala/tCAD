//
//  Limb.h
//  Carver
//
//  Created by paulobala on 18/06/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef Carver_Limb_h
#define Carver_Limb_h

class Limb{
    cv::Point centerLimb;
    vector<cv::Point> approxCountour;
public:
    Limb(){
        centerLimb = cv::Point();
        approxCountour = vector<cv::Point>();
    }
    
    cv::Point getCenterLimb(){return centerLimb;}
    void setCenterLimb(cv::Point value){centerLimb =value;}
    
    vector<cv::Point> getApproxCountour(){return approxCountour;}
    void setApproxCountour(vector<cv::Point> value){ approxCountour =value;}
};

#endif
