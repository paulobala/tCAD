#ifndef tCAD_Limb_h
#define tCAD_Limb_h

/*
 Represents a limb.
 */
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
