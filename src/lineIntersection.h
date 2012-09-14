#ifndef tCAD_lineIntersection_h
#define tCAD_lineIntersection_h

/*
 2D Line used for intersection. Based on http://forum.openframeworks.cc/index.php?&topic=1055.0
 */
class LineSegment

{
    
public:
    
    ofVec2f begin_;
    
    ofVec2f end_;
    
    
    LineSegment(const ofVec2f& begin, const ofVec2f& end)
    
    : begin_(begin), end_(end) {}
    
    
    
    enum IntersectResult { PARALLEL, COINCIDENT, NOT_INTERESECTING, INTERESECTING };
    
    
    IntersectResult Intersect(const LineSegment& other_line, ofVec2f& intersection)
    {
        float denom = ((other_line.end_.y - other_line.begin_.y)*(end_.x - begin_.x)) -
        ((other_line.end_.x - other_line.begin_.x)*(end_.y - begin_.y));
        float nume_a = ((other_line.end_.x - other_line.begin_.x)*(begin_.y - other_line.begin_.y)) -
        ((other_line.end_.y - other_line.begin_.y)*(begin_.x - other_line.begin_.x));
        float nume_b = ((end_.x - begin_.x)*(begin_.y - other_line.begin_.y)) -
        ((end_.y - begin_.y)*(begin_.x - other_line.begin_.x));
        if(denom == 0.0f)
        {
            if(nume_a == 0.0f && nume_b == 0.0f)
            {
                return COINCIDENT;
            }
            return PARALLEL;
        }
        float ua = nume_a / denom;
        float ub = nume_b / denom;
        if(ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f)
        {
            // Get the intersection point.
            intersection.x = begin_.x + ua*(end_.x - begin_.x);
            intersection.y = begin_.y + ua*(end_.y - begin_.y);
            return INTERESECTING;
        }
        return NOT_INTERESECTING;
    }
    ;
    
};

#endif