//=============================================================================//
// This source file is part of the Open Knowledge 3D engine as written by:     //
// F r a n k   W i l l e m s e n                                               //
//                                                                             //
// You're free to use the code in any way you like, modified, unmodified or    //
// cut'n'pasted into your own work.                                            //
//                                                                             //
// The architecture of the engine is based on the Doom 3 SDK as released to    //
// the public by ID-software.                                                  //
//                                                                             //
// I would like to thank ID for the code and knowledge sharing. Without them   //
// the wheel for sure was invented by other people but there wouldn't be that  //
// many high quality bikes :-)                                                 //
//                                                                             //
// THIS IS NOT ID-SOFTWARE OR IN ANY WAY RELATED TO ID-SOFTWARE. DON'T ASK     //
// THEM QUESTIONS IN RELATION TO THIS SOURCE                                   //
//                                                                             //
// Although very little, some source code is taken directly out of the Doom 3  //
// SDK. Please be carefull when distributing or using commercially.            //
//                                                                             //
// The code contains lots of stuff of lots of people on the internet. Thanks   //
// to all those kind of projects are possible.                                 //
//                                                                             //
// Although coded with care I don't take responsibility for damage caused by   //
// this code in any way, including legal aspects.                              //
//                                                                             //
//=============================================================================//
//-----------------------------------------------------------------------------
// Specific includes required for this class
//-----------------------------------------------------------------------------

#include "plane.h"

//-----------------------------------------------------------------------------
// Desc : Plane member functions
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Name : Plane() (ALTERNATE CONSTRUCTOR)
// Desc : Constructor for this class.
//-----------------------------------------------------------------------------
Plane::Plane( const Vector3d& vecNormal, float fDistance )
{
    // Initialise anything we need
    normal  = vecNormal;
    d       = fDistance;
    epsilon = 0.001f;
    // Only needed if provided normal not is unit vector
    normal.Normalize();
}

//-----------------------------------------------------------------------------
// Name : Plane() (ALTERNATE CONSTRUCTOR)
// Desc : Constructor for this class.
//-----------------------------------------------------------------------------
Plane::Plane( const Vector3d& vecNormal, const Vector3d& vecPointOnPlane )
{
    // Initialise anything we need
    normal = vecNormal;
    epsilon = 0.001f;
    // Only needed if provided normal not is unit vector
    normal.Normalize();
    d = -vecPointOnPlane.Dot(normal);
}

// for clockwise
Plane::Plane( const Vector3d& p1, const Vector3d& p2, const Vector3d& p3 )
{
    Vector3d m = p2-p1;
    Vector3d n = p3-p1;
    epsilon = 0.001f;
    normal = m.Cross(n);
    normal.Normalize();
    // Initialise anything we need
    d = -p1.Dot(normal);
}


bool Plane::operator== ( const Plane& p ) const
{
    if( normal.x ==p.normal.x &&
       normal.y ==p.normal.y &&
       normal.z ==p.normal.z &&
       d == p.d)
        return true;
    return false;
}


bool Plane::operator!= ( const Plane& p ) const
{
    if( normal.x !=p.normal.x ||
       normal.y !=p.normal.y ||
       normal.z !=p.normal.z ||
       d != p.d)
        return true;
    return false;
}

float Plane::DistanceTo(const Vector3d& vecPoint) const
{
    return vecPoint.Dot(normal) + d;
}

bool Plane::IsFrontFacingTo(const Vector3d& direction) const
{
    double dot = normal.Dot(direction);
    return (dot <= 0);
}

//-----------------------------------------------------------------------------
// Name : ClassifyPoint ()
// Desc : Classifies the vector relative to the plane
//-----------------------------------------------------------------------------
CLASSIFY Plane::ClassifyPoint( const Vector3d& vecPoint, float * Dist ) const
{
    float result = vecPoint.Dot(normal) + d;
    if (Dist)
        *Dist = result;
    if ( result < -epsilon )
        return CP_BACK;
    if ( result > epsilon )
        return CP_FRONT;
    return CP_ONPLANE;
}


//-----------------------------------------------------------------------------
// Name : ClassifyPoly ()
// Desc : Classifies the vertices passed relative to the plane
// Note : Because this is a multi purpose function, we require information
//        about the size of each individual vertex in the array passed. This
//        is used to seek to the start of each vertex regardless of its size.
//-----------------------------------------------------------------------------
CLASSIFY Plane::ClassifyPoly( const Vector3d pVertices[], unsigned long VertexCount, unsigned long VertexStride ) const
{
    unsigned char   *pVerts     = (unsigned char*)pVertices;
    unsigned long   Infront     = 0;
    unsigned long   Behind      = 0;
    unsigned long   OnPlane     = 0;
    float           result      = 0;
    // Loop round each vector
    for ( unsigned long i = 0; i < VertexCount; i++, pVerts += VertexStride )
    {
        // Calculate distance
        result = (*(Vector3d*)pVerts).Dot( normal ) + d;
        // Check the position
        if (result > epsilon )
        {
            Infront++;
        }
        else if (result < -epsilon )
        {
            Behind++;
        }
        else
        {
            OnPlane++;
            Infront++;
            Behind++;
        } // End if Result tested
    } // End For Each vector
      // Return Result
    if ( OnPlane == VertexCount )
        return CP_ONPLANE;
    if ( Behind  == VertexCount )
        return CP_BACK;
    if ( Infront == VertexCount )
        return CP_FRONT;
    return CP_SPANNING;
}

//-----------------------------------------------------------------------------
// Name : ClassifyLine ()
// Desc : Classifies the line passed relative to the plane
//-----------------------------------------------------------------------------
CLASSIFY Plane::ClassifyLine( const Vector3d& Point1, const Vector3d& Point2 ) const
{
    int     Infront = 0, Behind = 0, OnPlane=0;
    float   result  = 0;
    // Calculate distance
    result = Point1.Dot( normal ) + d;
    // Check the position
    if (result > epsilon )
    {
        Infront++;
    }
    else if (result < -epsilon)
    {
        Behind++;
    }
    else
    {
        OnPlane++;
        Infront++;
        Behind++;
    } // End if Result tested
      // Calculate distance
    result = Point2.Dot( normal ) + d;
    // Check the position
    if (result > epsilon )
    {
        Infront++;
    }
    else if (result < - epsilon)
    {
        Behind++;
    }
    else
    {
        OnPlane++;
        Infront++;
        Behind++;
    } // End if Result tested
      // Check Results
    if ( OnPlane == 2 )
        return CP_ONPLANE;
    if ( Behind  == 2 )
        return CP_BACK;
    if ( Infront == 2 )
        return CP_FRONT;
    return CP_SPANNING;
}

//-----------------------------------------------------------------------------
// Name : GetRayIntersect ()
// Desc : Calculates the Intersection point between a ray and this plane
//-----------------------------------------------------------------------------
bool Plane::GetRayIntersect( const Vector3d& p0,  const Vector3d& p1, Vector3d& Intersection) const
{
    float dot1 = p0.Dot(normal) + d;
    float dot2 = p1.Dot(normal) + d;
    // Check special case. Normally due to a coding error
    // (prevent dividing by 0)
    if((dot1-dot2)==0)
    {
        // if dot1==0 (or dot2) then we are on the plane.
        // consider this an intersection and return p0
        if(dot1==0)
        {
            Intersection = p0;
            return true;
        }
        // Not 0. We are on the front or on the back side
        return false;
    }
    float dot = dot1 / (dot1-dot2);
    Intersection = p0 + dot*(p1-p0);
    // Success!
    return true;
}



bool Plane::Equals(Vector3d& _normal, float _dist, float normal_eps, float dist_eps)
{
    if( fabs(normal.x - _normal.x) < normal_eps &&
       fabs(normal.y - _normal.y) < normal_eps  &&
       fabs(normal.z - _normal.z) < normal_eps  &&
       fabs(d - _dist) < dist_eps)
        return true;
    return false;
}


Plane Plane::operator-() const
{
    return Plane(-normal, -d);
}


void Plane::FitThroughPoint(const Vector3d& p)
{
    d = -p.Dot(normal);
}