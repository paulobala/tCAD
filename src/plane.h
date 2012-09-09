//=============================================================================//
// This source file is part of the Open Knowledge 3D engine	as written by:     //
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
// SDK. Please be carefull when distributing or using commercially.	           //		
//                                                                             //
// The code contains lots of stuff of lots of people on the internet. Thanks   //
// to all those kind of projects are possible.                                 //
//                                                                             //
// Although coded with care I don't take responsibility for damage caused by   //
// this code in any way, including legal aspects.                              // 
//                                                                             //
//=============================================================================//
#ifndef __PLANE_H__
#define __PLANE_H__

//-----------------------------------------------------------------------------
// Class specific includes
//-----------------------------------------------------------------------------
#include <stdlib.h>
#include "math.h"
#include <vector>
#include "Vector3d.h"
//#include "mathematics.h"

//-----------------------------------------------------------------------------
// Miscellaneous Defines
//-----------------------------------------------------------------------------
#define	SIDE_FRONT					0
#define	SIDE_BACK                       1
#define	SIDE_ON						2
#define	SIDE_CROSS					3

// plane sides
#define PLANESIDE_FRONT				0
#define PLANESIDE_BACK                   1
#define PLANESIDE_ON                     2
#define PLANESIDE_CROSS				3

//-----------------------------------------------------------------------------
// Typedefs, structures & Enumerators 
//-----------------------------------------------------------------------------
typedef enum _CLASSIFY {    // Plane / Poly Classification
    CP_FRONT    = 1,        // Is in front
    CP_BACK     = 2,        // Is behind
    CP_ONPLANE  = 3,        // On the plane
    CP_SPANNING = 4         // Spans the plane
} CLASSIFY;

//-----------------------------------------------------------------------------
// Main class definitions
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Name : Plane (Class)
// Desc : 3D Plane class, handles all plane classification / calculations
//-----------------------------------------------------------------------------
class Plane
{
public:
    //------------------------------------------------------------
	// Public Variables for This Class
	//------------------------------------------------------------
    Vector3d	normal;			// Plane Normal
    float	d;				// Plane Distance
    float epsilon ;
    
public:
    //------------------------------------------------------------
	// Constructors / Destructors for this Class
	//------------------------------------------------------------
	Plane() { d=0; epsilon = 0.001f;};
	~Plane() {};
	Plane(const Plane& copy);
	Plane& operator=(const Plane& copy);
    
	Plane( const Vector3d& vecNormal, float fDistance );
	Plane( const Vector3d& vecNormal, const Vector3d& vecPointOnPlane );
	Plane( const Vector3d& p1, const Vector3d& p2, const Vector3d& p3 );
    
	bool operator== ( const Plane& p ) const;
	bool operator!= ( const Plane& p ) const;
	
	bool Equals(Vector3d& _normal, float _d, float normal_eps, float dist_eps);
    
	void SetNormal(Vector3d& n)	{ normal = n; }
	void SetDist(float dist)	{ d = dist; }
    
	void Normalize();
    
	void FitThroughPoint(const Vector3d& p);
    
	//------------------------------------------------------------
	// Public Functions for This Class
	//------------------------------------------------------------
	CLASSIFY	ClassifyPoly( const Vector3d pVertices[], unsigned long VertexCount, unsigned long VertexStride ) const;
	CLASSIFY	ClassifyPoint( const Vector3d& vecPoint, float * Dist = NULL ) const;
	CLASSIFY	ClassifyLine( const Vector3d& Point1, const Vector3d& Point2 ) const;
    
	int Side( const Vector3d &v, const float epsilon ) const;
    
	float	DistanceTo(const Vector3d& vecPoint) const;
	bool		IsFrontFacingTo(const Vector3d& direction) const;
    
	bool GetRayIntersect( const Vector3d& p0,  const Vector3d& p1, Vector3d& Intersection) const;
    
	Plane operator-() const;
    
	bool Compare( const Plane &p, const float normalEps, const float distEps ) const;
    
	void Clear() { normal.Clear(); d = 0; }
};



inline 
Plane::Plane(const Plane& copy)
{
	*this = copy;
}


inline
Plane& Plane::operator=(const Plane& copy)
{
	normal	= copy.normal;
	d		= copy.d;
    
	return *this;
}


inline 
int Plane::Side( const Vector3d &v, const float epsilon ) const 
{
	float dist = DistanceTo( v );
    
	if ( dist > epsilon ) {
		return CP_FRONT;
	}
	else if ( dist < -epsilon ) {
		return CP_BACK;
	}
	else {
		return CP_ONPLANE;
	}
}


inline 
bool Plane::Compare( const Plane &p, const float normalEps, const float distEps ) const 
{
	if ( fabs( d - p.d ) > distEps ) 
	{
		return false;
	}
	if ( !normal.Compare( p.normal, normalEps ) ) 
	{
		return false;
	}
	return true;
}


inline
void Plane::Normalize()
{
	// Here we calculate the magnitude of the normal to the plane (point A B C)
	// To calculate magnitude you use the equation:  magnitude = sqrt( x^2 + y^2 + z^2)
	float mag = (float)sqrt(normal.x * normal.x + 
						   	normal.y * normal.y + 
							normal.z * normal.z );
    
	// Then we divide the plane's values by it's magnitude.
	// This makes it easier to work with.
	normal.x	/= mag;
	normal.y	/= mag;
	normal.z	/= mag;
	d			/= mag;	
}

#endif // __PLANE_H__