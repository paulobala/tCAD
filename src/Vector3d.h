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
#ifndef _Vector3d_h
#define _Vector3d_h

#include <math.h>

class Vector3d{
public:
    union {
        struct {
            float    x;              // Vector X Component
            float    y;              // Vector Y Component
            float    z;              // Vector Z Component
        }; // End struct
        
        float value[3];
    }; // End union
    
    Vector3d(){x=y=z=0;};
    Vector3d(float _x, float _y, float _z){x= _x; y = _y; z= _z;};
    void SetX(float _x)	{ x = _x; }
	void SetY(float _y)	{ y = _y; }
	void SetZ(float _z)	{ z = _z; }
    
	float GetX()	{ return x; }
	float GetY()	{ return y; }
	float GetZ()	{ return z; }
    
    bool     operator== ( const Vector3d& vec ) const;
    bool     operator!= ( const Vector3d& vec ) const;
    
	Vector3d  operator+  ( const Vector3d& vec ) const;
    Vector3d  operator-  ( const Vector3d& vec ) const;
    
    Vector3d& operator+= ( const Vector3d& vec );
    Vector3d& operator-= ( const Vector3d& vec );
    
    Vector3d  operator* ( float value ) const;
    Vector3d  operator/ ( float value ) const;
    Vector3d  operator/ ( const Vector3d& vec ) const;
    
    Vector3d& operator*= ( float value );
    Vector3d& operator/= ( float value );
    
    Vector3d  operator+  () const;
    Vector3d  operator-  () const;
    
	Vector3d  operator* (const Vector3d& vec) const;
	Vector3d& operator*=(const Vector3d& vec);
    
    operator float*();
    operator const float*() const;
    
    void Set(float x, float y, float z);

    void		Clear();
    bool		IsEmpty() const;
    
    Vector3d		Maximize( const Vector3d& V1 ) const;
    Vector3d		Minimize( const Vector3d& V1 ) const;
    
	Vector3d&	Scale( float scale );
    
    Vector3d		Cross( const Vector3d& V1 ) const;
    float		Dot( const Vector3d& vec ) const;
    
    float		Length() const;
    float		SquareLength() const;
	void		SetLength(float l);
    
    bool		FuzzyCompare( const Vector3d& vecCompare,  const float& Tolerance) const;
    
    float		Normalize();
    
	bool Compare( const Vector3d &a, const float epsilon ) const;
    
    
    //------------------------------------------------------------
	// Public Friend Operators For This Class
	//------------------------------------------------------------
    friend Vector3d operator *(float Value, const Vector3d vec );
    
    
private:
};
#endif