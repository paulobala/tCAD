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
#include "Vector3d.h"


bool Vector3d::operator== ( const Vector3d& vec ) const
{
    return (x == vec.x) && (y == vec.y) && (z == vec.z);
}



bool Vector3d::operator!= ( const Vector3d& vec ) const
{
    return (x != vec.x) || (y != vec.y) || (z != vec.z);
}



Vector3d Vector3d::operator+( const Vector3d& vec ) const
{
    return Vector3d( x + vec.x, y + vec.y, z + vec.z );
}



Vector3d Vector3d::operator-( const Vector3d& vec ) const
{
    return Vector3d( x - vec.x, y - vec.y, z - vec.z );
}

// Type casting operator

Vector3d::operator float*()
{
    return value;
}

// Type casting operator

Vector3d::operator const float*() const
{
    return (const float*)&x;
}


// sign operator ( x = +vec)

Vector3d Vector3d::operator+() const
{
    return *this;
}


// sign operator ( x = -vec)

Vector3d Vector3d::operator-() const
{
    return Vector3d(-x, -y, -z);
}



Vector3d& Vector3d::operator+=( const Vector3d& vec )
{
    x += vec.x;
    y += vec.y;
    z += vec.z;
    return *this;
}



Vector3d& Vector3d::operator-=( const Vector3d& vec )
{
    x -= vec.x;
    y -= vec.y;
    z -= vec.z;
    return *this;
}



Vector3d& Vector3d::operator*=(float value)
{
    x *= value;
    y *= value;
    z *= value;
    return *this;
}



Vector3d& Vector3d::operator/=(float value)
{
    float inv_value = 1.0f / value;
    x *= inv_value;
    y *= inv_value;
    z *= inv_value;
    return *this;
}


Vector3d Vector3d::operator*( float value  ) const
{
    return Vector3d( x * value, y * value, z * value );
}



Vector3d Vector3d::operator*(const Vector3d& vec) const
{
    return Vector3d( x * vec.x, y * vec.y, z * vec.z );
}



Vector3d& Vector3d::operator*=(const Vector3d& vec)
{
    x *= vec.x;
    y *= vec.y;
    z *= vec.z;
    return *this;
}



Vector3d operator* (float Value, const Vector3d vec )
{
    return Vector3d( vec.x * Value, vec.y * Value, vec.z * Value );
}



Vector3d Vector3d::operator/ ( float Value ) const
{
    float fValue = 1.0f / Value;
    return Vector3d( x * fValue, y * fValue, z * fValue );
}


Vector3d Vector3d::operator/ ( const Vector3d& vec ) const
{
    return Vector3d( x / vec.x, y / vec.y, z /vec.z );
}


void Vector3d::Set(float px, float py, float pz)
{
    x = px;
    y = py;
    z = pz;
}



void Vector3d::Clear()
{
    x = y = z = 0;
}



bool Vector3d::IsEmpty() const
{
    return (x == 0.0f) && (y == 0.0f) && (z == 0.0f);
}



Vector3d Vector3d::Maximize( const Vector3d& vec ) const
{
    return Vector3d( (x > vec.x) ? x : vec.x, (y > vec.y) ? y : vec.y, (z > vec.z) ? z : vec.z );
}



Vector3d Vector3d::Minimize( const Vector3d& vec ) const
{
    return Vector3d( (x < vec.x) ? x : vec.x, (y < vec.y) ? y : vec.y, (z < vec.z) ? z : vec.z );
}



Vector3d& Vector3d::Scale( float scale )
{
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
}



Vector3d Vector3d::Cross( const Vector3d& vec ) const
{
    return Vector3d( y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y - y * vec.x );
}



float Vector3d::Dot( const Vector3d& vec ) const
{
    return x * vec.x + y * vec.y + z * vec.z;
}



float Vector3d::Length() const
{
    return sqrtf(x * x + y * y + z * z);
}



float Vector3d::SquareLength() const
{
    return x * x + y * y + z * z;
}


void Vector3d::SetLength(float l)
{
    float len = sqrt(x*x + y*y + z*z);
    x *= l/len;
    y *= l/len;
    z *= l/len;
}


bool Vector3d::FuzzyCompare( const Vector3d& vecCompare,  const float& Tolerance) const
{
    if ( fabsf(x - vecCompare.x) >= Tolerance ) return false;
    if ( fabsf(y - vecCompare.y) >= Tolerance ) return false;
    if ( fabsf(z - vecCompare.z) >= Tolerance ) return false;
    return true;
}



float Vector3d::Normalize()
{
    float   denom;
    denom = sqrtf(x * x + y * y + z * z);
    if (fabsf(denom) < 1e-5f)
        return denom;
    denom = 1.0f / denom;
    x *= denom;
    y *= denom;
    z *= denom;
    return denom;
}



bool Vector3d::Compare( const Vector3d &a, const float epsilon ) const
{
    if ( fabs( x - a.x ) > epsilon )
    {
        return false;
    }
    if ( fabs( y - a.y ) > epsilon )
    {
        return false;
    }
    if ( fabs( z - a.z ) > epsilon )
    {
        return false;
    }
    return true;
}
