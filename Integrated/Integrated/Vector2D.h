#ifndef _INCLUDED_VECTOR2D_H
#define _INCLUDED_VECTOR2D_H

#include <math.h>

class Vector2D
{
public:
    double x, y;

    Vector2D()
        : x(0.0f), y(0.0f)
    {
	}

    Vector2D(double argX, double argY)
        : x(argX), y(argY)
    {
    }

    Vector2D(const Vector2D &v)
        : x(v.x), y(v.y)
    {
    }

    inline Vector2D operator - () const {
        return Vector2D(-x, -y);
    }

	inline Vector2D & operator = (const Vector2D &v) {
        x = v.x;
        y = v.y;
        return *this;
    }

    inline Vector2D & operator += (const Vector2D &v) {
        x += v.x;
        y += v.y;
        return *this;
    }

    inline Vector2D & operator -= (const Vector2D &v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    inline Vector2D & operator *= (double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    inline Vector2D & operator /=  (double scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    inline double length () const {
        return sqrtf(x*x + y*y);
    }

    inline double lengthSq () const {
        return x*x + y*y;
    }

	inline double dot(const Vector2D &v) {
		return x * v.x + y * v.y;
	}

    inline Vector2D & normalize () {
        double length = this->length();
        x /= length;
        y /= length;
        return *this;
    }

    inline Vector2D getNormalize () const {
        Vector2D res(*this);
        res /= this->length();
        return res;
    }

    inline Vector2D & toNull () {
        x = 0.0f;
        y = 0.0f;
        return *this;
    }
};

inline Vector2D operator + (const Vector2D &v1, const Vector2D &v2)
{
    return Vector2D(v1.x + v2.x, v1.y + v2.y);
}
inline Vector2D operator - (const Vector2D &v1, const Vector2D &v2)
{
    return Vector2D(v1.x - v2.x, v1.y - v2.y);
}
inline Vector2D operator * (const Vector2D &v, double scalar)
{
    return Vector2D(v.x * scalar, v.y * scalar);
}
inline Vector2D operator * (double scalar, const Vector2D &v)
{
    return Vector2D(scalar * v.x, scalar * v.y);
}
inline Vector2D operator * (const Vector2D &v1, const Vector2D &v2)
{
    return Vector2D(v1.x * v2.x, v1.y * v2.y);
}
inline Vector2D operator / (const Vector2D &v, double scalar)
{
    return Vector2D(v.x / scalar, v.y / scalar);
}
inline Vector2D operator / (double scalar, const Vector2D &v)
{
    return Vector2D(scalar / v.x, scalar / v.y);
}
inline Vector2D operator / (const Vector2D &v1, const Vector2D &v2)
{
    return Vector2D(v1.x / v2.x, v1.y / v2.y);
}
inline double operator & (const Vector2D &v1, const Vector2D &v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}
inline double operator ^ (const Vector2D &v1, const Vector2D &v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}
inline Vector2D lerp(const Vector2D &v1, const Vector2D &v2, double t)
{
	return v1*(1.f - t) + v2*t;
}

#endif