#ifndef QUATERNION_CUSTOM
#define QUATERNION_CUSTOM

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>

namespace customquat{

struct quaternion{

    union {
        float x;
        float r;
    };

    union {
        float y;
        float g;
    };

    union {
        float z;
        float b;
    };

    union {
        float w;
        float a;
    };

    quaternion(): x(0.0f),y(0.0f),z(0.0f),w(0.0f) {}

    quaternion(float _x, float _y, float _z, float _w = 0.0f)
    {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    }

    quaternion(const float* pFloat)
    {
        x = pFloat[0];
        y = pFloat[1];
        z = pFloat[2];
        w = pFloat[3];
    }


    quaternion(const glm::vec3& v , float _w = 0.0f)
    {
        x = v[0];
        y = v[1];
        z = v[2];
        w = _w;
    }

    quaternion(const glm::vec4& v)
    {
        x = v[0];
        y = v[1];
        z = v[2];
        w = v[3];
    }

    quaternion(float f)
    {
        x = y = z = w = f;
    }

    quaternion(float Angle, const glm::vec3& V){

        float HalfAngleInRadians = glm::radians(Angle/2);

        float SineHalfAngle = sinf(HalfAngleInRadians);
        float CosHalfAngle = cosf(HalfAngleInRadians);

        x = V.x * SineHalfAngle;
        y = V.y * SineHalfAngle;
        z = V.z * SineHalfAngle;
        w = CosHalfAngle;

    }

    quaternion& operator+=(const quaternion& r)
    {
        x += r.x;
        y += r.y;
        z += r.z;
        w += r.w;

        return *this;
    }

    quaternion& operator-=(const quaternion& r)
    {
        x -= r.x;
        y -= r.y;
        z -= r.z;
        w -= r.w;

        return *this;
    }

    quaternion& operator*=(float f)
    {
        x *= f;
        y *= f;
        z *= f;
        w *= f;

        return *this;
    }

    bool operator==(const quaternion& r)
    {
        return ((x == r.x) && (y == r.y) && (z == r.z) && (w == r.w));
    }

    bool operator!=(const quaternion& r)
    {
        return !(*this == r);
    }

    float length() const
    {
        float len = sqrtf(x * x + y * y + z * z + w * w);
        return len;
    }

    bool IsZero() const
    {
        return ((x + y + z + w) == 0.0f);
    }

    void normalize(){

        float Length =  sqrtf(x * x + y * y + z * z + w * w);
    
        x /= Length;
        y /= Length;
        z /= Length;
        w /= Length;
    };

    quaternion conjugate() const{
        quaternion ret(-x, -y, -z, w);
        return ret;
    }

    glm::vec3 ToDegrees(){
        
        return glm::vec3(
            glm::degrees(atan2(x * z + y * w, x * w - y * z)),
            glm::degrees(acos(-x * x - y * y - z * z - w * w)),
            glm::degrees(atan2(x * z - y * w, x * w + y * z))
        );
    }

    glm::vec3 ToVec3(){
        
        return glm::vec3(x,y,z);
    }




};





inline quaternion operator*(const quaternion& l, const quaternion& r){

    float w = (l.w * r.w) - (l.x * r.x) - (l.y * r.y) - (l.z * r.z);
    float x = (l.x * r.w) + (l.w * r.x) + (l.y * r.z) - (l.z * r.y);
    float y = (l.y * r.w) + (l.w * r.y) + (l.z * r.x) - (l.x * r.z);
    float z = (l.z * r.w) + (l.w * r.z) + (l.x * r.y) - (l.y * r.x);

    quaternion ret(x, y, z, w);

    return ret;
}

inline quaternion operator*(const quaternion& q, const glm::vec3& v){

    float w = - (q.x * v.x) - (q.y * v.y) - (q.z * v.z);
    float x =   (q.w * v.x) + (q.y * v.z) - (q.z * v.y);
    float y =   (q.w * v.y) + (q.z * v.x) - (q.x * v.z);
    float z =   (q.w * v.z) + (q.x * v.y) - (q.y * v.x);

    quaternion ret(x, y, z, w);

    return ret;
}

inline std::ostream& operator<<(std::ostream &out, const quaternion &q) {
    return out << q.x << ' ' << q.y << ' ' << q.z << ' ' << q.w;
}

inline quaternion operator+(const quaternion &q1, const quaternion &q2) {
    return quaternion(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w);
}

inline quaternion operator-(const quaternion &q1, const quaternion &q2) {
    return quaternion(q1.x - q2.x, q1.y - q2.y, q1.z - q2.z, q1.w - q2.w);
}

inline quaternion operator*(float t, const quaternion &q) {
    return quaternion(t*q.x, t*q.y, t*q.z, t*q.w);
}

inline quaternion operator*(const quaternion &q, float t) {
    return t * q;
}

inline quaternion operator/(const quaternion &q, double t) {
    return (1/t) * q;
}

inline quaternion unit_quaternion(quaternion q) {
    return q / q.length();
}

inline glm::vec3 rotation_quat(float Angle, const glm::vec3& v,const glm::vec3& target){

    quaternion rotation_q(Angle, v);

    quaternion conjugate_q = rotation_q.conjugate();

    return (rotation_q*target*conjugate_q).ToVec3();


}



}



#endif