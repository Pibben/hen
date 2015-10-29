/*
 * utils.h
 *
 *  Created on: Oct 6, 2014
 *      Author: per
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "veclib.h"

#include "CImg.h"

struct EmptyUniform {};

inline VecLib::Vector3f reflect(const VecLib::Vector3f& R, const VecLib::Vector3f& N) {
    VecLib::Vector3f Rout = R - 2.0f*N.dot(R)*N;

    return Rout;
}


inline VecLib::Matrix4f ortho(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    VecLib::Matrix4f m = VecLib::Matrix4f::Identity();
    m(0,0) = 2.0/(right - left);
    m(1,1) = 2.0/(top - bottom);
    m(2,2) = 2.0/(near - far);

    m(0,3) = (left+right)/(left-right);
    m(1,3) = (bottom+top)/(bottom-top);
    m(2,3) = (far+near)/(far-near);

    return m;
}

inline VecLib::Matrix4f proj(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    VecLib::Matrix4f m = VecLib::Matrix4f::Identity();
    m(0,0) = 2.0f*near/(right - left);
    m(1,1) = 2.0f*near/(top - bottom);
    m(2,2) = 2.0f/(near - far);

    m(0,2) = (right+left)/(right-left);
    m(1,2) = (top+bottom)/(top-bottom);
    m(2,2) = (near+far)/(near-far);
    m(3,2) = -1.0f;

    m(2,3) = 2*near*far / (near-far);

    return m;
}

inline VecLib::Matrix4f scaleBiasMatrix() {
    VecLib::Matrix4f scaleBias = VecLib::Matrix4f::Identity()*0.5f;
    //scaleBias.block<4,1>(0,3) += VecLib::Vector4f(1,1,1,1)*0.5; //TODO
    return scaleBias;
}

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T>
T normalize(T val) {
    val.normalize();
    return val;
}

inline VecLib::Matrix4f lookAt(const VecLib::Vector3f& eye, const VecLib::Vector3f& center, const VecLib::Vector3f& up) {
    const VecLib::Vector3f F = center - eye;
    const VecLib::Vector3f f = normalize(F);

    const VecLib::Vector3f UP = normalize(up);

    const VecLib::Vector3f s = f.cross(UP);
    const VecLib::Vector3f u = normalize(s).cross(f);

    VecLib::Matrix4f M = VecLib::Matrix4f::Identity();

    M(0, 0) = s[0];
    M(0, 1) = s[1];
    M(0, 2) = s[2];

    M(1, 0) = u[0];
    M(1, 1) = u[1];
    M(1, 2) = u[2];

    M(2, 0) = -f[0];
    M(2, 1) = -f[1];
    M(2, 2) = -f[2];

    //M.block<1,3>(0,0) = s;
   // M.block<1,3>(1,0) = u;
    //M.block<1,3>(2,0) = -f;

   // VecLib::Affine3f transform((VecLib::Translation3f(-eye)));

    VecLib::Matrix4f tm = VecLib::Matrix4f::Identity();

    tm(3, 0) = -eye[0];
    tm(3, 1) = -eye[1];
    tm(3, 2) = -eye[2];

    return M * tm;
}

inline cimg_library::CImg<unsigned char> normalizeDepth(cimg_library::CImg<float>& depth) {
    const int width = depth.width();
    const int height = depth.height();

    cimg_library::CImg<unsigned char> ret(width, height);

    for(int y = 0; y < height; ++y) {
        for(int x = 0; x < width; ++x) {
            const float dval = depth(x,y);
            if(dval == std::numeric_limits<float>::lowest()) {
                ret(x,y) = 0;
            } else {
                ret(x,y) = -(dval-10)*64;
            }
        }
    }

    return ret;
}

template <int enable>
struct Timer {
    void report(const std::string&) {}
    float getUS() {return 0;}
    void reset() {}
};

template <>
struct Timer<1> {
    decltype(std::chrono::steady_clock::now()) latest;
    Timer() {
        latest = std::chrono::steady_clock::now();
    }
    void report(const std::string& msg = "") {
        std::cout << msg << ": " << getUS() << " us" << std::endl;
    }

    float getUS() {
        auto now = std::chrono::steady_clock::now();
        auto diff = now - latest;
        latest = now;
        return std::chrono::duration <double, std::micro> (diff).count();
    }

    void reset() {
        latest = std::chrono::steady_clock::now();
    }
};


#endif /* UTILS_H_ */
