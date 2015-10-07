/*
 * utils.h
 *
 *  Created on: Oct 6, 2014
 *      Author: per
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <Eigen/Dense>

#include "CImg.h"

struct EmptyUniform {};

inline Eigen::Vector3f reflect(const Eigen::Vector3f& R, const Eigen::Vector3f& N) {
    Eigen::Vector3f Rout = R - 2.0*N.dot(R)*N;

    return Rout;
}


inline Eigen::Matrix4f ortho(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m(0,0) = 2.0/(right - left);
    m(1,1) = 2.0/(top - bottom);
    m(2,2) = 2.0/(near - far);

    m(0,3) = (left+right)/(left-right);
    m(1,3) = (bottom+top)/(bottom-top);
    m(2,3) = (far+near)/(far-near);

    return m;
}

inline Eigen::Matrix4f proj(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
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

inline Eigen::Matrix4f scaleBiasMatrix() {
    Eigen::Matrix4f scaleBias = Eigen::Matrix4f::Identity()*0.5f;
    scaleBias.block<4,1>(0,3) += Eigen::Vector4f(1,1,1,1)*0.5;
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

inline Eigen::Matrix4f lookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up) {
    const Eigen::Vector3f F = center - eye;
    const Eigen::Vector3f f = normalize(F);

    const Eigen::Vector3f UP = normalize(up);

    const Eigen::Vector3f s = f.cross(UP);
    const Eigen::Vector3f u = normalize(s).cross(f);

    Eigen::Matrix4f M = Eigen::Matrix4f::Identity();

    M.block<1,3>(0,0) = s;
    M.block<1,3>(1,0) = u;
    M.block<1,3>(2,0) = -f;

    Eigen::Affine3f transform((Eigen::Translation3f(-eye)));

    return M * transform.matrix();
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
