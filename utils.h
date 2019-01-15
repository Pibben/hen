/*
 * utils.h
 *
 *  Created on: Oct 6, 2014
 *      Author: per
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "veclib.h"

struct EmptyUniform {};

inline VecLib::Vector3f reflect(const VecLib::Vector3f& R, const VecLib::Vector3f& N) {
    VecLib::Vector3f Rout = R - 2.0f*N.dot(R)*N;

    return Rout;
}

inline VecLib::Matrix3f rotateY(float radians) {
    VecLib::Matrix3f m = VecLib::Matrix3f::Identity();
    float sinTheta = std::sin(radians); //TODO: sincos
    float cosTheta = std::cos(radians);
    m[0][0] = cosTheta;
    m[0][2] = sinTheta;
    m[2][0] = -sinTheta;
    m[2][2] = cosTheta;

    return m;
}


//TODO: Verify
inline VecLib::Matrix4f ortho(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    VecLib::Matrix4f m = VecLib::Matrix4f::Identity();
    m[0][0] = 2.0f/(right - left);
    m[1][1] = 2.0f/(top - bottom);
    m[2][2] = 2.0f/(near - far);

    m[3][0] = (left+right)/(left-right);
    m[3][1] = (bottom+top)/(bottom-top);
    m[3][2] = (far+near)/(far-near);

    return m;
}

inline VecLib::Matrix4f proj(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    VecLib::Matrix4f m = VecLib::Matrix4f::Identity();
    m[0][0] = 2.0f*near/(right - left);
    m[1][1] = 2.0f*near/(top - bottom);
    m[2][2] = 2.0f/(near - far);

    m[2][0] = (right+left)/(right-left);
    m[2][1] = (top+bottom)/(top-bottom);
    m[2][2] = (near+far)/(near-far);
    m[2][3] = -1.0f;

    m[3][2] = 2*near*far / (near-far);

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

std::vector<std::tuple<VecLib::Vector2f>> unitQuad() {
    VecLib::Vector2f v1(-1.0f, -1.0f);
    VecLib::Vector2f v2(-1.0f, 1.0f);
    VecLib::Vector2f v3(1.0f, 1.0f);
    VecLib::Vector2f v4(1.0f, -1.0f);



    std::vector<std::tuple<VecLib::Vector2f>> mesh { std::make_tuple(v1), std::make_tuple(v3), std::make_tuple(v2),
                                                     std::make_tuple(v1), std::make_tuple(v4), std::make_tuple(v3) };

    return mesh;
}

inline VecLib::Matrix4f lookAt(const VecLib::Vector3f& eye, const VecLib::Vector3f& center, const VecLib::Vector3f& up) {
    const VecLib::Vector3f F = center - eye;
    const VecLib::Vector3f f = normalize(F);

    const VecLib::Vector3f UP = normalize(up);

    const VecLib::Vector3f s = f.cross(UP);
    const VecLib::Vector3f u = normalize(s).cross(f);

    VecLib::Matrix4f M = VecLib::Matrix4f::Identity();

    M[0][0] = s[0];
    M[1][0] = s[1];
    M[2][0] = s[2];

    M[0][1] = u[0];
    M[1][1] = u[1];
    M[2][1] = u[2];

    M[0][2] = -f[0];
    M[1][2] = -f[1];
    M[2][2] = -f[2];

    //M.block<1,3>(0,0) = s;
   // M.block<1,3>(1,0) = u;
    //M.block<1,3>(2,0) = -f;

   // VecLib::Affine3f transform((VecLib::Translation3f(-eye)));

    VecLib::Matrix4f tm = VecLib::Matrix4f::Identity();

    tm[3][0] = -eye[0];
    tm[3][1] = -eye[1];
    tm[3][2] = -eye[2];

    return M * tm;
}
#if 0
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
#endif
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

template<class T, uint16_t WIDTH, uint16_t HEIGHT, uint16_t DEPTH = 1>
class PixelBuffer {
private:
    T* mData;
public:
    PixelBuffer() : mData(new T[WIDTH*HEIGHT*DEPTH]) {}
    ~PixelBuffer() { delete[] mData; }
    PixelBuffer(const PixelBuffer&) = delete;
    T& operator()(uint_fast16_t x, uint_fast16_t y, uint_fast16_t z = 0) {
        return mData[(x + (HEIGHT - y) * WIDTH) * DEPTH + z];
    }
    T* data() { return mData; }
    constexpr uint16_t width() { return WIDTH; }
    constexpr uint16_t height() { return HEIGHT; }
    void fill(T value) { std::fill(mData, mData+WIDTH*HEIGHT*DEPTH, value); };
};


#endif /* UTILS_H_ */
