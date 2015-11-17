//
// Created by per on 2015-10-20.
//

#ifndef HEN_VECLIB_H
#define HEN_VECLIB_H

namespace VecLib {
template <class T>
class Vector2 {
    T m[2];
public:
    Vector2() { }

    Vector2(T v1, T v2) {
        m[0] = v1;
        m[1] = v2;
    }

    const T &operator[](int idx) const {
        return m[idx];
    }

    T &operator[](int idx) {
        return m[idx];
    }

#define GEN_ACCESS1(name, idx1) \
    const T& name() const { \
        return m[idx1]; \
    } \
\
    T& name() { \
        return m[idx1]; \
    }

    GEN_ACCESS1(x, 0);
    GEN_ACCESS1(y, 1);
#undef GEN_ACESS2

    friend std::ostream& operator<<(std::ostream& os, const Vector2& v) {
        os << v.m[0] << " " << v.m[1];
        return os;
    }
};

template <class T>
Vector2<T> operator*(const Vector2<T>& v, T s) {
    return Vector2<T>(v[0]*s, v[1]*s);
}

template <class T>
Vector2<T> operator*(T s, const Vector2<T>& v) {
    return v*s;
}

template <class T>
Vector2<T> operator+(const Vector2<T>& v, T s) {
    return Vector2<T>(v[0]+s, v[1]+s);
}

template <class T>
Vector2<T> operator+(T s, const Vector2<T>& v) {
    return v+s;
}

template <class T>
Vector2<T> operator/(const Vector2<T>& v, T s) {
    return Vector2<T>(v[0]/s, v[1]/s);
}

template <class T>
Vector2<T> operator/(const Vector2<T>& v1, const Vector2<T>& v2) {
    return Vector2<T>(v1[0]/v2[0], v1[1]/v2[1]);
}

template <class T>
Vector2<T> operator+(const Vector2<T>& v1, const Vector2<T>& v2) {
    return Vector2<T>(v1[0]+v2[0], v1[1]+v2[1]);
}

template <class T>
Vector2<T> operator-(const Vector2<T>& v1, const Vector2<T>& v2) {
    return Vector2<T>(v1[0]-v2[0], v1[1]-v2[1]);
}

template <class T>
class Vector4;

template <class T>
class Vector3 {
    T m[3];
public:
    Vector3() {

    }

    Vector3(T v1, T v2, T v3) {
        m[0] = v1;
        m[1] = v2;
        m[2] = v3;
    }

    Vector3(const Vector4<T>& v4) : Vector3(v4[0], v4[1], v4[2]) {}

    void fill(T value) {
        std::fill(m, m+3, value);
    }

    const T& operator[](int idx) const {
        return m[idx];
    }

    T& operator[](int idx) {
        return m[idx];
    }

#define GEN_ACCESS2(name, idx1, idx2) \
    Vector2<T> name() const { \
        return Vector2<T>(m[idx1], m[idx2]); \
    }

    GEN_ACCESS2(xy, 0, 1);
#undef GEN_ACESS2

    T dot(const Vector3& other) const {
        return m[0] * other[0] + m[1] * other[1] + m[2] * other[2];
    }

    Vector3 cross(const Vector3& o) const {
        return Vector3(m[1]*o[2]-m[2]*o[1],
                       m[2]*o[0]-m[0]*o[2],
                       m[0]*o[1]-m[1]*o[0]);
    }

    Vector3 operator-() const {
        return Vector3(-m[0], -m[1], -m[2]);
    }

    T length() {
        return std::sqrt(dot(*this));
    }

    void normalize() {
        const T len = length();

        m[0] /= len;
        m[1] /= len;
        m[2] /= len;
    }

    void operator+=(const Vector3& other) {
        m[0] += other[0];
        m[1] += other[1];
        m[2] += other[2];
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        os << v.m[0] << " " << v.m[1] << " " << v.m[2];
        return os;
    }
};

template <class T>
Vector3<T> operator*(const Vector3<T>& v, T s) {
    return Vector3<T>(v[0]*s, v[1]*s, v[2]*s);
}

template <class T>
Vector3<T> operator*(T s, const Vector3<T>& v) {
    return v*s;
}

template <class T>
Vector3<T> operator+(const Vector3<T>& v1, const Vector3<T>& v2) {
    Vector3<T> retval(v1);
    retval += v2;
    return retval;
}

template <class T>
Vector3<T> operator-(const Vector3<T>& v1, const Vector3<T>& v2) {
    return Vector3<T>(v1[0]-v2[0], v1[1]-v2[1], v1[2]-v2[2]);
}

template <class T>
class Vector4 {
    T m[4];
public:
    Vector4() {}

    Vector4(T value) {
        m[0] = m[1] = m[2] = m[3] = value;
    }

    Vector4(const Vector2<T>& v, T v1, T v2) : Vector4(v[0], v[1], v1, v2) {}

    Vector4(T v1, T v2, T v3, T v4) {
        m[0] = v1;
        m[1] = v2;
        m[2] = v3;
        m[3] = v4;
    }

    Vector4(const Vector3<T>& vec3, T v) : Vector4(vec3[0], vec3[1], vec3[2], v) {}

    Vector4(const Vector3<T>& vec3) {
        //TODO:
        assert(0);
    }

    void fill(T value) {
        std::fill(m, m+4, value);
    }

    const Vector4& operator=(const Vector3<T>& vec3) {
        m[0] = vec3[0];
        m[1] = vec3[1];
        m[2] = vec3[2];

        return *this;
    }

    const T& operator[](int idx) const {
        return m[idx];
    }

    T& operator[](int idx) {
        return m[idx];
    }

#define GEN_ACCESS2(name, idx1, idx2) \
    Vector2<T> name() const { \
        return Vector2<T>(m[idx1], m[idx2]); \
    }

    GEN_ACCESS2(xy, 0, 1);
#undef GEN_ACESS2

#define GEN_ACCESS3(name, idx1, idx2, idx3) \
    Vector3<T> name() const { \
        return Vector3<T>(m[idx1], m[idx2], m[idx3]); \
    }

    GEN_ACCESS3(xyz, 0, 1, 2);
#undef GEN_ACESS3

    static Vector4 Ones() {
        return Vector4(1.0);
    }

    T dot(const Vector4& other) const {
        return m[0] * other[0] + m[1] * other[1] + m[2] * other[2] + m[3] * other[3];
    }

    Vector4 cwiseProduct(const Vector4& o) const {
        return Vector4(m[0]*o[0],
                       m[1]*o[1],
                       m[2]*o[2],
                       m[3]*o[3]);
    }

    void operator/=(T s) {
        m[0] /= s;
        m[1] /= s;
        m[2] /= s;
        m[3] /= s;
    }

    void operator*=(T s) {
        m[0] *= s;
        m[1] *= s;
        m[2] *= s;
        m[3] *= s;
    }

    Vector4 operator+(const Vector4& o) {
        return Vector4(m[0]+o[0], m[1]+o[1], m[2]+o[2], m[3]+o[3]);
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector4& v) {
        os << v.m[0] << " " << v.m[1] << " " << v.m[2] << " " << v.m[3];
        return os;
    }
};

template <class T>
Vector4<T> operator*(const Vector4<T>& v, T s) {
    return Vector4<T>(v[0]*s, v[1]*s, v[2]*s, v[3]*s);
}

template <class T>
Vector4<T> operator*(T s, const Vector4<T>& v) {
    return v*s;
}

template <class T>
Vector4<T> operator/(const Vector4<T>& v, T s) {
    Vector4<T> retval;
    retval /= s;
    return retval;
}

template <class T>
Vector4<T> operator/(const Vector4<T>& v1, const Vector4<T>& v2) {
    return Vector4<T>(v1[0]/v2[0], v1[1]/v2[1], v1[2]/v2[2], v1[3]/v2[3]);
}

template <class T>
Vector4<T> operator-(const Vector4<T>& v1, const Vector4<T>& v2) {
    return Vector4<T>(v1[0]-v2[0], v1[1]-v2[1], v1[2]-v2[2], v1[3]-v2[3]);
}



using Vector2f = Vector2<float>;
using Vector3f = Vector3<float>;
using Vector4f = Vector4<float>;

template <class T>
class Matrix4x4;

template <class T>
class Matrix3x3 {
    Vector3<T> m[3];

    Vector3<T> getRow(int i) const {
        return Vector3<T>(m[0][i], m[1][i], m[2][i]);
    }
public:
    explicit Matrix3x3(T val) { //TODO
        std::for_each(m, m+3, [val](Vector3<T>& v){
            v.fill(val);
        });
    }

    Matrix3x3(const Matrix4x4<T>& m4) {
        for(int i = 0; i < 3; ++i) {
            m[i] = m4[i];
        }
    }

    Vector3<T>& operator[](int idx) {
        return m[idx];
    }

    const Vector3<T>& operator[](int idx) const {
        return m[idx];
    }

    Matrix3x3 operator*(const Matrix3x3& o) const {
        Matrix3x3 r;

        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                r[i][j] = r[i].dot(getRow(j));
            }
        }

        return r;
    }

    Vector3<T> operator*(const Vector3<T>& o) const {
        return Vector3<T>(getRow(0).dot(o),
                          getRow(1).dot(o),
                          getRow(2).dot(o));
    }

    static Matrix3x3 Zeros() {
        Matrix3x3 retval(T(0));
        return retval;
    }

    static Matrix3x3 Identity() {
        Matrix3x3 retval = Zeros();
        retval[0][0] = T(1);
        retval[1][1] = T(1);
        retval[2][2] = T(1);

        return retval;
    }
};

template <class T>
class Matrix4x4 {
    Vector4<T> m[4];

    Vector4<T> getRow(int i) const {
        return Vector4<T>(m[0][i], m[1][i], m[2][i], m[3][i]);
    }

public:
    Matrix4x4 getScaled(T scale) const {
        Matrix4x4 r(*this);
        for(int i = 0; i < 4; ++i) {
            r[i] *= scale;
        }

        return r;
    }

    Matrix4x4() {

    }
    explicit Matrix4x4(T val) { //TODO
        std::for_each(m, m+4, [val](Vector4<T>& v){
            v.fill(val);
        });
    }

    Matrix4x4(const Matrix3x3<T>& m3) : Matrix4x4(T(0)) {
        for(int i = 0; i < 3; ++i) {
            m[i] = m3[i];
        }
        m[3][3] = T(1.0);
    }

    Vector4<T>& operator[](int idx) {
        return m[idx];
    }

    const Vector4<T>& operator[](int idx) const {
        return m[idx];
    }

    void operator*=(const Matrix4x4& o) {
        *this = *this * o;
    }

    Matrix4x4 operator*(const Matrix4x4& o) const {
        Matrix4x4 r;

        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                r[i][j] = getRow(j).dot(o[i]);
            }
        }

        return r;
    }

    Vector4<T> operator*(const Vector4<T>& o) const {
        return Vector4<T>(getRow(0).dot(o), getRow(1).dot(o), getRow(2).dot(o), getRow(3).dot(o));
    }

    static Matrix4x4 Zeros() {
        Matrix4x4 retval(T(0));
        return retval;
    }

    static Matrix4x4 Identity() {
        Matrix4x4 retval = Zeros();
        retval[0][0] = T(1);
        retval[1][1] = T(1);
        retval[2][2] = T(1);
        retval[3][3] = T(1);

        return retval;
    }

    friend std::ostream& operator<<(std::ostream& os, const Matrix4x4& m) {
        os << m.getRow(0) << std::endl << m.getRow(1) << std::endl << m.getRow(2) << std::endl << m.getRow(3);
        return os;
    }

};

template <class T>
Matrix4x4<T> operator*(const Matrix4x4<T>& v, T s) {
    return v.getScaled(s);
}

using Matrix3f = Matrix3x3<float>;
using Matrix4f = Matrix4x4<float>;
using Matrix3f = Matrix3x3<float>;
using Matrix4f = Matrix4x4<float>;
using Matrix3x3f = Matrix3x3<float>;
using Matrix4x4f = Matrix4x4<float>;

}; //ns



#endif //HEN_VECLIB_H
