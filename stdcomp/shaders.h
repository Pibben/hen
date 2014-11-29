/*
 * shaders.h
 *
 *  Created on: Oct 30, 2014
 *      Author: per
 */

#ifndef SHADERS_H_
#define SHADERS_H_

#include <Eigen/Dense>

#include "../utils.h"

template <class In, class InTraits, class Uniform>
class ColorVertexShader {
private:
public:
    Uniform& mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int COLOR_ATTACHMENT = 1;
    };

    ColorVertexShader(Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector4f& color = std::get<InTraits::COLOR_ATTACHMENT>(in);

        Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos, color);
    }

    Uniform& uniform() { return mUniform; }
};

template <class In, class InTraits, class Uniform>
class ColorFragmentShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    ColorFragmentShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector4f& color = std::get<InTraits::COLOR_ATTACHMENT>(in);
        //assert(pos[2] < 0.0);

        return std::make_tuple(color, pos[2]);
    }
};

template <class In, class InTraits, class Uniform>
class TextureVertexShader {
private:
public:
    Uniform& mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, float> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int TEXTURE_ATTACHMENT = 1;
        static constexpr int INV_DEPTH_ATTACHMENT = 2;
    };

    TextureVertexShader(Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector2f tex = std::get<InTraits::TEXTURE_ATTACHMENT>(in);

        const Eigen::Vector4f mvPos = mUniform.modelViewMatrix * pos;
        const Eigen::Vector4f outPos = mUniform.projMatrix * mvPos;

        const float invDepth = 1.0 / mvPos[2];

        return std::make_tuple(outPos, tex * invDepth, invDepth);
    }

    Uniform& uniform() { return mUniform; }
};

template <class In, class InTraits, class Uniform>
class TextureFragmentShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    TextureFragmentShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector2f& tex = std::get<InTraits::TEXTURE_ATTACHMENT>(in);
        const float invDepth = std::get<InTraits::INV_DEPTH_ATTACHMENT>(in);

        const auto realTex = tex / invDepth;

        auto color = mUniform.textureSampler.get(realTex[0], realTex[1]);

        return std::make_tuple(color, pos[2]);
    }
};

template <class In, class InTraits, class Uniform>
class MultiTextureFragmentShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    MultiTextureFragmentShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector2f& tex = std::get<InTraits::TEXTURE_ATTACHMENT>(in);

        auto color = mUniform.textureSampler.get(tex[0], tex[1]);
        auto ao = mUniform.aoSampler.get(tex[0], tex[1]);

        color = color.cwiseProduct(ao) / 255.0;

        return std::make_tuple(color, pos[2]);
    }
};

template <class In, class InTraits, class Uniform>
class FlatVertexShader {
private:
public:
    Uniform& mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int COLOR_ATTACHMENT = 1;
    };

    FlatVertexShader(Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos    = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector3f& normal = std::get<InTraits::NORMAL_ATTACHMENT>(in);

        const Eigen::Vector3f& lightPos = mUniform.lightPos;

        const Eigen::Vector4f P = mUniform.modelViewMatrix * pos;

        Eigen::Vector3f N = mUniform.modelViewMatrix.template topLeftCorner<3,3>() * normal;
        Eigen::Vector3f L = lightPos - P.topRows<3>();
        Eigen::Vector3f V = -P.topRows<3>();

        N.normalize();
        L.normalize();
        V.normalize();

        const auto R = reflect(-L, N);

        const float intensity = std::max(0.0f, R.dot(V));

        const Eigen::Vector4f color(255,255,255,255);

        const Eigen::Vector4f outPos = mUniform.projMatrix * P;
        return std::make_tuple(outPos, color*intensity);
    }

    Uniform& uniform() { return mUniform; }
};

template <class In, class InTraits, class Uniform>
class PhongVertexShader {
private:
public:
    Uniform& mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int NORMAL_ATTACHMENT = 1;
        static constexpr int LIGHT_ATTACHMENT = 2;
        static constexpr int VIEW_ATTACHMENT = 3;
    };

    PhongVertexShader(Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos    = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector3f& normal = std::get<InTraits::NORMAL_ATTACHMENT>(in);

        const Eigen::Vector3f& lightPos = mUniform.lightPos;

        const Eigen::Vector4f P = mUniform.modelViewMatrix * pos;

        const Eigen::Vector3f N = mUniform.modelViewMatrix.template topLeftCorner<3,3>() * normal;
        const Eigen::Vector3f L = lightPos - P.topRows<3>();
        const Eigen::Vector3f V = -P.topRows<3>();

        const Eigen::Vector4f outPos = mUniform.projMatrix * P;
        return std::make_tuple(outPos, N, L, V);
    }

    Uniform& uniform() { return mUniform; }
};

template <class In, class InTraits, class Uniform>
class PhongFragmentShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    PhongFragmentShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        Eigen::Vector3f N     = std::get<InTraits::NORMAL_ATTACHMENT>(in);
        Eigen::Vector3f L     = std::get<InTraits::LIGHT_ATTACHMENT>(in);
        Eigen::Vector3f V     = std::get<InTraits::VIEW_ATTACHMENT>(in);
        //assert(pos[2] < 0.0);

        N.normalize();
        L.normalize();
        V.normalize();

        const auto R = reflect(-L, N);

        const float intensity = std::max(0.0f, R.dot(V));

        const Eigen::Vector4f color(255,255,255,255);

        return std::make_tuple(color*intensity, pos[2]);
    }
};

template <class In, class InTraits, class Uniform>
class NormalViewVertexShader {
private:
public:
    Uniform& mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int NORMAL_ATTACHMENT = 1;
        static constexpr int VIEW_ATTACHMENT = 2;
    };

    NormalViewVertexShader(Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos    = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector3f& normal = std::get<InTraits::NORMAL_ATTACHMENT>(in);

        const Eigen::Vector4f P = mUniform.modelViewMatrix * pos;

        const Eigen::Vector3f N = mUniform.modelViewMatrix.template topLeftCorner<3,3>() * normal;
        const Eigen::Vector3f V = P.topRows<3>();

        const Eigen::Vector4f outPos = mUniform.projMatrix * P;
        return std::make_tuple(outPos, N, V);
    }

    Uniform& uniform() { return mUniform; }
};

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T>
T normalize(T val) {
    val.normalize();
    return val;
}


template <class In, class InTraits, class Uniform>
class EquiRectFragmentShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    EquiRectFragmentShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        Eigen::Vector3f N     = std::get<InTraits::NORMAL_ATTACHMENT>(in);
        Eigen::Vector3f V     = std::get<InTraits::VIEW_ATTACHMENT>(in);
        //assert(pos[2] < 0.0);

        N.normalize();
        V.normalize();

        auto R = reflect(V, N);

        R.normalize();

        Eigen::Vector2f tex;

        tex[1] = R[1]; // -1 .. 1
        R[1] = 0.0;
        tex[0] = normalize(R)[2] * 0.25; // -0.25 .. 0.25

        tex[1] = (tex[1] + 1.0) * 0.5; // 0 .. 1

        if(R[0] >= 0) {
            tex[0] = 0.25 + tex[0]; // 0 .. 0.5
        } else {
            tex[0] = 0.75 - tex[0]; // 0.5 .. 1.0
        }

        auto color = mUniform.textureSampler.get(tex[0], tex[1]);

        return std::make_tuple(color, pos[2]);
    }
};

#endif /* SHADERS_H_ */
