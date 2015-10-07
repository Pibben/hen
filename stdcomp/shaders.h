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
#include "samplers.h"

class ColorVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    };

    Uniform mUniform;

public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> InType;
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector4f& color = std::get<static_cast<int>(InTraits::COLOR_INDEX)>(in);

        Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos, color);
    }

    Eigen::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    Eigen::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class ColorFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> InType;
    typedef std::tuple<Eigen::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector4f& color = std::get<static_cast<int>(InTraits::COLOR_INDEX)>(in);
        //assert(pos[2] < 0.0);

        return std::make_tuple(color, pos[2]);
    }
};

class TextureVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    };
    Uniform mUniform;

public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> InType;
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, float> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2
    };

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector2f tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);

        const Eigen::Vector4f mvPos = mUniform.modelViewMatrix * pos;
        const Eigen::Vector4f outPos = mUniform.projMatrix * mvPos;

        const float invDepth = 1.0 / mvPos[2];

        return std::make_tuple(outPos, tex * invDepth, invDepth);
    }

    Eigen::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    Eigen::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class TextureFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2
    };

    typedef RGBATextureSampler<Eigen::Vector4f> SamplerType;

    struct Uniform {
        SamplerType textureSampler;
    };

    Uniform mUniform;
public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, float> InType;
    typedef std::tuple<Eigen::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    TextureFragmentShader(const std::string& filename) {
        cimg_library::CImg<unsigned char> texImg(filename.c_str());

        mUniform = {SamplerType(texImg)};
    }

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const float invDepth = std::get<static_cast<int>(InTraits::INV_DEPTH_INDEX)>(in);

        const auto realTex = tex / invDepth;

        auto color = mUniform.textureSampler.get(realTex[0], realTex[1]);

        return std::make_tuple(color, pos[2]);
    }
};

class MultiTextureFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2
    };

    typedef RGBATextureSampler<Eigen::Vector4f> SamplerType;

    struct Uniform {
        SamplerType textureSampler;
        SamplerType aoSampler;
    };

    Uniform mUniform;
public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, float> InType;
    typedef std::tuple<Eigen::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    MultiTextureFragmentShader(const std::string& filename1, const std::string& filename2) {
        cimg_library::CImg<unsigned char> texImg(filename1.c_str());
        cimg_library::CImg<unsigned char> aoImg(filename2.c_str());

        mUniform = {SamplerType(texImg), SamplerType(aoImg)};
    }

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const float invDepth = std::get<static_cast<int>(InTraits::INV_DEPTH_INDEX)>(in);

        const auto realTex = tex / invDepth;

        auto color = mUniform.textureSampler.get(realTex[0], realTex[1]);
        auto ao = mUniform.aoSampler.get(realTex[0], realTex[1]);

        color = color.cwiseProduct(ao) / 255.0;

        return std::make_tuple(color, pos[2]);
    }
};

class FlatVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
        Eigen::Vector3f lightPos;
    };

    Uniform mUniform;
public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> InType;
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

    FlatVertexShader(const Eigen::Vector3f& lightPos) {
        mUniform.lightPos = lightPos;
    }

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos    = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector3f& normal = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);

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

    Eigen::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    Eigen::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class PhongVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
        Eigen::Vector3f lightPos;
    };

    Uniform mUniform;
public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> InType;
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        LIGHT_INDEX = 2,
        VIEW_INDEX = 3
    };

    PhongVertexShader(const Eigen::Vector3f& lightPos) {
        mUniform.lightPos = lightPos;
    }

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos    = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector3f& normal = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);

        const Eigen::Vector3f& lightPos = mUniform.lightPos;

        const Eigen::Vector4f P = mUniform.modelViewMatrix * pos;

        const Eigen::Vector3f N = mUniform.modelViewMatrix.template topLeftCorner<3,3>() * normal;
        const Eigen::Vector3f L = lightPos - P.topRows<3>();
        const Eigen::Vector3f V = -P.topRows<3>();

        const Eigen::Vector4f outPos = mUniform.projMatrix * P;
        return std::make_tuple(outPos, N, L, V);
    }

    Eigen::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    Eigen::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class PhongFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        LIGHT_INDEX = 2,
        VIEW_INDEX = 3
    };

public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> InType;
    typedef std::tuple<Eigen::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        Eigen::Vector3f N     = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);
        Eigen::Vector3f L     = std::get<static_cast<int>(InTraits::LIGHT_INDEX)>(in);
        Eigen::Vector3f V     = std::get<static_cast<int>(InTraits::VIEW_INDEX)>(in);
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

class NormalViewVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    };

    Uniform mUniform;
public:
	typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> InType;
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        VIEW_INDEX = 2
    };

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos    = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector3f& normal = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);

        const Eigen::Vector4f P = mUniform.modelViewMatrix * pos;

        const Eigen::Vector3f N = mUniform.modelViewMatrix.template topLeftCorner<3,3>() * normal;
        const Eigen::Vector3f V = P.topRows<3>();

        const Eigen::Vector4f outPos = mUniform.projMatrix * P;
        return std::make_tuple(outPos, N, V);
    }

    Eigen::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    Eigen::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class EquiRectFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        VIEW_INDEX = 2
    };

    struct Uniform {
        RGBATextureSampler<Eigen::Vector4f> textureSampler;
    };

    Uniform mUniform;
public:
	typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f> InType;
    typedef std::tuple<Eigen::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    EquiRectFragmentShader(const std::string& filename) : mUniform{RGBATextureSampler<Eigen::Vector4f>(filename)} {}

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        Eigen::Vector3f N     = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);
        Eigen::Vector3f V     = std::get<static_cast<int>(InTraits::VIEW_INDEX)>(in);
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

class CubemapFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        VIEW_INDEX = 2
    };

    struct Uniform {
        CubeSampler<Eigen::Vector4f> cubeSampler;
    };

    Uniform mUniform;
public:
	typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f> InType;
    typedef std::tuple<Eigen::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    CubemapFragmentShader(const std::string& filename) : mUniform{CubeSampler<Eigen::Vector4f>(filename)} {}

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        Eigen::Vector3f N     = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);
        Eigen::Vector3f V     = std::get<static_cast<int>(InTraits::VIEW_INDEX)>(in);
        //assert(pos[2] < 0.0);

        N.normalize();
        V.normalize();

        auto R = reflect(V, N);

        auto color = mUniform.cubeSampler.get(R[0], R[1], R[2]);

        return std::make_tuple(color, pos[2]);
    }
};

class ShadowGenVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    };

    Uniform mUniform;
public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> InType;
    typedef std::tuple<Eigen::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0
    };

    ShadowGenVertexShader(const Eigen::Matrix4f& shadowProjectionMatrix, const Eigen::Matrix4f& shadowModelViewMatrix) {
        mUniform.projMatrix = shadowProjectionMatrix;
        mUniform.modelViewMatrix = shadowModelViewMatrix;
    }

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);

        const Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos);
    }

    Eigen::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    Eigen::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class ShadowGenFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0
    };
public:
    typedef std::tuple<Eigen::Vector4f> InType;
    typedef std::tuple<Eigen::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);

        return std::make_tuple(Eigen::Vector4f(), pos[2]);
    }

};

class ShadowTextureVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1
    };

    struct ShadowFragUniformType {
        RGBATextureSampler<Eigen::Vector4f> textureSampler;
        SingleChannelTextureSampler<float> shadowSampler;
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
        Eigen::Matrix4f shadowMatrix;
    };


    Uniform mUniform;
public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> InType;
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, Eigen::Vector4f, float> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        SHADOW_INDEX = 2,
        INV_DEPTH_INDEX = 3
    };

    ShadowTextureVertexShader(const Eigen::Matrix4f& shadowMatrix) {
        mUniform.shadowMatrix = shadowMatrix;
    }

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector2f tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);

        const Eigen::Vector4f mvPos = mUniform.modelViewMatrix * pos;
        const Eigen::Vector4f outPos = mUniform.projMatrix * mvPos;

        const float invDepth = 1.0 / mvPos[2];

        const Eigen::Vector4f shadow_coord = mUniform.shadowMatrix * pos;

        return std::make_tuple(outPos, tex * invDepth, shadow_coord, invDepth);
    }

    Eigen::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    Eigen::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class ShadowTextureFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        SHADOW_INDEX = 2,
        INV_DEPTH_INDEX = 3,
    };

    struct Uniform {
        RGBATextureSampler<Eigen::Vector4f> textureSampler;
        SingleChannelTextureSampler<float> shadowSampler;
    };

    Uniform mUniform;
public:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, Eigen::Vector4f, float> InType;
    typedef std::tuple<Eigen::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    ShadowTextureFragmentShader(const std::string& filename) : mUniform{RGBATextureSampler<Eigen::Vector4f>(filename),
                                                        SingleChannelTextureSampler<float>(2048, 2048)} {}

    OutType operator()(const InType& in) const {
        const Eigen::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const Eigen::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const Eigen::Vector4f& shadow_coord   = std::get<static_cast<int>(InTraits::SHADOW_INDEX)>(in);
        const float invDepth = std::get<static_cast<int>(InTraits::INV_DEPTH_INDEX)>(in);

        const auto realTex = tex / invDepth;

        auto color = mUniform.textureSampler.get(realTex[0], realTex[1]);

        const float shadow_depth = mUniform.shadowSampler.get(shadow_coord[0] / shadow_coord[3], shadow_coord[1] / shadow_coord[3]);

        float shade = 0.0;

        if(shadow_coord[2] / shadow_coord[3] - shadow_depth > 0.01) {
            shade = 0.1;
        } else {
            shade = 1.0;
        }

        return std::make_tuple(color * shade, pos[2]);
    }

    Uniform& uniform() { return mUniform; }

};

class CImgColorRasterShader {
private:
    enum class InTraits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    cimg_library::CImg<unsigned char>& mFrame;
    cimg_library::CImg<float>& mDepth;
public:
    typedef std::tuple<Eigen::Vector4f, float> InType;

    CImgColorRasterShader(cimg_library::CImg<unsigned char>& frame, cimg_library::CImg<float>& depth) : mFrame(frame), mDepth(depth) {}

    void operator()(const InType& fragment, unsigned int x, unsigned int y) const {
        constexpr int ColorAttachment = static_cast<int>(InTraits::COLOR_INDEX);
        constexpr int DepthAttachment = static_cast<int>(InTraits::DEPTH_INDEX);

        const auto& depth = std::get<DepthAttachment>(fragment);
        const auto& color = std::get<ColorAttachment>(fragment);

        if(depth > 0.0 && depth < 1.0 && depth < mDepth(x, y)) {
            mFrame(x, y, 0) = color[0];
            mFrame(x, y, 1) = color[1];
            mFrame(x, y, 2) = color[2];

            mDepth(x, y) = depth;
        }
    }

    void clear() const {
        mFrame.fill(0);
        mDepth.fill(std::numeric_limits<float>::max());
    }
};

class CImgDepthRasterShader {
private:
    enum class InTraits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    cimg_library::CImg<float>& mDepth;
public:
    typedef std::tuple<Eigen::Vector4f, float> InType;

    CImgDepthRasterShader(cimg_library::CImg<float>& depth) : mDepth(depth) {}

    void operator()(const InType& fragment, unsigned int x, unsigned int y) const {
        constexpr int DepthAttachment = static_cast<int>(InTraits::DEPTH_INDEX);

        const auto& depth = std::get<DepthAttachment>(fragment);

        if(depth > 0.0 && depth < 1.0 && depth < mDepth(x, y)) {
            mDepth(x, y) = depth;
        }
    }

    void clear() const {
        mDepth.fill(std::numeric_limits<float>::max());
    }
};

#endif /* SHADERS_H_ */
