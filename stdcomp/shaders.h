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
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { COLOR_ATTACHMENT = 1 };
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    };

public:
    Uniform mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int COLOR_ATTACHMENT = 1;
    };

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector4f& color = std::get<InTraits::COLOR_ATTACHMENT>(in);

        Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos, color);
    }

    Uniform& uniform() { return mUniform; }
};

class ColorFragmentShader {
private:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { COLOR_ATTACHMENT = 1 };
    };

public:
    typedef In VertOutFragInType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector4f& color = std::get<InTraits::COLOR_ATTACHMENT>(in);
        //assert(pos[2] < 0.0);

        return std::make_tuple(color, pos[2]);
    }
};

class TextureVertexShader {
private:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { TEXTURE_ATTACHMENT = 1 };
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    };
public:
    Uniform mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, float> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int TEXTURE_ATTACHMENT = 1;
        static constexpr int INV_DEPTH_ATTACHMENT = 2;
    };

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

class TextureFragmentShader {
private:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, float> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { TEXTURE_ATTACHMENT = 1 };
        enum { INV_DEPTH_ATTACHMENT = 2 };
    };

    typedef RGBATextureSampler<Eigen::Vector4f> SamplerType;

    struct Uniform {
        SamplerType textureSampler;
    };
public:
    Uniform mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    TextureFragmentShader(const std::string& filename) {
        cimg_library::CImg<unsigned char> texImg(filename.c_str());

        mUniform = {SamplerType(texImg)};
    }

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector2f& tex = std::get<InTraits::TEXTURE_ATTACHMENT>(in);
        const float invDepth = std::get<InTraits::INV_DEPTH_ATTACHMENT>(in);

        const auto realTex = tex / invDepth;

        auto color = mUniform.textureSampler.get(realTex[0], realTex[1]);

        return std::make_tuple(color, pos[2]);
    }
};

class MultiTextureFragmentShader {
private:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, float> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { TEXTURE_ATTACHMENT = 1 };
        enum { INV_DEPTH_ATTACHMENT = 2 };
    };

    typedef RGBATextureSampler<Eigen::Vector4f> SamplerType;

    struct Uniform {
        SamplerType textureSampler;
        SamplerType aoSampler;
    };
public:
    Uniform mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    MultiTextureFragmentShader(const std::string& filename1, const std::string& filename2) {
        cimg_library::CImg<unsigned char> texImg(filename1.c_str());
        cimg_library::CImg<unsigned char> aoImg(filename2.c_str());

        mUniform = {SamplerType(texImg), SamplerType(aoImg)};
    }

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector2f& tex = std::get<InTraits::TEXTURE_ATTACHMENT>(in);
        const float invDepth = std::get<InTraits::INV_DEPTH_ATTACHMENT>(in);

        const auto realTex = tex / invDepth;

        auto color = mUniform.textureSampler.get(realTex[0], realTex[1]);
        auto ao = mUniform.aoSampler.get(realTex[0], realTex[1]);

        color = color.cwiseProduct(ao) / 255.0;

        return std::make_tuple(color, pos[2]);
    }
};

class FlatVertexShader {
private:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { NORMAL_ATTACHMENT = 1 };
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
        Eigen::Vector3f lightPos;
    };
public:
    Uniform mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int COLOR_ATTACHMENT = 1;
    };

    FlatVertexShader(const Eigen::Vector3f& lightPos) {
        mUniform.lightPos = lightPos;
    }

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

class PhongVertexShader {
private:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { NORMAL_ATTACHMENT = 1 };
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
        Eigen::Vector3f lightPos;
    };
public:
    Uniform mUniform; //TODO: Private?
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int NORMAL_ATTACHMENT = 1;
        static constexpr int LIGHT_ATTACHMENT = 2;
        static constexpr int VIEW_ATTACHMENT = 3;
    };

    PhongVertexShader(const Eigen::Vector3f& lightPos) {
        mUniform.lightPos = lightPos;
    }

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

class PhongFragmentShader {
private:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> In;

    struct InTraits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int NORMAL_ATTACHMENT = 1;
        static constexpr int LIGHT_ATTACHMENT = 2;
        static constexpr int VIEW_ATTACHMENT = 3;
    };

public:
    typedef In VertOutFragInType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

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

class NormalViewVertexShader {
private:
	typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { NORMAL_ATTACHMENT = 1 };
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    };

public:
    Uniform mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int NORMAL_ATTACHMENT = 1;
        static constexpr int VIEW_ATTACHMENT = 2;
    };

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

class EquiRectFragmentShader {
private:
	typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f> In;

    struct InTraits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int NORMAL_ATTACHMENT = 1;
        static constexpr int VIEW_ATTACHMENT = 2;
    };
    struct Uniform {
        RGBATextureSampler<Eigen::Vector4f> textureSampler;
    };
public:
    Uniform mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    EquiRectFragmentShader(const std::string& filename) : mUniform{RGBATextureSampler<Eigen::Vector4f>(filename)} {}

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

class CubemapFragmentShader {
private:
	typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f> In;

    struct InTraits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int NORMAL_ATTACHMENT = 1;
        static constexpr int VIEW_ATTACHMENT = 2;
    };

    struct Uniform {
        CubeSampler<Eigen::Vector4f> cubeSampler;
    };
public:
    Uniform mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    CubemapFragmentShader(const std::string& filename) : mUniform{CubeSampler<Eigen::Vector4f>(filename)} {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        Eigen::Vector3f N     = std::get<InTraits::NORMAL_ATTACHMENT>(in);
        Eigen::Vector3f V     = std::get<InTraits::VIEW_ATTACHMENT>(in);
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
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { TEXTURE_ATTACHMENT = 1 };
    };

    struct Uniform {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    };
public:
    Uniform mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
    };

    ShadowGenVertexShader(const Eigen::Matrix4f& shadowProjectionMatrix, const Eigen::Matrix4f& shadowModelViewMatrix) {
        mUniform.projMatrix = shadowProjectionMatrix;
        mUniform.modelViewMatrix = shadowModelViewMatrix;
    }

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);

        const Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos);
    }

    Uniform& uniform() { return mUniform; }
};

class ShadowGenFragmentShader {
private:
    typedef std::tuple<Eigen::Vector4f> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
    };
public:
    typedef In VertOutFragInType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);

        return std::make_tuple(Eigen::Vector4f(), pos[2]);
    }

};

class ShadowTextureVertexShader {
private:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { TEXTURE_ATTACHMENT = 1 };
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

public:
    Uniform mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int TEXTURE_ATTACHMENT = 1;
        static constexpr int SHADOW_ATTACHMENT = 2;
        static constexpr int INV_DEPTH_ATTACHMENT = 3;
    };

    ShadowTextureVertexShader(const Eigen::Matrix4f& shadowMatrix) {
        mUniform.shadowMatrix = shadowMatrix;
    }

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector2f tex = std::get<InTraits::TEXTURE_ATTACHMENT>(in);

        const Eigen::Vector4f mvPos = mUniform.modelViewMatrix * pos;
        const Eigen::Vector4f outPos = mUniform.projMatrix * mvPos;

        const float invDepth = 1.0 / mvPos[2];

        const Eigen::Vector4f shadow_coord = mUniform.shadowMatrix * pos;

        return std::make_tuple(outPos, tex * invDepth, shadow_coord, invDepth);
    }

    Uniform& uniform() { return mUniform; }
};

class ShadowTextureFragmentShader {
private:
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f, Eigen::Vector4f, float> In;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { TEXTURE_ATTACHMENT = 1 };
        enum { SHADOW_ATTACHMENT = 2 };
        enum { INV_DEPTH_ATTACHMENT = 3 };
    };

    struct Uniform {
        RGBATextureSampler<Eigen::Vector4f> textureSampler;
        SingleChannelTextureSampler<float> shadowSampler;
    };
public:
    Uniform mUniform;
    typedef In VertOutFragInType;
    typedef Uniform FragUniformType;

    typedef std::tuple<Eigen::Vector4f, float> OutType;

    struct Traits {
        static constexpr int COLOR_ATTACHMENT = 0;
        static constexpr int DEPTH_ATTACHMENT = 1;
    };

    ShadowTextureFragmentShader(const std::string& filename) : mUniform{RGBATextureSampler<Eigen::Vector4f>(filename),
                                                        SingleChannelTextureSampler<float>(2048, 2048)} {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector2f& tex = std::get<InTraits::TEXTURE_ATTACHMENT>(in);
        const Eigen::Vector4f& shadow_coord   = std::get<InTraits::SHADOW_ATTACHMENT>(in);
        const float invDepth = std::get<InTraits::INV_DEPTH_ATTACHMENT>(in);

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
    cimg_library::CImg<unsigned char>& mFrame;
    cimg_library::CImg<float>& mDepth;
public:
    CImgColorRasterShader(cimg_library::CImg<unsigned char>& frame, cimg_library::CImg<float>& depth) : mFrame(frame), mDepth(depth) {}

    template<class Color>
    void operator()(const Color& color, float depth, unsigned int x, unsigned int y) const {

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
    cimg_library::CImg<float>& mDepth;
public:
    CImgDepthRasterShader(cimg_library::CImg<float>& depth) : mDepth(depth) {}

    template<class Color>
    void operator()(const Color& color, float depth, unsigned int x, unsigned int y) const {

        if(depth > 0.0 && depth < 1.0 && depth < mDepth(x, y)) {
            mDepth(x, y) = depth;
        }
    }

    void clear() const {
        mDepth.fill(std::numeric_limits<float>::max());
    }
};

#endif /* SHADERS_H_ */
