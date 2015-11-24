/*
 * shaders.h
 *
 *  Created on: Oct 30, 2014
 *      Author: per
 */

#ifndef SHADERS_H_
#define SHADERS_H_

#include <cmath>

#include "../utils.h"
#include "samplers.h"

class ColorVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

    struct Uniform {
        VecLib::Matrix4f projMatrix;
        VecLib::Matrix4f modelViewMatrix;
    };

    Uniform mUniform;

public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector4f> InType;
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector4f& color = std::get<static_cast<int>(InTraits::COLOR_INDEX)>(in);

        VecLib::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos, color);
    }

    VecLib::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class ColorFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector4f> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector4f& color = std::get<static_cast<int>(InTraits::COLOR_INDEX)>(in);
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
        VecLib::Matrix4f projMatrix;
        VecLib::Matrix4f modelViewMatrix;
    };
    Uniform mUniform;

public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f> InType;
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f, float> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2
    };

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);

        const VecLib::Vector4f mvPos = mUniform.modelViewMatrix * pos;
        const VecLib::Vector4f outPos = mUniform.projMatrix * mvPos;

        const float invDepth = 1.0 / mvPos[2];

        return std::make_tuple(outPos, tex * invDepth, invDepth);
    }

    VecLib::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class TextureFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2
    };

    typedef RGBATextureSampler<VecLib::Vector4f> SamplerType;

    struct Uniform {
        SamplerType textureSampler;
    };

    Uniform mUniform;
public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f, float> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    TextureFragmentShader(const std::string& filename) {
        cimg_library::CImg<unsigned char> texImg(filename.c_str());

        mUniform = {SamplerType(texImg)};
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
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

    typedef RGBATextureSampler<VecLib::Vector4f> SamplerType;

    struct Uniform {
        SamplerType textureSampler;
        SamplerType aoSampler;
    };

    Uniform mUniform;
public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f, float> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

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
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const float invDepth = std::get<static_cast<int>(InTraits::INV_DEPTH_INDEX)>(in);

        const auto realTex = tex / invDepth;

        auto color = mUniform.textureSampler.get(realTex[0], realTex[1]);
        auto ao = mUniform.aoSampler.get(realTex[0], realTex[1]);

        color = color.cwiseProduct(ao) / 255.0f;

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
        VecLib::Matrix4f projMatrix;
        VecLib::Matrix4f modelViewMatrix;
        VecLib::Vector3f lightPos;
    };

    Uniform mUniform;
public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

    FlatVertexShader(const VecLib::Vector3f& lightPos) {
        mUniform.lightPos = lightPos;
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos    = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector3f& normal = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);

        const VecLib::Vector3f& lightPos = mUniform.lightPos;

        const VecLib::Vector4f P = mUniform.modelViewMatrix * pos;

        VecLib::Vector3f N = VecLib::Matrix3f(mUniform.modelViewMatrix) * normal;
        VecLib::Vector3f L = lightPos - P.xyz();
        VecLib::Vector3f V = -P.xyz();

        N.normalize();
        L.normalize();
        V.normalize();

        const auto R = reflect(-L, N);

        const float intensity = std::max(0.0f, R.dot(V));

        const VecLib::Vector4f color(255,255,255,255);

        const VecLib::Vector4f outPos = mUniform.projMatrix * P;
        return std::make_tuple(outPos, color*intensity);
    }

    VecLib::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class PhongVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1
    };

    struct Uniform {
        VecLib::Matrix4f projMatrix;
        VecLib::Matrix4f modelViewMatrix;
        VecLib::Vector3f lightPos;
    };

    Uniform mUniform;
public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f, VecLib::Vector3f, VecLib::Vector3f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        LIGHT_INDEX = 2,
        VIEW_INDEX = 3
    };

    PhongVertexShader(const VecLib::Vector3f& lightPos) {
        mUniform.lightPos = lightPos;
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos    = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector3f& normal = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);

        const VecLib::Vector3f& lightPos = mUniform.lightPos;

        const VecLib::Vector4f P = mUniform.modelViewMatrix * pos;

        const VecLib::Vector3f N = VecLib::Matrix3f(mUniform.modelViewMatrix) * normal;
        const VecLib::Vector3f L = lightPos - P.xyz();
        const VecLib::Vector3f V = -P.xyz();

        const VecLib::Vector4f outPos = mUniform.projMatrix * P;
        return std::make_tuple(outPos, N, L, V);
    }

    VecLib::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
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
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f, VecLib::Vector3f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        VecLib::Vector3f N     = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);
        VecLib::Vector3f L     = std::get<static_cast<int>(InTraits::LIGHT_INDEX)>(in);
        VecLib::Vector3f V     = std::get<static_cast<int>(InTraits::VIEW_INDEX)>(in);
        //assert(pos[2] < 0.0);

        N.normalize();
        L.normalize();
        V.normalize();

        const auto R = reflect(-L, N);

        const float intensity = std::max(0.0f, R.dot(V));

        const VecLib::Vector4f color(255,255,255,255);

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
        VecLib::Matrix4f projMatrix;
        VecLib::Matrix4f modelViewMatrix;
    };

    Uniform mUniform;
public:
	typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f, VecLib::Vector3f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        VIEW_INDEX = 2
    };

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos    = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector3f& normal = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);

        const VecLib::Vector4f P = mUniform.modelViewMatrix * pos;

        const VecLib::Vector3f N = VecLib::Matrix3f(mUniform.modelViewMatrix) * normal;
        const VecLib::Vector3f V = P.xyz();

        const VecLib::Vector4f outPos = mUniform.projMatrix * P;
        return std::make_tuple(outPos, N, V);
    }

    VecLib::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class EquiRectFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        VIEW_INDEX = 2
    };

    struct Uniform {
        RGBATextureSampler<VecLib::Vector4f> textureSampler;
    };

    Uniform mUniform;
public:
	typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    EquiRectFragmentShader(const std::string& filename) : mUniform{RGBATextureSampler<VecLib::Vector4f>(filename)} {}

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        VecLib::Vector3f N     = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);
        VecLib::Vector3f V     = std::get<static_cast<int>(InTraits::VIEW_INDEX)>(in);
        //assert(pos[2] < 0.0);

        N.normalize();
        V.normalize();

        auto R = reflect(V, N);

        R.normalize();

        VecLib::Vector2f tex;

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
        CubeSampler<VecLib::Vector4f> cubeSampler;
    };

    Uniform mUniform;
public:
	typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    CubemapFragmentShader(const std::string& filename) : mUniform{CubeSampler<VecLib::Vector4f>(filename)} {}

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        VecLib::Vector3f N     = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);
        VecLib::Vector3f V     = std::get<static_cast<int>(InTraits::VIEW_INDEX)>(in);
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
        VecLib::Matrix4f projMatrix;
        VecLib::Matrix4f modelViewMatrix;
    };

    Uniform mUniform;
public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f> InType;
    typedef std::tuple<VecLib::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0
    };

    ShadowGenVertexShader(const VecLib::Matrix4f& shadowProjectionMatrix, const VecLib::Matrix4f& shadowModelViewMatrix) {
        mUniform.projMatrix = shadowProjectionMatrix;
        mUniform.modelViewMatrix = shadowModelViewMatrix;
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);

        const VecLib::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos);
    }

    VecLib::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
};

class ShadowGenFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0
    };
public:
    typedef std::tuple<VecLib::Vector4f> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);

        return std::make_tuple(VecLib::Vector4f(), pos[2]);
    }

};

class ShadowTextureVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1
    };

    struct ShadowFragUniformType {
        RGBATextureSampler<VecLib::Vector4f> textureSampler;
        SingleChannelTextureSampler<float> shadowSampler;
    };

    struct Uniform {
        VecLib::Matrix4f projMatrix;
        VecLib::Matrix4f modelViewMatrix;
        VecLib::Matrix4f shadowMatrix;
    };


    Uniform mUniform;
public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f> InType;
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f, VecLib::Vector4f, float> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        SHADOW_INDEX = 2,
        INV_DEPTH_INDEX = 3
    };

    ShadowTextureVertexShader(const VecLib::Matrix4f& shadowMatrix) {
        mUniform.shadowMatrix = shadowMatrix;
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);

        const VecLib::Vector4f mvPos = mUniform.modelViewMatrix * pos;
        const VecLib::Vector4f outPos = mUniform.projMatrix * mvPos;

        const float invDepth = 1.0 / mvPos[2];

        const VecLib::Vector4f shadow_coord = mUniform.shadowMatrix * pos;

        return std::make_tuple(outPos, tex * invDepth, shadow_coord, invDepth);
    }

    VecLib::Matrix4f& projMatrix() { return mUniform.projMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mUniform.modelViewMatrix; }
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
        RGBATextureSampler<VecLib::Vector4f> textureSampler;
        SingleChannelTextureSampler<float> shadowSampler;
    };

    Uniform mUniform;
public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f, VecLib::Vector4f, float> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    ShadowTextureFragmentShader(const std::string& filename) : mUniform{RGBATextureSampler<VecLib::Vector4f>(filename),
                                                        SingleChannelTextureSampler<float>(2048, 2048)} {}

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const VecLib::Vector4f& shadow_coord   = std::get<static_cast<int>(InTraits::SHADOW_INDEX)>(in);
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

    cimg_library::CImg<float>& getDepthTexture() { return mUniform.shadowSampler.texture(); }

};

class ShadertoyVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0
    };

public:
    typedef std::tuple<VecLib::Vector2f> InType;
    typedef std::tuple<VecLib::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0
    };

    OutType operator()(const InType& in) const {
        const VecLib::Vector2f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        VecLib::Vector4f outPos(pos, 0.0f, 1.0f);

        return std::make_tuple(outPos);
    }

};

float mod(float x, float y) {
    return std::fmod(x, y);
}

float step(float edge, float x) {
    return x < edge ? 0.0f : 1.0f;
}

float clamp(float val, float min, float max) {
    return val < min ? min : val > max ? max : val;
}

float smoothstep(float edge0, float edge1, float x)
{
    // Scale, bias and saturate x to 0..1 range
    x = clamp((x - edge0)/(edge1 - edge0), 0.0, 1.0);
    // Evaluate polynomial
    return x*x*(3 - 2*x);
}



VecLib::Vector3f mix(const VecLib::Vector3f& x, const VecLib::Vector3f& y, float a) {
    VecLib::Vector3f mx = x * (1.0f - a);
    VecLib::Vector3f my = y * a;
    return mx + my;
}

class ShadertoyFragmentShader {
protected:
    using vec2 = VecLib::Vector2f;
    using vec3 = VecLib::Vector3f;
    using vec4 = VecLib::Vector4f;
    using mat2 = VecLib::Matrix2f;
    using mat3 = VecLib::Matrix3f;
    using mat4 = VecLib::Matrix4f;

    enum class InTraits {
        POSITION_INDEX = 0
    };

    float iGlobalTime;
    VecLib::Vector2f iResolution;

public:
    typedef std::tuple<VecLib::Vector4f> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    ShadertoyFragmentShader() : iResolution(640, 480) {}

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    void setTime(float t) {
        iGlobalTime = t;
        //printf("%f\n", iGlobalTime);
    }

    virtual void mainImage(vec4& fragColor, const vec2& fragCoord) const = 0;

    OutType operator()(const InType& in) const {


        const VecLib::Vector4f& fragCoord = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);

        VecLib::Vector4f fragColor;

        mainImage(fragColor, fragCoord.xy());


        return std::make_tuple(fragColor * 255.0f, 0.2f);
    }

};

inline float abs(float v) {
    return std::abs(v);
}

inline double abs(double v) {
    return std::abs(v);
}

class ShadertoyWaveFragmentShader : public ShadertoyFragmentShader {
public:
    const vec3 COLOR1 = vec3(0.0, 0.0, 0.3);
    const vec3 COLOR2 = vec3(0.5, 0.0, 0.0);
    static constexpr float BLOCK_WIDTH = 0.01;

    //https://www.shadertoy.com/view/4dsGzH
    virtual void mainImage(vec4& fragColor, const vec2& fragCoord) const {

        vec2 uv = fragCoord.xy() / iResolution.xy();

        // To create the BG pattern
        vec3 final_color = vec3(1.0);
        vec3 bg_color = vec3(0.0);
        vec3 wave_color = vec3(0.0);

        float c1 = mod(uv.x(), 2.0 * BLOCK_WIDTH);
        c1 = step(BLOCK_WIDTH, c1);

        float c2 = mod(uv.y(), 2.0 * BLOCK_WIDTH);
        c2 = step(BLOCK_WIDTH, c2);

        bg_color = mix(uv.x() * COLOR1, uv.y() * COLOR2, c1 * c2);

        // To create the waves
        float wave_width = 0.01;
        uv  = -1.0f + 2.0f * uv;
        uv.y() += 0.1;
        for(float i = 0.0; i < 10.0; i++) {

            uv.y() += (0.07 * sin(uv.x() + i/7.0 + iGlobalTime ));
            wave_width = abs(1.0 / (150.0 * uv.y()));
            wave_color += vec3(wave_width * 1.9, wave_width, wave_width * 1.5);
        }

        final_color = bg_color + wave_color;


        fragColor = vec4(final_color, 1.0);
    }
};

class ShadertoySeascapeFragmentShader : public ShadertoyFragmentShader {
    //https://www.shadertoy.com/view/Ms2SD1
public:
    static const int NUM_STEPS = 8;
    static constexpr float PI	 	= 3.1415;
    static constexpr float EPSILON	= 1e-3;
    float EPSILON_NRM	= 0.1 / iResolution.x();

// sea
    static const int ITER_GEOMETRY = 3;
    static const int ITER_FRAGMENT = 5;
    static constexpr float SEA_HEIGHT = 0.6;
    static constexpr float SEA_CHOPPY = 4.0;
    static constexpr float SEA_SPEED = 0.8;
    static constexpr float SEA_FREQ = 0.16;


// math
    static mat3 fromEuler(vec3 ang) {
        vec2 a1 = vec2(sin(ang.x()),cos(ang.x()));
        vec2 a2 = vec2(sin(ang.y()),cos(ang.y()));
        vec2 a3 = vec2(sin(ang.z()),cos(ang.z()));
        mat3 m;
        m[0] = vec3(a1.y()*a3.y()+a1.x()*a2.x()*a3.x(),a1.y()*a2.x()*a3.x()+a3.y()*a1.x(),-a2.y()*a3.x());
        m[1] = vec3(-a2.y()*a1.x(),a1.y()*a2.y(),a2.x());
        m[2] = vec3(a3.y()*a1.x()*a2.x()+a1.y()*a3.x(),a1.x()*a3.x()-a1.y()*a3.y()*a2.x(),a2.y()*a3.y());
        return m;
    }
    static float hash( vec2 p ) {
        float h = dot(p,vec2(127.1,311.7));
        return VecLib::fract(sin(h)*43758.5453123); //TODO
    }
    static float noise(  vec2 p ) {
        vec2 i = floor( p );
        vec2 f = fract( p );
        vec2 u = f*f*(3.0f-2.0f*f); //TODO
        return -1.0+2.0*VecLib::mix( VecLib::mix( hash( i + vec2(0.0,0.0) ), //TODO
                                  hash( i + vec2(1.0,0.0) ), u.x()),
                                     VecLib::mix( hash( i + vec2(0.0,1.0) ),
                                  hash( i + vec2(1.0,1.0) ), u.x()), u.y());
    }

// lighting
    static float diffuse(vec3 n,vec3 l,float p) {
        return pow(dot(n,l) * 0.4 + 0.6,p);
    }
    static float specular(vec3 n,vec3 l,vec3 e,float s) {
        float nrm = (s + 8.0) / (3.1415 * 8.0);
        return pow(std::max(dot(reflect(e,n),l),0.0f),s) * nrm; //TODO
    }

// sky
    static vec3 getSkyColor(vec3 e) {
        e.y() = std::max(e.y(),0.0f); //TODO
        vec3 ret;
        ret.x() = std::pow(1.0-e.y(),2.0); //TODO
        ret.y() = 1.0-e.y();
        ret.z() = 0.6+(1.0-e.y())*0.4;
        return ret;
    }

// sea
    static float sea_octave(vec2 uv, float choppy) {
        uv += noise(uv);
        vec2 wv = 1.0f-abs(sin(uv)); //TODO
        vec2 swv = abs(cos(uv));
        wv = mix(wv,swv,wv);
        return pow(1.0f-pow(wv.x() * wv.y(),0.65),choppy); //TODO
    }

    float map(vec3 p) const {
        const mat2 octave_m = mat2(1.6,1.2,-1.2,1.6);
        const float SEA_TIME = iGlobalTime * SEA_SPEED;
        float freq = SEA_FREQ;
        float amp = SEA_HEIGHT;
        float choppy = SEA_CHOPPY;
        vec2 uv = p.xz(); uv.x() *= 0.75;

        float d, h = 0.0;
        for(int i = 0; i < ITER_GEOMETRY; i++) {
            d = sea_octave((uv+SEA_TIME)*freq,choppy);
            d += sea_octave((uv-SEA_TIME)*freq,choppy);
            h += d * amp;
            uv *= octave_m; freq *= 1.9; amp *= 0.22;
            choppy = VecLib::mix(choppy,1.0,0.2); //TODO
        }
        return p.y() - h;
    }

    float map_detailed(vec3 p) const {
        const mat2 octave_m = mat2(1.6,1.2,-1.2,1.6);
        const float SEA_TIME = iGlobalTime * SEA_SPEED;
        float freq = SEA_FREQ;
        float amp = SEA_HEIGHT;
        float choppy = SEA_CHOPPY;
        vec2 uv = p.xz(); uv.x() *= 0.75;

        float d, h = 0.0;
        for(int i = 0; i < ITER_FRAGMENT; i++) {
            d = sea_octave((uv+SEA_TIME)*freq,choppy);
            d += sea_octave((uv-SEA_TIME)*freq,choppy);
            h += d * amp;
            uv *= octave_m; freq *= 1.9; amp *= 0.22;
            choppy = VecLib::mix(choppy,1.0,0.2);
        }
        return p.y() - h;
    }

    static vec3 getSeaColor(vec3 p, vec3 n, vec3 l, vec3 eye, vec3 dist) {
        static const vec3 SEA_BASE = vec3(0.1,0.19,0.22);
        static const vec3 SEA_WATER_COLOR = vec3(0.8,0.9,0.6);


        float fresnel = 1.0 - std::max(dot(n,-eye),0.0f); //TODO
        fresnel = pow(fresnel,3.0) * 0.65;

        vec3 reflected = getSkyColor(reflect(eye,n));
        vec3 refracted = SEA_BASE + diffuse(n,l,80.0) * SEA_WATER_COLOR * 0.12f;

        vec3 color = mix(refracted,reflected,fresnel);

        float atten = std::max(1.0 - dot(dist,dist) * 0.001, 0.0); //TODO
        color += SEA_WATER_COLOR * (p.y() - SEA_HEIGHT) * 0.18f * atten;

        color += vec3(specular(n,l,eye,60.0));

        return color;
    }

// tracing
    vec3 getNormal(vec3 p, float eps) const {
        vec3 n;
        n.y() = map_detailed(p);
        n.x() = map_detailed(vec3(p.x()+eps,p.y(),p.z())) - n.y();
        n.z() = map_detailed(vec3(p.x(),p.y(),p.z()+eps)) - n.y();
        n.y() = eps;
        return normalize(n);
    }

    float heightMapTracing(vec3 ori, vec3 dir, vec3& p) const {
        float tm = 0.0;
        float tx = 1000.0;
        float hx = map(ori + dir * tx);
        if(hx > 0.0) return tx;
        float hm = map(ori + dir * tm);
        float tmid = 0.0;
        for(int i = 0; i < NUM_STEPS; i++) {
            tmid = VecLib::mix(tm,tx, hm/(hm-hx));
            p = ori + dir * tmid;
            float hmid = map(p);
            if(hmid < 0.0) {
                tx = tmid;
                hx = hmid;
            } else {
                tm = tmid;
                hm = hmid;
            }
        }
        return tmid;
    }

// main
    virtual void mainImage( vec4& fragColor, const vec2& fragCoord ) const {
        vec2 uv = fragCoord.xy() / iResolution.xy();
        uv = uv * 2.0f - 1.0f;
        uv.x() *= iResolution.x() / iResolution.y();
        float time = iGlobalTime * 0.3;// + iMouse.x*0.01;

        // ray
        vec3 ang = vec3(sin(time*3.0)*0.1,sin(time)*0.2+0.3,time);
        vec3 ori = vec3(0.0,3.5,time*5.0);
        vec3 dir = normalize(vec3(uv.xy(),-2.0)); dir.z() += length(uv) * 0.15;
        dir = normalize(dir) * fromEuler(ang);

        // tracing
        vec3 p;
        heightMapTracing(ori,dir,p);
        vec3 dist = p - ori;
        vec3 n = getNormal(p, dot(dist,dist) * EPSILON_NRM);
        vec3 light = normalize(vec3(0.0,1.0,0.8));

        // color
        vec3 color = mix(
                getSkyColor(dir),
                getSeaColor(p,n,light,dir,dist),
                pow(smoothstep(0.0,-0.05,dir.y()),0.3));

        // post
        fragColor = vec4(pow(color,vec3(0.75)), 1.0);
    }
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
    typedef std::tuple<VecLib::Vector4f, float> InType;

    CImgColorRasterShader(cimg_library::CImg<unsigned char>& frame, cimg_library::CImg<float>& depth) : mFrame(frame), mDepth(depth) {}

    void operator()(const InType& fragment, unsigned int x, unsigned int y) const {
        constexpr int ColorAttachment = static_cast<int>(InTraits::COLOR_INDEX);
        constexpr int DepthAttachment = static_cast<int>(InTraits::DEPTH_INDEX);

        const auto& depth = std::get<DepthAttachment>(fragment);
        const auto& color = std::get<ColorAttachment>(fragment);

        if(depth > 0.0 && depth < 1.0 && depth < mDepth(x, y)) {
            mFrame(x, y, 0, 0) = color[0] <= 255 ? color[0] : 255;
            mFrame(x, y, 0, 1) = color[1] <= 255 ? color[1] : 255;
            mFrame(x, y, 0, 2) = color[2] <= 255 ? color[2] : 255;

            mDepth(x, y) = depth;
        }
    }

    void clear() const {
        mFrame.fill(0);
        mDepth.fill(std::numeric_limits<float>::max());
    }

    unsigned int getXResolution() const { return mFrame.width(); }
    unsigned int getYResolution() const { return mFrame.height(); }
};

class CImgDepthRasterShader {
private:
    enum class InTraits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    cimg_library::CImg<float>& mDepth;
public:
    typedef std::tuple<VecLib::Vector4f, float> InType;

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

    unsigned int getXResolution() const { return mDepth.width(); }
    unsigned int getYResolution() const { return mDepth.height(); }

};

#endif /* SHADERS_H_ */
