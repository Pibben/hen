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
#include "../veclib.h"
#include "samplers.h"
#if 0
class ColorVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

    VecLib::Matrix4f mProjMatrix;
    VecLib::Matrix4f mModelViewMatrix;

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

        VecLib::Vector4f outPos = mProjMatrix * mModelViewMatrix * pos;

        return std::make_tuple(outPos, color);
    }

    VecLib::Matrix4f& projMatrix() { return mProjMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mModelViewMatrix; }
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
#endif
class TextureVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1
    };

    VecLib::Matrix4f mProjMatrix;
    VecLib::Matrix4f mModelViewMatrix;

public:
    using InType = typename std::tuple<VecLib::Vector4f, VecLib::Vector2f>;
    using OutType = typename std::tuple<VecLib::Vector4f, VecLib::Vector2f, float>;

    enum class Traits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2
    };

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);

        const VecLib::Vector4f mvPos = mModelViewMatrix * pos;
        const VecLib::Vector4f outPos = mProjMatrix * mvPos;

        const float invDepth = 1.0f / mvPos[2];

        return std::make_tuple(outPos, tex * invDepth, invDepth);
    }

    VecLib::Matrix4f& projMatrix() { return mProjMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mModelViewMatrix; }
};

class TextureFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2
    };

    RGBATextureSampler<VecLib::Vector4f> mTextureSampler;

public:
    using InType = std::tuple<VecLib::Vector4f, VecLib::Vector2f, float>;
    using OutType = std::tuple<VecLib::Vector4f, float>;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    TextureFragmentShader(const std::string& filename) : mTextureSampler(filename) {}

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const float invDepth = std::get<static_cast<int>(InTraits::INV_DEPTH_INDEX)>(in);

        const auto realTex = tex / invDepth;

        auto color = mTextureSampler.get(realTex[0], realTex[1]);

        return std::make_tuple(color, pos[2]);
    }
};
#if 0
class MultiTextureFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2
    };

    RGBATextureSampler<VecLib::Vector4f> mTextureSampler;
    SingleChannelTextureSampler<float> mAoSampler;

public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f, float> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    MultiTextureFragmentShader(const std::string& filename1, const std::string& filename2)
        : mTextureSampler(filename1), mAoSampler(filename2) {}

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const float invDepth = std::get<static_cast<int>(InTraits::INV_DEPTH_INDEX)>(in);

        const auto realTex = tex / invDepth;

        auto color = mTextureSampler.get(realTex[0], realTex[1]);
        auto ao = mAoSampler.get(realTex[0], realTex[1]);

        return std::make_tuple(color, pos[2]);
    }
};

class FlatVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1
    };

    VecLib::Matrix4f mProjMatrix;
    VecLib::Matrix4f mModelViewMatrix;
    VecLib::Vector3f mLightPos;

public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        COLOR_INDEX = 1
    };

    FlatVertexShader(const VecLib::Vector3f& lightPos) {
        mLightPos = lightPos;
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos    = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector3f& normal = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);

        const VecLib::Vector3f& lightPos = mLightPos;

        const VecLib::Vector4f P = mModelViewMatrix * pos;

        VecLib::Vector3f N = VecLib::Matrix3f(mModelViewMatrix) * normal;
        VecLib::Vector3f L = lightPos - P.xyz();
        VecLib::Vector3f V = -P.xyz();

        N.normalize();
        L.normalize();
        V.normalize();

        const auto R = reflect(-L, N);

        const float intensity = std::max(0.0f, R.dot(V));

        const VecLib::Vector4f color(1.0f, 1.0f, 1.0f, 1.0f);

        const VecLib::Vector4f outPos = mProjMatrix * P;
        return std::make_tuple(outPos, color*intensity);
    }

    VecLib::Matrix4f& projMatrix() { return mProjMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mModelViewMatrix; }
};

class PhongVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1
    };

    VecLib::Matrix4f mProjMatrix;
    VecLib::Matrix4f mModelViewMatrix;
    VecLib::Vector3f mLightPos;

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
        mLightPos = lightPos;
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos    = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector3f& normal = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);

        const VecLib::Vector3f& lightPos = mLightPos;

        const VecLib::Vector4f P = mModelViewMatrix * pos;

        const VecLib::Vector3f N = VecLib::Matrix3f(mModelViewMatrix) * normal;
        const VecLib::Vector3f L = lightPos - P.xyz();
        const VecLib::Vector3f V = -P.xyz();

        const VecLib::Vector4f outPos = mProjMatrix * P;
        return std::make_tuple(outPos, N, L, V);
    }

    VecLib::Matrix4f& projMatrix() { return mProjMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mModelViewMatrix; }
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

        const VecLib::Vector4f color(1.0f, 1.0f, 1.0f, 1.0f);

        return std::make_tuple(color*intensity, pos[2]);
    }
};

class NormalViewVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1
    };

    VecLib::Matrix4f mProjMatrix;
    VecLib::Matrix4f mModelViewMatrix;

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

        const VecLib::Vector4f P = mModelViewMatrix * pos;

        const VecLib::Vector3f N = VecLib::Matrix3f(mModelViewMatrix) * normal;
        const VecLib::Vector3f V = P.xyz();

        const VecLib::Vector4f outPos = mProjMatrix * P;
        return std::make_tuple(outPos, N, V);
    }

    VecLib::Matrix4f& projMatrix() { return mProjMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mModelViewMatrix; }
};

class EquiRectFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        VIEW_INDEX = 2
    };

    RGBATextureSampler<VecLib::Vector4f> mTextureSampler;

public:
	typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    EquiRectFragmentShader(const std::string& filename) : mTextureSampler(RGBATextureSampler<VecLib::Vector4f>(filename)) {}

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

        auto color = mTextureSampler.get(tex[0], tex[1]);

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

    CubeSampler<VecLib::Vector4f> mCubeSampler;

public:
	typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    CubemapFragmentShader(const std::string& filename) : mCubeSampler(CubeSampler<VecLib::Vector4f>(filename)) {}

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        VecLib::Vector3f N     = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);
        VecLib::Vector3f V     = std::get<static_cast<int>(InTraits::VIEW_INDEX)>(in);
        //assert(pos[2] < 0.0);

        N.normalize();
        V.normalize();

        auto R = reflect(V, N);

        auto color = mCubeSampler.get(R[0], R[1], R[2]);

        return std::make_tuple(color, pos[2]);
    }
};

class ShadowGenVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1
    };

    VecLib::Matrix4f mProjMatrix;
    VecLib::Matrix4f mModelViewMatrix;

public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f> InType;
    typedef std::tuple<VecLib::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0
    };

    ShadowGenVertexShader(const VecLib::Matrix4f& shadowProjectionMatrix, const VecLib::Matrix4f& shadowModelViewMatrix) {
        mProjMatrix = shadowProjectionMatrix;
        mModelViewMatrix = shadowModelViewMatrix;
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);

        const VecLib::Vector4f outPos = mProjMatrix * mModelViewMatrix * pos;

        return std::make_tuple(outPos);
    }

    VecLib::Matrix4f& projMatrix() { return mProjMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mModelViewMatrix; }
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

//    struct ShadowFragUniformType {
//        RGBATextureSampler<VecLib::Vector4f> textureSampler;
//        SingleChannelTextureSampler<float> shadowSampler;
//    };

    VecLib::Matrix4f mProjMatrix;
    VecLib::Matrix4f mModelViewMatrix;
    VecLib::Matrix4f mShadowMatrix;

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
        mShadowMatrix = shadowMatrix;
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);

        const VecLib::Vector4f mvPos = mModelViewMatrix * pos;
        const VecLib::Vector4f outPos = mProjMatrix * mvPos;

        const float invDepth = 1.0 / mvPos[2];

        const VecLib::Vector4f shadow_coord = mShadowMatrix * pos;

        return std::make_tuple(outPos, tex * invDepth, shadow_coord, invDepth);
    }

    VecLib::Matrix4f& projMatrix() { return mProjMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mModelViewMatrix; }
};

class ShadowTextureFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        SHADOW_INDEX = 2,
        INV_DEPTH_INDEX = 3,
    };

    RGBATextureSampler<VecLib::Vector4f> mTextureSampler;
    SingleChannelTextureSampler<float, float> mShadowSampler;

public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f, VecLib::Vector4f, float> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    ShadowTextureFragmentShader(const std::string& filename) : mTextureSampler(RGBATextureSampler<VecLib::Vector4f>(filename)),
                                                               mShadowSampler(SingleChannelTextureSampler<float, float>(2048, 2048)) {}

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const VecLib::Vector4f& shadow_coord   = std::get<static_cast<int>(InTraits::SHADOW_INDEX)>(in);
        const float invDepth = std::get<static_cast<int>(InTraits::INV_DEPTH_INDEX)>(in);

        const auto realTex = tex / invDepth;

        auto color = mTextureSampler.get(realTex[0], realTex[1]);

        const float shadow_depth = mShadowSampler.get(shadow_coord[0] / shadow_coord[3], shadow_coord[1] / shadow_coord[3]);

        float shade = 0.0;

        if(shadow_coord[2] / shadow_coord[3] - shadow_depth > 0.01) {
            shade = 0.1;
        } else {
            shade = 1.0;
        }

        return std::make_tuple(color * shade, pos[2]);
    }

    cimg_library::CImg<float>& getDepthTexture() { return mShadowSampler.texture(); }

};
#endif
class ShadertoyVertexShader {
private:
    enum class InTraits { POSITION_INDEX = 0 };

public:
    using InType = std::tuple<VecLib::Vector2f>;
    using OutType = std::tuple<VecLib::Vector4f>;

    enum class Traits { POSITION_INDEX = 0 };

    OutType operator()(const InType& in) const {
        const VecLib::Vector2f& pos = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        VecLib::Vector4f outPos(pos, 0.0f, 1.0f);

        return std::make_tuple(outPos);
    }
};

static float mod(float x, float y) { return std::fmod(x, y); }

static float step(float edge, float x) { return x < edge ? 0.0f : 1.0f; }

static float clamp(float val, float min, float max) { return val < min ? min : val > max ? max : val; }

static float smoothstep(float edge0, float edge1, float x) {
    // Scale, bias and saturate x to 0..1 range
    x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    // Evaluate polynomial
    return x * x * (3 - 2 * x);
}

static VecLib::Vector3f mix(const VecLib::Vector3f& x, const VecLib::Vector3f& y, float a) {
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

    enum class InTraits { POSITION_INDEX = 0 };

    float iGlobalTime{0.0f};
    VecLib::Vector2f iResolution;

public:
    using InType = std::tuple<VecLib::Vector4f>;
    using OutType = std::tuple<VecLib::Vector4f, float>;

    ShadertoyFragmentShader() : iResolution(640, 480) {}

    enum class Traits { COLOR_INDEX = 0, DEPTH_INDEX = 1 };

    void setTime(float t) {
        iGlobalTime = t;
        // printf("%f\n", iGlobalTime);
    }

    virtual void mainImage(vec4& fragColor, const vec2& fragCoord) const = 0;  // NOLINT

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& fragCoord = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);

        VecLib::Vector4f fragColor;

        mainImage(fragColor, fragCoord.xy());

        return std::make_tuple(fragColor, 0.2f);
    }
};

class ShadertoyWaveFragmentShader : public ShadertoyFragmentShader {
public:
    const vec3 COLOR1 = vec3(0.0, 0.0, 0.3);
    const vec3 COLOR2 = vec3(0.5, 0.0, 0.0);
    static constexpr float BLOCK_WIDTH = 0.01;

    // https://www.shadertoy.com/view/4dsGzH
    void mainImage(vec4& fragColor, const vec2& fragCoord) const override {
        vec2 uv = fragCoord.xy() / iResolution.xy();

        // To create the BG pattern
        auto final_color = vec3(1.0f);
        auto bg_color = vec3(0.0f);
        auto wave_color = vec3(0.0f);

        float c1 = mod(uv.x(), 2.0f * BLOCK_WIDTH);
        c1 = step(BLOCK_WIDTH, c1);

        float c2 = mod(uv.y(), 2.0f * BLOCK_WIDTH);
        c2 = step(BLOCK_WIDTH, c2);

        bg_color = mix(uv.x() * COLOR1, uv.y() * COLOR2, c1 * c2);

        // To create the waves
        float wave_width = 0.01f;
        uv = -1.0f + 2.0f * uv;
        uv.y() += 0.1f;
        for (float i = 0.0f; i < 10.0f; i++) {                              // NOLINT
            uv.y() += (0.07f * std::sin(uv.x() + i / 7.0f + iGlobalTime));  // TODO
            wave_width = std::abs(1.0f / (150.0f * uv.y()));
            wave_color += vec3(wave_width * 1.9f, wave_width, wave_width * 1.5f);
        }

        final_color = bg_color + wave_color;

        fragColor = vec4(final_color, 1.0f);
    }
};

class ShadertoySeascapeFragmentShader : public ShadertoyFragmentShader {
    // https://www.shadertoy.com/view/Ms2SD1
public:
    static const int NUM_STEPS = 8;
    static constexpr float PI = 3.1415f;
    static constexpr float EPSILON = 1e-3f;
    float EPSILON_NRM = 0.1f / iResolution.x();

    // sea
    static const int ITER_GEOMETRY = 3;
    static const int ITER_FRAGMENT = 5;
    static constexpr float SEA_HEIGHT = 0.6f;
    static constexpr float SEA_CHOPPY = 4.0f;
    static constexpr float SEA_SPEED = 0.8f;
    static constexpr float SEA_FREQ = 0.16f;

    // math
    static mat3 fromEuler(vec3 ang) {
        vec2 a1 = vec2(std::sin(ang.x()), std::cos(ang.x()));
        vec2 a2 = vec2(std::sin(ang.y()), std::cos(ang.y()));
        vec2 a3 = vec2(std::sin(ang.z()), std::cos(ang.z()));
        mat3 m;
        m[0] = vec3(a1.y() * a3.y() + a1.x() * a2.x() * a3.x(), a1.y() * a2.x() * a3.x() + a3.y() * a1.x(),
                    -a2.y() * a3.x());
        m[1] = vec3(-a2.y() * a1.x(), a1.y() * a2.y(), a2.x());
        m[2] = vec3(a3.y() * a1.x() * a2.x() + a1.y() * a3.x(), a1.x() * a3.x() - a1.y() * a3.y() * a2.x(),
                    a2.y() * a3.y());
        return m;
    }
    static float hash(vec2 p) {
        float h = dot(p, vec2(127.1f, 311.7f));
        return VecLib::fract(std::sin(h) * 43758.5453123f);  // TODO
    }
    static float noise(vec2 p) {
        vec2 i = floor(p);
        vec2 f = fract(p);
        vec2 u = f * f * (3.0f - 2.0f * f);                                        // TODO
        return -1.0f + 2.0f * VecLib::mix(VecLib::mix(hash(i + vec2(0.0f, 0.0f)),  // TODO
                                                      hash(i + vec2(1.0f, 0.0f)), u.x()),
                                          VecLib::mix(hash(i + vec2(0.0f, 1.0f)), hash(i + vec2(1.0f, 1.0f)), u.x()),
                                          u.y());
    }

    // lighting
    static float diffuse(vec3 n, vec3 l, float p) { return std::pow(dot(n, l) * 0.4f + 0.6f, p); }
    static float specular(vec3 n, vec3 l, vec3 e, float s) {
        float nrm = (s + 8.0f) / (3.1415f * 8.0f);
        return std::pow(std::max(dot(reflect(e, n), l), 0.0f), s) * nrm;  // TODO
    }

    // sky
    static vec3 getSkyColor(vec3 e) {
        e.y() = std::max(e.y(), 0.0f);  // TODO
        vec3 ret;
        ret.x() = std::pow(1.0f - e.y(), 2.0f);  // TODO
        ret.y() = 1.0f - e.y();
        ret.z() = 0.6f + (1.0f - e.y()) * 0.4f;
        return ret;
    }

    // sea
    static float sea_octave(vec2 uv, float choppy) {
        uv += noise(uv);
        vec2 wv = 1.0f - abs(sin(uv));  // TODO
        vec2 swv = abs(cos(uv));
        wv = mix(wv, swv, wv);
        return std::pow(1.0f - std::pow(wv.x() * wv.y(), 0.65f), choppy);  // TODO
    }

    float map(vec3 p) const {
        const mat2 octave_m = mat2(1.6f, 1.2f, -1.2f, 1.6f);
        const float SEA_TIME = iGlobalTime * SEA_SPEED;
        float freq = SEA_FREQ;
        float amp = SEA_HEIGHT;
        float choppy = SEA_CHOPPY;
        vec2 uv = p.xz();
        uv.x() *= 0.75f;

        float d, h = 0.0f;
        for (int i = 0; i < ITER_GEOMETRY; i++) {
            d = sea_octave((uv + SEA_TIME) * freq, choppy);
            d += sea_octave((uv - SEA_TIME) * freq, choppy);
            h += d * amp;
            uv *= octave_m;
            freq *= 1.9f;
            amp *= 0.22f;
            choppy = VecLib::mix(choppy, 1.0f, 0.2f);  // TODO
        }
        return p.y() - h;
    }

    float map_detailed(vec3 p) const {
        const mat2 octave_m = mat2(1.6f, 1.2f, -1.2f, 1.6);
        const float SEA_TIME = iGlobalTime * SEA_SPEED;
        float freq = SEA_FREQ;
        float amp = SEA_HEIGHT;
        float choppy = SEA_CHOPPY;
        vec2 uv = p.xz();
        uv.x() *= 0.75f;

        float d, h = 0.0f;
        for (int i = 0; i < ITER_FRAGMENT; i++) {
            d = sea_octave((uv + SEA_TIME) * freq, choppy);
            d += sea_octave((uv - SEA_TIME) * freq, choppy);
            h += d * amp;
            uv *= octave_m;
            freq *= 1.9f;
            amp *= 0.22f;
            choppy = VecLib::mix(choppy, 1.0f, 0.2f);
        }
        return p.y() - h;
    }

    static vec3 getSeaColor(vec3 p, vec3 n, vec3 l, vec3 eye, vec3 dist) {
        static const vec3 SEA_BASE = vec3(0.1f, 0.19f, 0.22f);
        static const vec3 SEA_WATER_COLOR = vec3(0.8f, 0.9f, 0.6f);

        float fresnel = 1.0f - std::max(dot(n, -eye), 0.0f);  // TODO
        fresnel = std::pow(fresnel, 3.0f) * 0.65f;            // TODO

        vec3 reflected = getSkyColor(reflect(eye, n));
        vec3 refracted = SEA_BASE + diffuse(n, l, 80.0f) * SEA_WATER_COLOR * 0.12f;

        vec3 color = mix(refracted, reflected, fresnel);

        float atten = std::max(1.0f - dot(dist, dist) * 0.001f, 0.0f);  // TODO
        color += SEA_WATER_COLOR * (p.y() - SEA_HEIGHT) * 0.18f * atten;

        color += vec3(specular(n, l, eye, 60.0f));

        return color;
    }

    // tracing
    vec3 getNormal(vec3 p, float eps) const {
        vec3 n;
        n.y() = map_detailed(p);
        n.x() = map_detailed(vec3(p.x() + eps, p.y(), p.z())) - n.y();
        n.z() = map_detailed(vec3(p.x(), p.y(), p.z() + eps)) - n.y();
        n.y() = eps;
        return normalize(n);
    }

    float heightMapTracing(vec3 ori, vec3 dir, vec3& p) const {  // NOLINT
        float tm = 0.0f;
        float tx = 1000.0f;
        float hx = map(ori + dir * tx);
        if (hx > 0.0f) {
            return tx;
        }
        float hm = map(ori + dir * tm);
        float tmid = 0.0f;
        for (int i = 0; i < NUM_STEPS; i++) {
            tmid = VecLib::mix(tm, tx, hm / (hm - hx));
            p = ori + dir * tmid;
            float hmid = map(p);
            if (hmid < 0.0f) {
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
    void mainImage(vec4& fragColor, const vec2& fragCoord) const override {
        vec2 uv = fragCoord.xy() / iResolution.xy();
        uv = uv * 2.0f - 1.0f;
        uv.x() *= iResolution.x() / iResolution.y();
        float time = iGlobalTime * 0.3f;  // + iMouse.x*0.01;

        // ray
        vec3 ang = vec3(std::sin(time * 3.0f) * 0.1f, std::sin(time) * 0.2f + 0.3f, time);
        vec3 ori = vec3(0.0f, 3.5f, time * 5.0f);
        vec3 dir = normalize(vec3(uv.xy(), -2.0f));
        dir.z() += length(uv) * 0.15f;
        dir = normalize(dir) * fromEuler(ang);

        // tracing
        vec3 p;
        heightMapTracing(ori, dir, p);
        vec3 dist = p - ori;
        vec3 n = getNormal(p, dot(dist, dist) * EPSILON_NRM);
        vec3 light = normalize(vec3(0.0f, 1.0f, 0.8));

        // color
        vec3 color = mix(getSkyColor(dir), getSeaColor(p, n, light, dir, dist),
                         std::pow(smoothstep(0.0f, -0.05f, dir.y()), 0.3f));

        // post
        fragColor = vec4(pow(color, vec3(0.75f)), 1.0f);
    }
};
#if 0
class NormalMapVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        NORMAL_INDEX = 1,
        TEXTURE_INDEX = 2,
        TANGENT_INDEX = 3,
    };

    VecLib::Matrix4f mProjMatrix;
    VecLib::Matrix4f mModelViewMatrix;
    VecLib::Vector3f mLightPos;

public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector3f, VecLib::Vector2f, VecLib::Vector4f> InType;
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f, float, VecLib::Vector3f, VecLib::Vector3f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2,
        EYE_DIR_INDEX = 3,
        LIGHT_DIR_INDEX = 4
    };

    NormalMapVertexShader(const VecLib::Vector3f& lightPos) {
        mLightPos = lightPos;
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector3f& normal = std::get<static_cast<int>(InTraits::NORMAL_INDEX)>(in);
        const VecLib::Vector4f& tangent = std::get<static_cast<int>(InTraits::TANGENT_INDEX)>(in);
        const VecLib::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);

        //assert(tangent.w() == 1.0f);
        //printf("%f\n", tangent.w());

        VecLib::Vector4f P = mModelViewMatrix * pos;

        VecLib::Vector3f N = normalize(VecLib::Matrix3x3f(mModelViewMatrix) * normal);
        VecLib::Vector3f T = normalize(VecLib::Matrix3x3f(mModelViewMatrix) * tangent.xyz());
        VecLib::Vector3f B = cross(N, T) * tangent.w();

        VecLib::Vector3f L = mLightPos - P.xyz();
        VecLib::Vector3f outLightDir = normalize(VecLib::Vector3f(L.dot(T), L.dot(B), L.dot(N)));

        VecLib::Vector3f V = -P.xyz();
        VecLib::Vector3f outEyeDir = normalize(VecLib::Vector3f(V.dot(T), V.dot(B), V.dot(N)));

        const VecLib::Vector4f outPos = mProjMatrix * P;

        const float invDepth = 1.0 / P[2];

        //std::cout << "Eye: " << outEyeDir << std::endl;
        //std::cout << "Light: " << outLightDir << std::endl;

        return std::make_tuple(outPos, tex * invDepth, invDepth, outEyeDir, outLightDir);
    }

    VecLib::Matrix4f& projMatrix() { return mProjMatrix; }
    VecLib::Matrix4f& modelViewMatrix() { return mModelViewMatrix; }
};

class NormalMapFragmentShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
        TEXTURE_INDEX = 1,
        INV_DEPTH_INDEX = 2,
        EYE_DIR_INDEX = 3,
        LIGHT_DIR_INDEX = 4
    };

    RGBATextureSampler<VecLib::Vector4f> mNormalSampler;

public:
    typedef std::tuple<VecLib::Vector4f, VecLib::Vector2f, float, VecLib::Vector3f, VecLib::Vector3f> InType;
    typedef std::tuple<VecLib::Vector4f, float> OutType;

    enum class Traits {
        COLOR_INDEX = 0,
        DEPTH_INDEX = 1
    };

    NormalMapFragmentShader(const std::string& filename) {
        cimg_library::CImg<unsigned char> texImg(filename.c_str());

        mNormalSampler = RGBATextureSampler<VecLib::Vector4f>(texImg);
    }

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos   = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const float invDepth = std::get<static_cast<int>(InTraits::INV_DEPTH_INDEX)>(in);
        const auto eyeDir = std::get<static_cast<int>(InTraits::EYE_DIR_INDEX)>(in);
        const auto lightDir = std::get<static_cast<int>(InTraits::LIGHT_DIR_INDEX)>(in);

        const auto realTex = tex / invDepth;

        //auto color = mUniform.textureSampler.get(realTex[0], realTex[1]);

        VecLib::Vector3f V = normalize(eyeDir);
        VecLib::Vector3f L = normalize(lightDir);
        VecLib::Vector3f N = normalize(mNormalSampler.get(realTex.x(), realTex.y()).xyz() * 2.0f - 1.0f);

        VecLib::Vector3f R = reflect(-L, N);

        //std::cout << "R: " << R << std::endl;
        //std::cout << "V: " << V << std::endl;

        const float intensity = std::max(0.0f, R.dot(V));
        //printf("%f\n", intensity);

        const VecLib::Vector4f color(1.0f, 1.0f, 1.0f, 1.0f);

        return std::make_tuple(color*intensity, pos[2]);
    }
};
#endif
class CImgColorRasterShader {
private:
    enum class InTraits { COLOR_INDEX = 0, DEPTH_INDEX = 1 };

    PixelBuffer<unsigned char>& mFrame;
    PixelBuffer<float>& mDepth;

public:
    using InType = std::tuple<VecLib::Vector4f, float>;

    CImgColorRasterShader(PixelBuffer<unsigned char>& frame, PixelBuffer<float>& depth)
        : mFrame(frame), mDepth(depth) {}

    void operator()(const InType& fragment, unsigned int x, unsigned int y) const {
        constexpr auto ColorAttachment = static_cast<int>(InTraits::COLOR_INDEX);
        constexpr auto DepthAttachment = static_cast<int>(InTraits::DEPTH_INDEX);

        const auto& depth = std::get<DepthAttachment>(fragment);
        const auto& color = std::get<ColorAttachment>(fragment);

        if (depth > 0.0f && depth < 1.0f && depth < mDepth(x, y)) {
            mFrame(x, y, 0) = color[0] <= 1.0f ? color[0] * 255.0f : 255;
            mFrame(x, y, 1) = color[1] <= 1.0f ? color[1] * 255.0f : 255;
            mFrame(x, y, 2) = color[2] <= 1.0f ? color[2] * 255.0f : 255;

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
#if 0
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
#endif
#endif /* SHADERS_H_ */
