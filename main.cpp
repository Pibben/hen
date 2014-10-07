/*
 * main.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: per
 */

#include "hen.h"
#include "utils.h"

template <class In, class InTraits, class Uniform>
class ColorVertexShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int COLOR_ATTACHMENT = 1;
    };

    ColorVertexShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector4f& color = std::get<InTraits::COLOR_ATTACHMENT>(in);

        Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos, color);
    }
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

        return std::make_tuple(color, pos[2]);
    }
};

template <class In, class InTraits, class Uniform>
class TextureVertexShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int TEXTURE_ATTACHMENT = 1;
    };

    TextureVertexShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector2f& tex = std::get<InTraits::TEXTURE_ATTACHMENT>(in);

        Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos, tex);
    }
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

        auto color = mUniform.textureSampler.get(tex[0], tex[1]);

        return std::make_tuple(color, pos[2]);
    }
};

int main(int argc, char** argv) {
    //Input types
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> MyVertInType;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { TEXTURE_ATTACHMENT = 1 };
    };

    //Vertex shader
    struct MyVertUniformType {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    } vertUniform;

    vertUniform.projMatrix = Eigen::Matrix4f::Identity();
    vertUniform.modelViewMatrix = proj(-250, 250, -250, 250, 12, 50);

    typedef TextureVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    //Fragment shader
    cimg_library::CImg<unsigned char> texImg("/home/per/gradient.jpg");
    struct MyFragUniformType {
        TextureSampler<Eigen::Vector4f> textureSampler;
        MyFragUniformType(cimg_library::CImg<unsigned char>& img) : textureSampler(img) {}
    } fragUniform(texImg);

    typedef TextureFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    //Renderer
    Renderer<typename MyFragmentShaderType::OutType, typename MyFragmentShaderType::Traits> renderer;

    //Draw
    std::vector<Eigen::Vector4f> vertices = {Eigen::Vector4f(-100, -100, 12, 1),
                                             Eigen::Vector4f(200, -100, 12, 1),
                                             Eigen::Vector4f(-100, 200, 20, 1),
                                             Eigen::Vector4f(200, 200, 20, 1)};

    std::vector<Eigen::Vector4f> colors = {Eigen::Vector4f(255,0,0,1), //Red
                                           Eigen::Vector4f(0,255,0,1), //Green
                                           Eigen::Vector4f(0,0,255,1), //Blue
                                           Eigen::Vector4f(255,0,255,1)}; //Purple

    std::vector<Eigen::Vector2f> uvs = {Eigen::Vector2f(0,0),
                                        Eigen::Vector2f(1,0),
                                        Eigen::Vector2f(0,1),
                                        Eigen::Vector2f(1,1)};

    renderer.render<MyVertInType, MyVertexShaderType, MyFragmentShaderType>({{vertices[0], uvs[0]},
                                                                             {vertices[1], uvs[1]},
                                                                             {vertices[2], uvs[2]},

                                                                             {vertices[1], uvs[1]},
                                                                             {vertices[2], uvs[2]},
                                                                             {vertices[3], uvs[3]}},
                                                                             vertexShader, fragmentShader);
}


