/*
 * main.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: per
 */

#include "hen.h"
#include "utils.h"

template <class In, class InTraits, class Uniform>
class MyVertexShader {
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

    MyVertexShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector4f& color = std::get<InTraits::COLOR_ATTACHMENT>(in);

        Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos, color);
    }
};

template <class In, class InTraits, class Uniform>
class MyFragmentShader {
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

    MyFragmentShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector4f& color = std::get<InTraits::COLOR_ATTACHMENT>(in);

        return std::make_tuple(color, pos[2]);
    }
};

int main(int argc, char** argv) {
    //Input types
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> MyVertInType;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { COLOR_ATTACHMENT = 1 };
    };

    //Vertex shader
    struct MyVertUniformType {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    } vertUniform;

    vertUniform.projMatrix = Eigen::Matrix4f::Identity();
    vertUniform.modelViewMatrix = proj(-250, 250, -250, 250, 10, 50);

    typedef MyVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    //Fragment shader
    struct MyFragUniformType {
    } fragUniform;

    typedef MyFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    //Renderer
    Renderer<MyVertexShaderType, MyFragmentShaderType> renderer(vertexShader, fragmentShader);

    //Draw
    std::vector<Eigen::Vector4f> vertices = {Eigen::Vector4f(  0,   0, 12, 1),
                                             Eigen::Vector4f(200,   0, 12, 1),
                                             Eigen::Vector4f(  0, 200, 20, 1),
                                             Eigen::Vector4f(200, 200, 20, 1)};

    std::vector<Eigen::Vector4f> colors = {Eigen::Vector4f(255,0,0,1), //Red
                                           Eigen::Vector4f(0,255,0,1), //Green
                                           Eigen::Vector4f(0,0,255,1), //Blue
                                           Eigen::Vector4f(255,0,255,1)}; //Purple

    renderer.render<MyVertInType>({{vertices[0], colors[0]},
                                   {vertices[1], colors[1]},
                                   {vertices[2], colors[2]},

                                   {vertices[1], colors[1]},
                                   {vertices[2], colors[2]},
                                   {vertices[3], colors[3]}});
}


