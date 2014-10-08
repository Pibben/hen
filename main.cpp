/*
 * main.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: per
 */
#include <chrono>
#include <thread>

#include <Eigen/Dense>

#include "hen.h"
#include "io.h"
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
        assert(pos[2] < 0.0);

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
        //auto color = Eigen::Vector4f(255, 0, 255, 255);

        return std::make_tuple(color, pos[2]);
    }
};
#if 0
static void textureTest() {
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

    vertUniform.projMatrix = proj(-250, 250, -250, 250, 12, 50);
    vertUniform.modelViewMatrix = Eigen::Matrix4f::Identity();

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
    Renderer<typename MyFragmentShaderType::OutType, typename MyFragmentShaderType::Traits, 640, 480> renderer;

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

    for(int i = 0; i < 360; i+=5) {
    //int i = 95;
        printf("%d\n", i);

        Eigen::AngleAxisf aa((i/180.0)*M_PI, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotMatrix = aa.matrix();
        vertUniform.modelViewMatrix.block<3,3>(0,0) = rotMatrix;

        renderer.render<MyVertInType, MyVertexShaderType, MyFragmentShaderType>({{vertices[0], uvs[0]},
                                                                                 {vertices[1], uvs[1]},
                                                                                 {vertices[2], uvs[2]},

                                                                                 {vertices[1], uvs[1]},
                                                                                 {vertices[2], uvs[2]},
                                                                                 {vertices[3], uvs[3]}},
                                                                                 vertexShader, fragmentShader);

        //std::this_thread::sleep_for (std::chrono::seconds(1));
    }
}

void wireframe() {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector2f> uvs;
    std::vector<Face> faces;

    loadObj("models/cow/cowTM08New00RTime02-tri.obj", vertices, uvs, faces);
    //loadObj("models/box.obj", vertices, uvs, faces);
    printf("Loaded %d faces\n", faces.size());

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

    vertUniform.projMatrix = proj(-5, 5, -5, 5, 5, 30);

    vertUniform.modelViewMatrix = Eigen::Matrix4f::Identity();

    Eigen::AngleAxisf aa((180/180.0)*M_PI, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf bb((90/180.0)*M_PI, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f rotMatrix = Eigen::Matrix4f::Identity();
    rotMatrix.block<3,3>(0,0) = aa.matrix()*bb.matrix();

    Eigen::Affine3f transform(Eigen::Translation3f(0,0,-5));
    vertUniform.modelViewMatrix = transform.matrix()*rotMatrix;


    typedef ColorVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    struct MyFragUniformType {} fragUniform;

    typedef ColorFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    //Renderer
    Renderer<typename MyFragmentShaderType::OutType, typename MyFragmentShaderType::Traits, 640, 480> renderer;

    std::vector<MyVertInType> m;

    for(int j = 0; j < faces.size(); ++j) {
        const auto& f = faces[j];
        for(int i = 0; i < 3; ++i) {
            Eigen::Vector4f v = Eigen::Vector4f::Ones();
            v.block<3,1>(0,0) = vertices[f.first[i]];
            //printf("%d (%f %f %f) ", f.first[i], v[0], v[1], v[2]);
            m.push_back({v, Eigen::Vector4f(255,255,255,255)});
        }
        //printf("\n");
        //break;
    }

    renderer.render<MyVertInType, MyVertexShaderType, MyFragmentShaderType>(m, vertexShader, fragmentShader);
}

#endif

int main(int argc, char** argv) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector2f> uvs;
    std::vector<Face> faces;

    loadObj("models/cow/cowTM08New00RTime02-tri.obj", vertices, uvs, faces);
    //loadObj("models/box.obj", vertices, uvs, faces);
    printf("Loaded %lu faces\n", faces.size());

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

    vertUniform.projMatrix = proj(-5, 5, -5, 5, 5, 30);

    vertUniform.modelViewMatrix = Eigen::Matrix4f::Identity();

    Eigen::Affine3f transform(Eigen::Translation3f(0,0,-10));
    vertUniform.modelViewMatrix = transform.matrix();


    typedef TextureVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/models/cow/colorOpacityCow.png");
    struct MyFragUniformType {
        TextureSampler<Eigen::Vector4f> textureSampler;
        MyFragUniformType(cimg_library::CImg<unsigned char>& img) : textureSampler(img) {}
    } fragUniform(texImg);

    typedef TextureFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    //Renderer
    Renderer<typename MyFragmentShaderType::OutType, typename MyFragmentShaderType::Traits, 640, 480> renderer;

    std::vector<MyVertInType> m;

    for(int j = 0; j < faces.size(); ++j) {
        const auto& f = faces[j];
        for(int i = 0; i < 3; ++i) {
            Eigen::Vector4f v = Eigen::Vector4f::Ones();
            v.block<3,1>(0,0) = vertices[f.first[i]];
            //printf("%d (%f %f %f) ", f.first[i], v[0], v[1], v[2]);
            m.push_back({v, uvs[f.second[i]]});
        }
        //printf("\n");
        //break;
    }


    cimg_library::CImgDisplay disp;
    cimg_library::CImg<unsigned char> img(640, 480, 1, 3);
    //cimg_library::CImg<float> depth(640, 480);

    while(true) {
        Eigen::AngleAxisf aa((1/180.0)*M_PI, Eigen::Vector3f::UnitY());
        Eigen::Matrix4f rotMatrix = Eigen::Matrix4f::Identity();
        rotMatrix.block<3,3>(0,0) = aa.matrix();
        vertUniform.modelViewMatrix *= rotMatrix;

        renderer.render<MyVertInType, MyVertexShaderType, MyFragmentShaderType>(m, vertexShader, fragmentShader);

        renderer.readback(img);
        disp.display(img);

        //renderer.readbackDepth(depth);
        //disp.display(normalizeDepth(depth));

        renderer.clear();
        //disp.wait(100);
        if(disp.is_closed()) {
            break;
        }
    }
}



