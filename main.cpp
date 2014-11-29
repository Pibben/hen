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
#include "stdcomp/shaders.h"
#include "utils.h"


template <class In>
std::vector<In> loadMeshColor(const std::string& filename, const Eigen::Vector4f& color) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector2f> uvs;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, vertices, uvs, normals, faces);
    printf("Loaded %lu faces\n", faces.size());

    std::vector<In> m;

    for(int j = 0; j < faces.size(); ++j) {
        const auto& f = faces[j];
        for(int i = 0; i < 3; ++i) {
            Eigen::Vector4f v = Eigen::Vector4f::Ones();
            v.topRows<3>() = vertices[f.coords[i]];
            m.push_back({v, color});
        }
    }

    return m;
}

template <class In>
std::vector<In> loadMesh(const std::string& filename) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector2f> uvs;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, vertices, uvs, normals, faces);
    printf("Loaded %lu faces\n", faces.size());

    std::vector<In> m;

    for(int j = 0; j < faces.size(); ++j) {
        const auto& f = faces[j];
        for(int i = 0; i < 3; ++i) {
            Eigen::Vector4f v = Eigen::Vector4f::Ones();
            v.topRows<3>() = vertices[f.coords[i]];
            m.push_back({v, uvs[f.uvs[i]]});
        }
    }

    return m;
}

template <class In>
std::vector<In> loadMeshNormal(const std::string& filename) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector2f> uvs;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, vertices, uvs, normals, faces);
    printf("Loaded %lu faces\n", faces.size());

    std::vector<In> m;

    for(int j = 0; j < faces.size(); ++j) {
        const auto& f = faces[j];

        Eigen::Vector3f vs[3];

        for(int i = 0; i < 3; ++i) {
            vs[i] = vertices[f.coords[i]];
        }

        for(int i = 0; i < 3; ++i) {
#if 0
            const Eigen::Vector3f norm = (vs[1]-vs[0]).cross(vs[2]-vs[0]);
#else
            const Eigen::Vector3f norm = normals[f.coords[i]];
#endif
            Eigen::Vector4f v = Eigen::Vector4f::Ones();
            v.topRows<3>() = vs[i];
            m.push_back({v, norm});
        }
    }

    return m;
}

template <class Mesh, class VertexShader, class FragmentShader>
void animate(const Mesh& mesh, VertexShader& vertexShader, FragmentShader& fragmentShader) {
    vertexShader.uniform().projMatrix = proj(-5, 5, -5, 5, 5, 30);

    vertexShader.uniform().modelViewMatrix = Eigen::Matrix4f::Identity();

    Eigen::Affine3f transform(Eigen::Translation3f(0,0,-10));
    vertexShader.uniform().modelViewMatrix = transform.matrix();

    const int width = 640*2;
    const int height = 480*2;

    //Renderer
    Renderer<typename FragmentShader::OutType, typename FragmentShader::Traits, width, height> renderer;

    cimg_library::CImgDisplay disp;
    cimg_library::CImg<unsigned char> img(width, height, 1, 3);
    //cimg_library::CImg<float> depth(640, 480);

    while(true) {
        Eigen::AngleAxisf aa((1/180.0)*M_PI, Eigen::Vector3f::UnitY());
        Eigen::Matrix4f rotMatrix = Eigen::Matrix4f::Identity();
        rotMatrix.topLeftCorner<3,3>() = aa.matrix();
        vertexShader.uniform().modelViewMatrix *= rotMatrix;

        renderer.template render<typename Mesh::value_type, VertexShader, FragmentShader>(mesh, vertexShader, fragmentShader);

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

void textureAnimation() {
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

    typedef TextureVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/models/cow/colorOpacityCow.png");
    //cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/models/cow/colorOpacityCowAO.png");
    struct MyFragUniformType {
        TextureSampler<Eigen::Vector4f> textureSampler;
    };
    MyFragUniformType fragUniform = {TextureSampler<Eigen::Vector4f>(texImg)};

    typedef TextureFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    auto m = loadMesh<MyVertInType>("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void colorAnimation(const Eigen::Vector4f& color) {
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

    typedef ColorVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    struct MyFragUniformType {} fragUniform;

    typedef ColorFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    auto m = loadMeshColor<MyVertInType>("models/sphere2.obj", color);

    animate(m, vertexShader, fragmentShader);
}

void multiTextureAnimation() {
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

    typedef TextureVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/models/cow/colorOpacityCow.png");
    cimg_library::CImg<unsigned char> aoImg("/home/per/code/hen/models/cow/colorOpacityCowAO.png");

    typedef TextureSampler<Eigen::Vector4f> SamplerType;

    struct MyFragUniformType {
        SamplerType textureSampler;
        SamplerType aoSampler;
    };

    MyFragUniformType fragUniform = {SamplerType(texImg), SamplerType(aoImg)};

    typedef MultiTextureFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    auto m = loadMesh<MyVertInType>("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void flatShading() {
    //Input types
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> MyVertInType;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { NORMAL_ATTACHMENT = 1 };
    };

    //Vertex shader
    struct MyVertUniformType {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
        Eigen::Vector3f lightPos;
    } vertUniform;

    vertUniform.lightPos = Eigen::Vector3f(100,100,100);

    typedef FlatVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    struct MyFragUniformType {} fragUniform;

    typedef ColorFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    auto m = loadMeshNormal<MyVertInType>("models/cow/cowTM08New00RTime02-tri.obj");

    animate(m, vertexShader, fragmentShader);
}

void textureTest() {
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

    typedef TextureVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/checkerboard.jpg");
    //cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/models/cow/colorOpacityCowAO.png");
    struct MyFragUniformType {
        TextureSampler<Eigen::Vector4f> textureSampler;
    };
    MyFragUniformType fragUniform = {TextureSampler<Eigen::Vector4f>(texImg)};

    typedef TextureFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    //auto m = loadMesh<MyVertInType>("models/cow/cowTM08New00RTime02-tri-norm.obj");
    std::vector<MyVertInType> mesh;

    static const float size = 3.5;

    Eigen::Vector4f corners[] = {{size, 0, size, 1}, {-size, 0, size, 1}, {-size, 0, -size, 1}, {size, 0, -size, 1}};
    Eigen::Vector2f uvs[] = {{1, 1}, {0, 1}, {0, 0}, {1, 0}};

    mesh.emplace_back(corners[0], uvs[0]);
    mesh.emplace_back(corners[1], uvs[1]);
    mesh.emplace_back(corners[2], uvs[2]);

    mesh.emplace_back(corners[0], uvs[0]);
    mesh.emplace_back(corners[2], uvs[2]);
    mesh.emplace_back(corners[3], uvs[3]);

    animate(mesh, vertexShader, fragmentShader);
}

void phongShading() {
    //Input types
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> MyVertInType;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { NORMAL_ATTACHMENT = 1 };
    };

    //Vertex shader
    struct MyVertUniformType {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
        Eigen::Vector3f lightPos;
    } vertUniform;
    vertUniform.lightPos = Eigen::Vector3f(100,100,100);

    typedef PhongVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    struct MyFragUniformType {
    } fragUniform;

    typedef PhongFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    auto m = loadMeshNormal<MyVertInType>("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void equiRectangular() {
    //Input types
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> MyVertInType;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { NORMAL_ATTACHMENT = 1 };
    };

    //Vertex shader
    struct MyVertUniformType {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    } vertUniform;

    typedef NormalViewVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/equirect.jpg");
    //cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/models/cow/colorOpacityCowAO.png");
    struct MyFragUniformType {
        TextureSampler<Eigen::Vector4f> textureSampler;
    };
    MyFragUniformType fragUniform = {TextureSampler<Eigen::Vector4f>(texImg)};


    typedef EquiRectFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    auto m = loadMeshNormal<MyVertInType>("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

int main(int argc, char** argv) {
    equiRectangular();
}



