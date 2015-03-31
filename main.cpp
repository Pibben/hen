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

#include "CImg.h"

template <class OutType>
class TextureSampler {
    cimg_library::CImg<unsigned char>& mImg;
    unsigned int mSizeX;
    unsigned int mSizeY;

public:
    TextureSampler(cimg_library::CImg<unsigned char>& img) : mImg(img), mSizeX(img.width()), mSizeY(img.height()) {}

    OutType get(float u, float v) const {
        int x = u * (mSizeX-1) + 0.5;
        int y = mSizeY - (v * (mSizeY-1) + 0.5);

        assert(x >= 0 && x < mSizeX);
        assert(y >= 0 && y < mSizeY);

        const float r = mImg(x, y, 0);
        const float g = mImg(x, y, 1);
        const float b = mImg(x, y, 2);
        const float a = 255.0;

        return OutType(r, g, b, a);
    }
};



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

template <class PixelType>
class Visitor {
private:
    cimg_library::CImg<unsigned char>& mImg;
    int mCount;
public:
    Visitor(cimg_library::CImg<unsigned char>& img) : mImg(img), mCount(0) {}

    void operator()(const PixelType& p) {
        for(int j = 0; j < 3; ++j) {
            mImg(mCount % mImg.width(), mImg.height() - 1 - mCount/mImg.width(), j) = p[j];
        }
        mCount++;
    }
};

template <class PixelType>
class DepthVisitor {
private:
    cimg_library::CImg<float>& mImg;
    int mCount;
public:
    DepthVisitor(cimg_library::CImg<float>& img) : mImg(img), mCount(0) {}

    void operator()(const PixelType& p) {
        mImg(mCount % mImg.width(), mImg.height() - 1 - mCount/mImg.width()) = p;
        mCount++;
    }
};

template <class Mesh, class VertexShader, class FragmentShader>
void animate(const Mesh& mesh, VertexShader& vertexShader, FragmentShader& fragmentShader) {
    vertexShader.uniform().projMatrix = proj(-5, 5, -5, 5, 5, 30);

    vertexShader.uniform().modelViewMatrix = Eigen::Matrix4f::Identity();
    vertexShader.uniform().modelViewMatrix = lookAt({0, 7, 7}, {0, 0, 0}, {0, 1, 0}); //TODO: Should be in projMatrix?

    const int width = 640*2;
    const int height = 480*2;

    //Renderer
    typedef Renderer<typename FragmentShader::OutType, typename FragmentShader::Traits, width, height> RendererType;
    RendererType renderer;

    cimg_library::CImgDisplay disp;
    cimg_library::CImg<unsigned char> img(width, height, 1, 3);
    //cimg_library::CImg<float> depth(640, 480);

    Eigen::AngleAxisf aa((1/180.0)*M_PI, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f rotMatrix = Eigen::Matrix4f::Identity();
    rotMatrix.topLeftCorner<3,3>() = aa.matrix();
    while(true) {
        vertexShader.uniform().modelViewMatrix *= rotMatrix;

        renderer.template render<typename Mesh::value_type>(mesh, vertexShader, fragmentShader);

        Visitor<typename RendererType::DataType> visitor(img);
        renderer.visitFramebuffer(visitor);

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

template <class Mesh, class VertexShader, class FragmentShader>
void animateDepth(const Mesh& mesh, VertexShader& vertexShader, FragmentShader& fragmentShader) {
    vertexShader.uniform().projMatrix = proj(-5, 5, -5, 5, 5, 30);

    vertexShader.uniform().modelViewMatrix = Eigen::Matrix4f::Identity();
    vertexShader.uniform().modelViewMatrix = lookAt({0, 7, 7}, {0, 0, 0}, {0, 1, 0}); //TODO: Should be in projMatrix?

    const int width = 640*2;
    const int height = 480*2;

    //Renderer
    typedef Renderer<typename FragmentShader::OutType, typename FragmentShader::Traits, width, height> RendererType;
    RendererType renderer;

    cimg_library::CImgDisplay disp;
    cimg_library::CImg<float> depth(width, height);

    Eigen::AngleAxisf aa((1/180.0)*M_PI, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f rotMatrix = Eigen::Matrix4f::Identity();
    rotMatrix.topLeftCorner<3,3>() = aa.matrix();
    while(true) {
        vertexShader.uniform().modelViewMatrix *= rotMatrix;

        //Render depth
        renderer.template render<typename Mesh::value_type>(mesh, vertexShader, fragmentShader);

        DepthVisitor<typename RendererType::DepthType> visitor(depth);
        renderer.visitDepthbuffer(visitor);

        //disp.display(img);

        //renderer.readbackDepth(depth);
        disp.display(normalizeDepth(depth));

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
#if 0
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
#endif

void cubeMap() {
	typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> MyVertInType; //TODO:

	NormalViewVertexShader vertexShader;
	CubemapFragmentShader fragmentShader;

	cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/cubemap.jpg");
	fragmentShader.setCubeSampler(CubeSampler<Eigen::Vector4f>(texImg));

    auto m = loadMeshNormal<MyVertInType>("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void shadow() {
    //Input types
    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> MyVertInType;

    struct InTraits {
        enum { POSITION_ATTACHMENT = 0 };
        enum { TEXTURE_ATTACHMENT = 1 };
    };

    const auto lightPos = Eigen::Vector3f(100,100,100);

    //Vertex shader
    struct ShadowVertUniformType {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
    } shadowVertUniform;

    shadowVertUniform.projMatrix = proj(-1, 1, -1, 1, 1, 1000);
    shadowVertUniform.modelViewMatrix = lookAt(lightPos, {0, 0, 0}, {0, 1, 0});

    typedef ShadowGenVertexShader<MyVertInType, InTraits, ShadowVertUniformType> ShadowVertexShaderType;
    ShadowVertexShaderType shadowVertexShader(shadowVertUniform);

    struct ShadowFragUniformType {} shadowFragUniform;

    typedef ShadowGenFragmentShader<typename ShadowVertexShaderType::OutType, typename ShadowVertexShaderType::Traits, ShadowFragUniformType> ShadowFragmentShaderType;
    ShadowFragmentShaderType shadowFragmentShader(shadowFragUniform);
#if 0

    //Vertex shader
    struct MyVertUniformType {
        Eigen::Matrix4f projMatrix;
        Eigen::Matrix4f modelViewMatrix;
        Eigen::Matrix4f shadowMatrix;
    } vertUniform;

    typedef TextureVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/models/cow/colorOpacityCow.png");
    //cimg_library::CImg<unsigned char> texImg("/home/per/code/hen/models/cow/colorOpacityCowAO.png");
    struct MyFragUniformType {
        TextureSampler<Eigen::Vector4f> textureSampler;
        TextureSampler<float> shadowSampler;
    };
    MyFragUniformType fragUniform = {TextureSampler<Eigen::Vector4f>(texImg)};

    typedef TextureFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);
#endif
    auto m = loadMesh<MyVertInType>("models/cow/cowTM08New00RTime02-tri-norm.obj");

    //animate(m, vertexShader, fragmentShader);
    animateDepth(m, shadowVertexShader, shadowFragmentShader);
}

int main(int argc, char** argv) {
    shadow();
}



