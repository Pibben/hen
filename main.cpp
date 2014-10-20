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
        assert(pos[2] < 0.0);

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

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int TEXTURE_ATTACHMENT = 1;
    };

    TextureVertexShader(Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector2f& tex = std::get<InTraits::TEXTURE_ATTACHMENT>(in);

        Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos, tex);
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

        auto color = mUniform.textureSampler.get(tex[0], tex[1]);
        //auto color = Eigen::Vector4f(255, 0, 255, 255);

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

static Eigen::Vector3f reflect(const Eigen::Vector3f& R, const Eigen::Vector3f& N) {
    Eigen::Vector3f Rout = R - 2.0*N.dot(R)*N;

    //std::cout << R << ", " << N << " --> " << Rout << std::endl;

    return Rout;
}

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

        Eigen::Vector3f N = mUniform.modelViewMatrix.template block<3,3>(0,0) * normal;
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
            v.block<3,1>(0,0) = vertices[f.coords[i]];
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
            v.block<3,1>(0,0) = vertices[f.coords[i]];
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

        const Eigen::Vector3f norm = (vs[1]-vs[0]).cross(vs[2]-vs[0]);

        for(int i = 0; i < 3; ++i) {
            Eigen::Vector4f v = Eigen::Vector4f::Ones();
            v.block<3,1>(0,0) = vs[i];
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


    //Renderer
    Renderer<typename FragmentShader::OutType, typename FragmentShader::Traits, 640, 480> renderer;

    cimg_library::CImgDisplay disp;
    cimg_library::CImg<unsigned char> img(640, 480, 1, 3);
    //cimg_library::CImg<float> depth(640, 480);

    while(true) {
        Eigen::AngleAxisf aa((1/180.0)*M_PI, Eigen::Vector3f::UnitY());
        Eigen::Matrix4f rotMatrix = Eigen::Matrix4f::Identity();
        rotMatrix.block<3,3>(0,0) = aa.matrix();
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
        MyFragUniformType(cimg_library::CImg<unsigned char>& img) : textureSampler(img) {}
    } fragUniform(texImg);

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
    struct MyFragUniformType {
        TextureSampler<Eigen::Vector4f> textureSampler;
        TextureSampler<Eigen::Vector4f> aoSampler;
        MyFragUniformType(cimg_library::CImg<unsigned char>& img, cimg_library::CImg<unsigned char>& ao) : textureSampler(img), aoSampler(ao) {}
    } fragUniform(texImg, aoImg);

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

int main(int argc, char** argv) {
    flatShading();
}



