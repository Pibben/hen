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


typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> PositionAndColor;
std::vector<PositionAndColor> loadMeshColor(const std::string& filename, const Eigen::Vector4f& color) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector2f> uvs;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, vertices, uvs, normals, faces);
    printf("Loaded %lu faces\n", faces.size());

    std::vector<PositionAndColor> m;

    for(size_t j = 0; j < faces.size(); ++j) {
        const auto& f = faces[j];
        for(int i = 0; i < 3; ++i) {
            Eigen::Vector4f v = Eigen::Vector4f::Ones();
            v.topRows<3>() = vertices[f.coords[i]];
            m.emplace_back(v, color);
        }
    }

    return m;
}

typedef std::tuple<Eigen::Vector4f, Eigen::Vector2f> PositionAndUv;
std::vector<PositionAndUv> loadMeshUv(const std::string &filename) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector2f> uvs;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, vertices, uvs, normals, faces);
    printf("Loaded %lu faces\n", faces.size());

    std::vector<PositionAndUv> m;

    for(size_t j = 0; j < faces.size(); ++j) {
        const auto& f = faces[j];
        for(int i = 0; i < 3; ++i) {
            Eigen::Vector4f v = Eigen::Vector4f::Ones();
            v.topRows<3>() = vertices[f.coords[i]];
            m.emplace_back(v, uvs[f.uvs[i]]);
        }
    }

    return m;
}

typedef std::tuple<Eigen::Vector4f, Eigen::Vector3f> PositionAndNormal;
static std::vector<PositionAndNormal> loadMeshNormal(const std::string& filename) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector2f> uvs;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, vertices, uvs, normals, faces);
    printf("Loaded %lu faces\n", faces.size());

    std::vector<PositionAndNormal> m;

    for(size_t j = 0; j < faces.size(); ++j) {
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
            m.emplace_back(v, norm);
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
    vertexShader.projMatrix() = proj(-5, 5, -5, 5, 5, 30);

    vertexShader.modelViewMatrix() = lookAt({0, 7, 7}, {0, 0, 0}, {0, 1, 0}); //TODO: Should be in projMatrix?

    const int width = 640*2;
    const int height = 480*2;

    //Renderer
    Renderer renderer;

    cimg_library::CImgDisplay disp;
    cimg_library::CImg<unsigned char> img(width, height, 1, 3);
    cimg_library::CImg<float> depth(width, height);

    Eigen::AngleAxisf aa((1/180.0)*M_PI, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f rotMatrix = Eigen::Matrix4f::Identity();
    rotMatrix.topLeftCorner<3,3>() = aa.matrix();

    CImgColorRasterShader rasterShader(img, depth);

    Timer<1> t;

    while(true) {
        vertexShader.modelViewMatrix() *= rotMatrix;

        t.reset();
        renderer.render<typename Mesh::value_type>(mesh, vertexShader, fragmentShader, rasterShader);
        float us = t.getUS();
        printf("%2.2f fps\n", 1.0 / (us / 1000.0 / 1000.0));

        img.mirror('y'); //TODO: Investigate
        disp.display(img);

        if(disp.is_closed()) {
            break;
        }
    }
}


template <class Mesh, class VertexGenShader, class FragmentGenShader,
                      class VertexShader, class FragmentShader>
void animateShadow(const Mesh& mesh, VertexGenShader& vertexGenShader, FragmentGenShader& fragmentGenShader,
                                     VertexShader& vertexShader, FragmentShader& fragmentShader) {
    vertexShader.projMatrix() = proj(-5, 5, -5, 5, 5, 30);

    vertexShader.modelViewMatrix() = lookAt({0, 7, 7}, {0, 0, 0}, {0, 1, 0}); //TODO: Should be in projMatrix?

    const int width = 640*2;
    const int height = 480*2;

    //Renderers
    Renderer renderer;

    cimg_library::CImgDisplay disp;
    cimg_library::CImg<unsigned char> img(width, height, 1, 3);
    cimg_library::CImg<float> depth(width, height);

    Eigen::AngleAxisf aa((1/180.0)*M_PI, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f rotMatrix = Eigen::Matrix4f::Identity();
    rotMatrix.topLeftCorner<3,3>() = aa.matrix();

    auto& depthTexture = fragmentShader.getDepthTexture();

    CImgDepthRasterShader shadowRasteShader(depthTexture);
    CImgColorRasterShader rasterShader(img, depth);

    while(true) {
        vertexShader.modelViewMatrix() *= rotMatrix;

        //Render shadow depth
        renderer.render<typename Mesh::value_type>(mesh, vertexGenShader, fragmentGenShader, shadowRasteShader);

        depthTexture.mirror('y'); //TODO: Investigate

        renderer.render<typename Mesh::value_type>(mesh, vertexShader, fragmentShader, rasterShader);

        img.mirror('y'); //TODO: Investigate
        disp.display(img);

        if(disp.is_closed()) {
            break;
        }
    }
}

void texture() {
    TextureVertexShader vertexShader;
    TextureFragmentShader fragmentShader("/home/per/code/hen/models/cow/colorOpacityCow.png");

    auto m = loadMeshUv("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void color(const Eigen::Vector4f &color) {
    ColorVertexShader vertexShader;
    ColorFragmentShader fragmentShader;

    auto m = loadMeshColor("models/sphere2.obj", color);

    animate(m, vertexShader, fragmentShader);
}

void multiTexture() {
    TextureVertexShader vertexShader;
    MultiTextureFragmentShader fragmentShader("/home/per/code/hen/models/cow/colorOpacityCow.png",
                                              "/home/per/code/hen/models/cow/colorOpacityCowAO.png");

    auto m = loadMeshUv("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void flatShading() {
    FlatVertexShader vertexShader(Eigen::Vector3f(100,100,100));
    ColorFragmentShader fragmentShader;

    auto m = loadMeshNormal("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void phongShading() {
    PhongVertexShader vertexShader(Eigen::Vector3f(100,100,100));
    PhongFragmentShader fragmentShader;

    auto m = loadMeshNormal("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void equiRectangular() {
	NormalViewVertexShader vertexShader;
	EquiRectFragmentShader fragmentShader("/home/per/code/hen/equirect.jpg");

    auto m = loadMeshNormal("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void cubeMap() {
	NormalViewVertexShader vertexShader;
	CubemapFragmentShader fragmentShader("/home/per/code/hen/cubemap.jpg");

    auto m = loadMeshNormal("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

void shadow() {
    const auto lightPos = Eigen::Vector3f(5,5,5);
    const auto shadowProjectionMatrix = proj(-1, 1, -1, 1, 1, 1000);
    const auto shadowModelViewMatrix  = lookAt(lightPos, {0, 0, 0}, {0, 1, 0});

    ShadowGenVertexShader shadowGenVertexShader(shadowProjectionMatrix, shadowModelViewMatrix);
    ShadowGenFragmentShader shadowGenFragmentShader;

    const auto shadowMatrix = scaleBiasMatrix() * shadowProjectionMatrix * shadowModelViewMatrix;
    ShadowTextureVertexShader shadowTextureVertexShader(shadowMatrix);
    ShadowTextureFragmentShader shadowTextureFragmentShader("/home/per/code/hen/models/cow/colorOpacityCow.png");

    //Animate
    auto m = loadMeshUv("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animateShadow(m, shadowGenVertexShader, shadowGenFragmentShader, shadowTextureVertexShader, shadowTextureFragmentShader);
}

int main(int argc, char** argv) {
    shadow();
}



