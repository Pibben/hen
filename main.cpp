/*
 * main.cpp
 *
 *  Created on: Oct 6, 2014
 *      Author: per
 */
#include <chrono>
#include <thread>

#include "cfw/cfw.h"
#include "hen.h"
#include "io.h"
#include "stdcomp/shaders.h"
#include "utils.h"
#include "veclib.h"

using PositionAndColor = std::tuple<VecLib::Vector4f, VecLib::Vector4f>;
static std::vector<PositionAndColor> loadMeshColor(const std::string& filename, const VecLib::Vector4f& color) {
    std::vector<VecLib::Vector3f> vertices;
    std::vector<VecLib::Vector2f> uvs;
    std::vector<VecLib::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, &vertices, &uvs, &normals, &faces);
    printf("Loaded %lu faces\n", faces.size());

    std::vector<PositionAndColor> m;

    for (const auto& f : faces) {
        for (int i = 0; i < 3; ++i) {
            m.emplace_back(VecLib::Vector4f(vertices[f.coords[i]], 1.0), color);
        }
    }

    return m;
}

using PositionAndUv = std::tuple<VecLib::Vector4f, VecLib::Vector2f>;
static std::vector<PositionAndUv> loadMeshUv(const std::string& filename) {
    std::vector<VecLib::Vector3f> vertices;
    std::vector<VecLib::Vector2f> uvs;
    std::vector<VecLib::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, &vertices, &uvs, &normals, &faces);
    printf("Loaded %lu faces\n", faces.size());

    std::vector<PositionAndUv> m;

    for (const auto& f : faces) {
        for (int i = 0; i < 3; ++i) {
            m.emplace_back(VecLib::Vector4f(vertices[f.coords[i]], 1.0), uvs[f.uvs[i]]);
        }
    }

    return m;
}

using PositionAndNormal = std::tuple<VecLib::Vector4f, VecLib::Vector3f>;
static std::vector<PositionAndNormal> loadMeshNormal(const std::string& filename) {
    std::vector<VecLib::Vector3f> vertices;
    std::vector<VecLib::Vector2f> uvs;
    std::vector<VecLib::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, &vertices, &uvs, &normals, &faces);
    printf("Loaded %lu faces\n", faces.size());

    assert(!faces.empty());

    std::vector<PositionAndNormal> m;

    for (const auto& f : faces) {
        for (int i = 0; i < 3; ++i) {
            m.emplace_back(VecLib::Vector4f(vertices[f.coords[i]], 1.0), normals[f.coords[i]]);
        }
    }

    return m;
}

using PositionNormalAndTangent = std::tuple<VecLib::Vector4f, VecLib::Vector3f, VecLib::Vector2f, VecLib::Vector4f>;
static std::vector<PositionNormalAndTangent> loadMeshTangent(const std::string& filename) {
    std::vector<VecLib::Vector3f> vertices;
    std::vector<VecLib::Vector2f> uvs;
    std::vector<VecLib::Vector3f> normals;
    std::vector<Face> faces;

    loadObj(filename, &vertices, &uvs, &normals, &faces);
    printf("Loaded %lu faces\n", faces.size());

    std::vector<PositionNormalAndTangent> m;

    std::vector<VecLib::Vector3f> tan1(vertices.size());
    std::vector<VecLib::Vector3f> tan2(vertices.size());

    for (auto& face : faces) {
        const int i1 = face.coords[0];
        const int i2 = face.coords[1];
        const int i3 = face.coords[2];

        const int u1 = face.uvs[0];
        const int u2 = face.uvs[1];
        const int u3 = face.uvs[2];

        const VecLib::Vector3f& v1 = vertices[i1];
        const VecLib::Vector3f& v2 = vertices[i2];
        const VecLib::Vector3f& v3 = vertices[i3];

        const VecLib::Vector2f& w1 = uvs[u1];
        const VecLib::Vector2f& w2 = uvs[u2];
        const VecLib::Vector2f& w3 = uvs[u3];

        const float x1 = v2.x() - v1.x();
        const float x2 = v3.x() - v1.x();
        const float y1 = v2.y() - v1.y();
        const float y2 = v3.y() - v1.y();
        const float z1 = v2.z() - v1.z();
        const float z2 = v3.z() - v1.z();

        const float s1 = w2.x() - w1.x();
        const float s2 = w3.x() - w1.x();
        const float t1 = w2.y() - w1.y();
        const float t2 = w3.y() - w1.y();

        const float r = 1.0f / (s1 * t2 - s2 * t1);

        const VecLib::Vector3f sdir((t2 * x1 - t1 * x2) * r, (t2 * y1 - t1 * y2) * r, (t2 * z1 - t1 * z2) * r);
        const VecLib::Vector3f tdir((s1 * x2 - s2 * x1) * r, (s1 * y2 - s2 * y1) * r, (s1 * z2 - s2 * z1) * r);

        tan1[i1] += sdir;
        tan1[i2] += sdir;
        tan1[i3] += sdir;

        tan2[i1] += tdir;
        tan2[i2] += tdir;
        tan2[i3] += tdir;
    }

    std::vector<VecLib::Vector4f> tangents(vertices.size());

    for (size_t j = 0; j < vertices.size(); ++j) {
        const VecLib::Vector3f& n = normals[j];
        const VecLib::Vector3f& t = tan1[j];

        // std::cout << t << std::endl;
        // std::cout << tan2[j] << std::endl;

        tangents[j] = (t - n * dot(n, t));
        tangents[j].normalize();
        tangents[j].w() = (dot(cross(n, t), tan2[j]) < 0.0F) ? -1.0F : 1.0F;
    }

    for (const auto& f : faces) {
        for (int i = 0; i < 3; ++i) {
            // std::cout <<  tangents[f.uvs[i]] << std::endl;
            m.emplace_back(VecLib::Vector4f(vertices[f.coords[i]], 1.0), normals[f.coords[i]], uvs[f.uvs[i]],
                           tangents[f.coords[i]]);
        }
    }

    // exit(0);

    return m;
}

#if 0
template <class Mesh, class VertexShader, class FragmentShader>
static void animate(const Mesh& mesh, VertexShader& vertexShader, FragmentShader& fragmentShader) {
    vertexShader.projMatrix() = proj(-5, 5, -5, 5, 5, 30);

    vertexShader.modelViewMatrix() = lookAt({0, 7, 7}, {0, 0, 0}, {0, 1, 0});

    const int width = 640*2;
    const int height = 480*2;

    //Renderer
    Renderer renderer;

    cimg_library::CImgDisplay disp;
    cimg_library::CImg<unsigned char> framebuffer(width, height, 1, 3);
    cimg_library::CImg<float> zbuffer(width, height);

    VecLib::Matrix4f rotMatrix = rotateY(1/180.0*M_PI);

    CImgColorRasterShader rasterShader(framebuffer, zbuffer);

    Timer<1> t;

    while(true) {
        vertexShader.modelViewMatrix() *= rotMatrix;

        t.reset();
        renderer.render<typename Mesh::value_type>(mesh, vertexShader, fragmentShader, rasterShader);
        float us = t.getUS();
        printf("%2.2f fps\n", 1.0 / (us / 1000.0 / 1000.0));

        framebuffer.mirror('y'); //TODO: Investigate
        disp.display(framebuffer);

        if(disp.is_closed()) {
            break;
        }
    }
}


template <class Mesh, class VertexGenShader, class FragmentGenShader,
                      class VertexShader, class FragmentShader>
static void animateShadow(const Mesh& mesh, VertexGenShader& vertexGenShader, FragmentGenShader& fragmentGenShader,
                                     VertexShader& vertexShader, FragmentShader& fragmentShader) {
    vertexShader.projMatrix() = proj(-5, 5, -5, 5, 5, 30);

    vertexShader.modelViewMatrix() = lookAt({0, 7, 7}, {0, 0, 0}, {0, 1, 0});

    const int width = 640*2;
    const int height = 480*2;

    //Renderers
    Renderer renderer;

    cimg_library::CImgDisplay disp;
    cimg_library::CImg<unsigned char> framebuffer(width, height, 1, 3);
    cimg_library::CImg<float> zbuffer(width, height);

    //VecLib::AngleAxisf aa((1/180.0)*M_PI, VecLib::Vector3f::UnitY());
    //VecLib::Matrix4f rotMatrix = VecLib::Matrix4f::Identity();
    //rotMatrix.topLeftCorner<3,3>() = aa.matrix();

    auto& depthTexture = fragmentShader.getDepthTexture(); //TODO: Who should own the shadow depth buffer?

    CImgDepthRasterShader shadowRasteShader(depthTexture);
    CImgColorRasterShader rasterShader(framebuffer, zbuffer);

    while(true) {
        //vertexShader.modelViewMatrix() *= rotMatrix;

        //Render shadow depth
        renderer.render<typename Mesh::value_type>(mesh, vertexGenShader, fragmentGenShader, shadowRasteShader);

        depthTexture.mirror('y'); //TODO: Investigate

        renderer.render<typename Mesh::value_type>(mesh, vertexShader, fragmentShader, rasterShader);

        framebuffer.mirror('y'); //TODO: Investigate
        disp.display(framebuffer);

        if(disp.is_closed()) {
            break;
        }
    }
}

static void texture() {
    TextureVertexShader vertexShader;
    TextureFragmentShader fragmentShader("/home/per/code/hen/models/cow/colorOpacityCow.png");

    auto m = loadMeshUv("/home/per/code/hen/models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

static void normalMap() {
    NormalMapVertexShader vertexShader(VecLib::Vector3f(100,100,100));
    NormalMapFragmentShader fragmentShader("/home/per/code/hen/models/cow/colorOpacityCowNorm.png");

    auto m = loadMeshTangent("models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

static void color(const VecLib::Vector4f &color) {
    ColorVertexShader vertexShader;
    ColorFragmentShader fragmentShader;

    auto m = loadMeshColor("/home/per/code/hen/models/sphere2.obj", color);

    animate(m, vertexShader, fragmentShader);
}

static void multiTexture() {
    TextureVertexShader vertexShader;
    MultiTextureFragmentShader fragmentShader("/home/per/code/hen/models/cow/colorOpacityCow.png",
                                              "/home/per/code/hen/models/cow/colorOpacityCowAO.png");

    auto m = loadMeshUv("/home/per/code/hen/models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

static void flatShading() {
    FlatVertexShader vertexShader(VecLib::Vector3f(100.0f, 100.0f, 100.0f));
    ColorFragmentShader fragmentShader;

    auto m = loadMeshNormal("/home/per/code/hen/models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

static void phongShading() {
    PhongVertexShader vertexShader(VecLib::Vector3f(100.0f, 100.0f, 100.0f));
    PhongFragmentShader fragmentShader;

    auto m = loadMeshNormal("/home/per/code/hen/models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

static void equiRectangular() {
	NormalViewVertexShader vertexShader;
	EquiRectFragmentShader fragmentShader("/home/per/code/hen/equirect.jpg");

    auto m = loadMeshNormal("/home/per/code/hen/models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

static void cubeMap() {
	NormalViewVertexShader vertexShader;
	CubemapFragmentShader fragmentShader("/home/per/code/hen/cubemap.jpg");

    auto m = loadMeshNormal("/home/per/code/hen/models/cow/cowTM08New00RTime02-tri-norm.obj");

    animate(m, vertexShader, fragmentShader);
}

static void shadow() {
    const auto lightPos = VecLib::Vector3f(5,5,5);
    const auto shadowProjectionMatrix = proj(-1, 1, -1, 1, 1, 1000);
    const auto shadowModelViewMatrix  = lookAt(lightPos, {0, 0, 0}, {0, 1, 0});

    ShadowGenVertexShader shadowGenVertexShader(shadowProjectionMatrix, shadowModelViewMatrix);
    ShadowGenFragmentShader shadowGenFragmentShader;

    const auto shadowMatrix = scaleBiasMatrix() * shadowProjectionMatrix * shadowModelViewMatrix;
    ShadowTextureVertexShader shadowTextureVertexShader(shadowMatrix);
    ShadowTextureFragmentShader shadowTextureFragmentShader("/home/per/code/hen/models/cow/colorOpacityCow.png");

    //Animate
    auto m = loadMeshUv("/home/per/code/hen/models/cow/cowTM08New00RTime02-tri-norm.obj");

    animateShadow(m, shadowGenVertexShader, shadowGenFragmentShader, shadowTextureVertexShader, shadowTextureFragmentShader);
}
#endif

static float time() {
    std::chrono::milliseconds ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

    // printf("%d %f\n", ms.count(), (int)ms.count() / 1000.0f);

    return static_cast<int>(ms.count()) / 1000.0f;
}

static void shaderToy() {
    constexpr int width = 640;
    constexpr int height = 480;

    // Renderer
    Renderer renderer;

    cfw::Window disp(width, height);
    PixelBuffer<unsigned char> framebuffer(width, height, 3);
    PixelBuffer<float> zbuffer(width, height);

    auto mesh = unitQuad();

    ShadertoyVertexShader vertexShader;
    ShadertoyWaveFragmentShader fragmentShader;
    CImgColorRasterShader rasterShader(framebuffer, zbuffer);

    Timer<1> t;

    bool going = true;

    disp.setCloseCallback([&going]() { going = false; });

    while (going) {
        t.reset();
        fragmentShader.setTime(time());  // TODO: Start time from 0.0
        renderer.render<std::tuple<VecLib::Vector2f>>(mesh, vertexShader, fragmentShader, rasterShader);
        float us = t.getUS();
        printf("%2.2f fps\n", 1.0 / (static_cast<double>(us) / 1000.0 / 1000.0));

        // framebuffer.mirror('y'); //TODO: Investigate
        disp.render(framebuffer.data(), width, height);  // disp.display(framebuffer);
        disp.paint();

        // if(disp.is_closed()) {
        //    break;
        //}
    }
}

static void libtest() {
    //    Eigen::Matrix4f m = lookAt({0, 7, 7}, {0, 0, 0}, {0, 1, 0});
    //    std::cout << m * Eigen::Vector4f(1, 0, 0, 0) << std::endl;
    //    std::cout << m * Eigen::Vector4f(0, 1, 0, 0) << std::endl;
    //    std::cout << m * Eigen::Vector4f(0, 0, 1, 0) << std::endl;
    //    std::cout << m * Eigen::Vector4f(0, 0, 0, 1) << std::endl;
    VecLib::Matrix4f m = lookAt({0, 7, 7}, {0, 0, 0}, {0, 1, 0});
    std::cout << m * VecLib::Vector4f(1, 0, 0, 0) << std::endl;
    std::cout << m * VecLib::Vector4f(0, 1, 0, 0) << std::endl;
    std::cout << m * VecLib::Vector4f(0, 0, 1, 0) << std::endl;
    std::cout << m * VecLib::Vector4f(0, 0, 0, 1) << std::endl;
    /*
        0.707107
        0
        0
        0
        0
        0.707107
        0.707107
        0
        0
        -0.707107
        0.707107
        0
        0
        0
        -9.8995
     */
}

int main(int /*argc*/, char** /*argv*/) { shaderToy(); }
