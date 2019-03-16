#include "cfw/cfw.h"
#include "hen.h"
#include "io.h"
#include "stdcomp/shaders.h"
#include "utils.h"
#include "veclib.h"


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
    CImgColorRasterShader rasterShader(&framebuffer, &zbuffer);

    Timer<1> t;

    bool going = true;

    disp.setCloseCallback([&going]() { going = false; });

    while (going) {
        t.reset();
        fragmentShader.setTime(time());  // TODO: Start time from 0.0
        rasterShader.clear();
        renderer.render<std::tuple<VecLib::Vector2f>>(mesh, vertexShader, fragmentShader, rasterShader);
        float us = t.getUS();
        printf("%2.2f fps\n", 1.0 / (static_cast<double>(us) / 1000.0 / 1000.0));

        disp.render(framebuffer.data(), width, height);
        disp.paint();
    }
}

int main(int /*argc*/, char** /*argv*/) { shaderToy(); }
