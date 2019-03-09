//
// Created by per on 2019-03-07.
//

#include <hen.h>
#include <utils.h>
#include <cstring>
#include <cassert>

class TestVertexShader {
private:
    enum class InTraits {
        POSITION_INDEX = 0,
    };

public:
    typedef std::tuple<VecLib::Vector2f> InType;
    typedef std::tuple<VecLib::Vector4f> OutType;

    enum class Traits {
        POSITION_INDEX = 0,
    };

    OutType operator()(const InType& in) const {
        auto pos = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);

        pos.x() = (pos.x() - 8.0f) / 8.0f;
        pos.y() = (pos.y() - 4.0f) / 4.0f;

        return std::make_tuple(VecLib::Vector4f(pos, 0.0f, 1.0f));
    }
};

class TestFragmentShader {
private:
    enum class InTraits { POSITION_INDEX = 0 };

public:
    using InType = std::tuple<VecLib::Vector4f>;
    using OutType = std::tuple<uint8_t>;

    enum class Traits { COLOR_INDEX = 0 };

    OutType operator()(const InType& in) const {
        const auto& pos = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        // assert(pos[2] < 0.0);

        return OutType(0xff);
    }
};

class TestRasterShader {
private:
    enum class InTraits { COLOR_INDEX = 0 };

    PixelBuffer<unsigned char>* mFrame;

public:
    using InType = std::tuple<uint8_t>;

    TestRasterShader(PixelBuffer<uint8_t>* frame)
            : mFrame(frame) {}

    void operator()(const InType& fragment, unsigned int x, unsigned int y) const {
        constexpr auto ColorAttachment = static_cast<int>(InTraits::COLOR_INDEX);

        const auto& color = std::get<ColorAttachment>(fragment);

        //printf("%d %d\n", x, y);
        assert(x >= 0 && x < mFrame->width());
        assert(y >= 0 && y < mFrame->height());
        mFrame->at(x, y) += 1;
    }

    void clear() const {
        mFrame->fill(0);
    }

    unsigned int getXResolution() const { return mFrame->width(); }
    unsigned int getYResolution() const { return mFrame->height(); }
};

static void testRasterization() {
    Renderer renderer;

    const uint16_t width = 16;
    const uint16_t height = 8;

    PixelBuffer<uint8_t> framebuffer(width, height);
    TestRasterShader rasterShader(&framebuffer);
    TestVertexShader vertexShader;
    TestFragmentShader fragmentShader;

    using Vertex = std::tuple<VecLib::Vector2f>;
    std::vector<Vertex> verts;

    verts.push_back(Vertex({1.0f, 1.0f}));
    verts.push_back(Vertex({6.0f, 2.0f}));
    verts.push_back(Vertex({2.0f, 4.0f}));

    verts.push_back(Vertex({4.5f, 0.5f}));
    verts.push_back(Vertex({4.5f, 0.5f}));
    verts.push_back(Vertex({4.5f, 0.5f}));

    verts.push_back(Vertex({6.25f, 0.25f}));
    verts.push_back(Vertex({6.25f, 1.25f}));
    verts.push_back(Vertex({5.25f, 1.25f}));

    verts.push_back(Vertex({7.5f, 0.5f}));
    verts.push_back(Vertex({7.5f, 1.5f}));
    verts.push_back(Vertex({6.5f, 1.5f}));

    verts.push_back(Vertex({15.0f, 0.0f}));
    verts.push_back(Vertex({14.5f, 2.5f}));
    verts.push_back(Vertex({13.5f, 1.5f}));

    verts.push_back(Vertex({14.5f, 2.5f}));
    verts.push_back(Vertex({14.5f, 4.5f}));
    verts.push_back(Vertex({13.5f, 1.5f}));

    verts.push_back(Vertex({ 9.75f, 0.75f}));
    verts.push_back(Vertex({11.75f, 2.50f}));
    verts.push_back(Vertex({ 7.75f, 2.50f}));

    verts.push_back(Vertex({11.75f, 2.50f}));
    verts.push_back(Vertex({ 9.50f, 5.25f}));
    verts.push_back(Vertex({ 7.75f, 2.50f}));


    verts.push_back(Vertex({7.0f, 4.0f}));
    verts.push_back(Vertex({9.5f, 5.5f}));
    verts.push_back(Vertex({8.0f, 7.0f}));

    verts.push_back(Vertex({7.0f, 4.0f}));
    verts.push_back(Vertex({8.0f, 7.0f}));
    verts.push_back(Vertex({5.0f, 6.0f}));

    verts.push_back(Vertex({7.0f, 4.0f}));
    verts.push_back(Vertex({5.0f, 6.0f}));
    verts.push_back(Vertex({1.0f, 6.0f}));

    verts.push_back(Vertex({11.5f, 4.5f}));
    verts.push_back(Vertex({12.5f, 5.5f}));
    verts.push_back(Vertex({11.5f, 6.5f}));

    verts.push_back(Vertex({13.5f, 5.5f}));
    verts.push_back(Vertex({15.5f, 5.5f}));
    verts.push_back(Vertex({13.5f, 7.5f}));

    verts.push_back(Vertex({15.5f, 5.5f}));
    verts.push_back(Vertex({15.5f, 7.5f}));
    verts.push_back(Vertex({13.5f, 7.5f}));

    // Clipping
    verts.push_back(Vertex({ 9.5f, 7.5f}));
    verts.push_back(Vertex({10.5f, 7.5f}));
    verts.push_back(Vertex({ 9.5f, 9.0f}));

    renderer.render(verts, vertexShader, fragmentShader, rasterShader);
#if 0
    puts("{");
    for(int y = 0; y < height; ++y) {
        printf("  ");
        for(int x = 0; x < width; ++x) {
            printf("%d, ", framebuffer.at(x, y));
        }
        puts("");
    }
    puts("};");
#else
    uint8_t facit[] = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };

    assert(memcmp(facit, framebuffer.data(), width * height) == 0);
#endif
}


int main() {
    testRasterization();
    //work();
}