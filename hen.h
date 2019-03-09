#ifndef __HEN_H__
#define __HEN_H__

// TODO: Unify empty uniforms
// TODO: Inheritance in shaders?
// TODO: Handle quads

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "veclib.h"

template <typename... Tp, typename Func, std::size_t... Idx>
std::tuple<Tp...> mytransformHelper(const std::tuple<Tp...>& t1, const std::tuple<Tp...>& t2, Func&& f,
                                    std::index_sequence<Idx...> /*unused*/) {
    return std::tuple<Tp...>((f(std::get<Idx>(t1), std::get<Idx>(t2)))...);
}

template <typename FuncT, typename... Tp>
inline std::tuple<Tp...> mytransform(const std::tuple<Tp...>& t1, const std::tuple<Tp...>& t2, FuncT f) {
    return mytransformHelper(t1, t2, f, std::index_sequence_for<Tp...>{});
}

template <class Tuple>
Tuple tupleDiff(const Tuple& a, const Tuple& b) {
    return mytransform(a, b, [](auto ta, auto tb) { return ta - tb; });
}

template <class Tuple>
Tuple tupleAddScaled(const Tuple& a, const Tuple& b, float s) {
    return mytransform(a, b, [s](auto ta, auto tb) { return ta + tb * s; });
}

template <class Type>
class TupleInterpolator {
    Type mStart;
    Type mDelta;

public:
    TupleInterpolator(const Type& in1, const Type& in2) {
        mStart = in1;
        mDelta = tupleDiff(in2, in1);
    }

    Type run(float val) {
        assert(val >= 0.0f);
        assert(val <= 1.00001f);

        return tupleAddScaled(mStart, mDelta, val);
    }
};

class Renderer {
private:
    bool mCullingEnabled = true;

    template <class Vertex, class FragmentShader, class RasterShader>
    void rasterizeFragment(const Vertex& v, const FragmentShader& fragmentShader, const RasterShader& rasterShader,
                           unsigned int x, unsigned int y) {

        const auto fragment = fragmentShader(v);

        rasterShader(fragment, x, y);
    }

    template <int PositionAttachment, class Vertex, class FragmentShader>
    void rasterizeLine(Vertex v1, Vertex v2, const FragmentShader& fragmentShader) {
        float x0 = std::get<PositionAttachment>(v1)[0];
        float y0 = std::get<PositionAttachment>(v1)[1];
        float x1 = std::get<PositionAttachment>(v2)[0];
        float y1 = std::get<PositionAttachment>(v2)[1];

        const bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

        if (steep) {
            std::swap(x0, y0);
            std::swap(x1, y1);
        }

        if (x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
            std::swap(v1, v2);
        }

        TupleInterpolator<Vertex> inp(v1, v2);

        const float dx = x1 - x0;
        const float dy = std::abs(y1 - y0);

        const float inpDist = std::sqrt(dx * dx + dy * dy);
        const float inpStep = 1.0f / inpDist;
        float inpPos = 0.0f;

        float error = dx / 2.0f;
        const int ystep = (y0 < y1) ? 1 : -1;
        auto y = static_cast<int>(y0);

        const auto maxX = static_cast<int>(x1);

        for (auto x = static_cast<int>(x0); x < maxX; x++) {
            inpPos += inpStep;
            if (steep) {
                rasterizeFragment(inp.run(inpPos), fragmentShader, y, x);
            } else {
                rasterizeFragment(inp.run(inpPos), fragmentShader, x, y);
            }

            error -= dy;
            if (error < 0) {
                y += ystep;
                error += dx;
            }
        }
    }

    enum { TOP_PART_Y_STEP = -1, BOTTOM_PART_Y_STEP = 1 };

    template <int PositionAttachment, int yStep, class Vertex, class FragmentShader, class RasterShader>
    void rasterizeTrianglePart(Vertex v1, Vertex v2, Vertex v3, const FragmentShader& fragmentShader,
                               const RasterShader& rasterShader) {
        auto p1 = std::get<PositionAttachment>(v1);
        auto p2 = std::get<PositionAttachment>(v2);
        auto p3 = std::get<PositionAttachment>(v3);

        if (p1[0] > p2[0]) {
            std::swap(p1, p2);
            std::swap(v1, v2);
        }

        //assert(roundAwayFromZero(p1[1]) == roundAwayFromZero(p2[1]));  // ??

        const uint16_t width  = rasterShader.getXResolution();
        const uint16_t height = rasterShader.getYResolution();

        //                                          Flat side up              Flat side Down
        const int yFirst         = yStep == 1 ? roundTowardsZero(p1[1]) :  roundTowardsZero(p1[1] - 1.0f);  // Flat side
        const int yOneBeyondLast = yStep == 1 ? roundTowardsZero(p3[1]) : roundAwayFromZero(p3[1] - 1.0f); // Pointy side

        const float yDistF = p3[1] - p1[1];

        TupleInterpolator<Vertex> inpl(v1, v3);
        TupleInterpolator<Vertex> inpr(v2, v3);

        const float yCenter = yFirst + 0.5f;
        float ypos = (yCenter - p1[1]) / yDistF;
        const float ystep = yStep / yDistF;

        for (int y = yFirst; yStep * (y - yOneBeyondLast) < 0; y += yStep) {
            if (y >= 0 && y < height) {
                Vertex vx0 = inpl.run(ypos);
                Vertex vx1 = inpr.run(ypos);

                TupleInterpolator<Vertex> inpx(vx0, vx1);
                const auto &posBegin = std::get<PositionAttachment>(vx0);
                const auto &posEnd = std::get<PositionAttachment>(vx1);
                const int xBegin = std::max(0, roundTowardsZero(posBegin[0]));
                const int xEnd = std::min((int) width, roundTowardsZero(posEnd[0]));

                const float xDistF = posEnd[0] - posBegin[0];
                const float xCenter = xBegin + 0.5f;
                float xpos = (xCenter - posBegin[0]) / xDistF;
                const float xstep = 1.0f / xDistF;

                for (int x = xBegin; x < xEnd; ++x) {
                    rasterizeFragment(inpx.run(xpos), fragmentShader, rasterShader, x, y);
                    xpos += xstep;
                }
            }
            ypos += ystep;
        }
    }

    inline int roundAwayFromZero(float f) { return std::floor(f + 0.5f); }

    inline int roundTowardsZero(float f) { return std::ceil(f - 0.5f); }

    template <class Vertex>
    VecLib::Vector3f cross3(const Vertex& v1, const Vertex& v2) {
        VecLib::Vector3f p1(v1);
        VecLib::Vector3f p2(v2);
        return p1.cross(p2);
    }

    template <class Vertex>
    bool isBackface(const Vertex& v1, const Vertex& v2, const Vertex& v3) {
        auto c = cross3(v2 - v1, v3 - v1);
        return c[2] < 0.0f;
    }

public:
    template <int PositionAttachment, class Vertex, class FragmentShader>
    void rasterizeTriangleLines(Vertex v1, Vertex v2, Vertex v3, const FragmentShader& fragmentShader) {
        rasterizeLine<PositionAttachment>(v1, v2, fragmentShader);
        rasterizeLine<PositionAttachment>(v1, v3, fragmentShader);
        rasterizeLine<PositionAttachment>(v2, v3, fragmentShader);
    }

    template <int PositionAttachment, class Vertex, class FragmentShader, class RasterShader>
    void rasterizeTriangle(Vertex v1, Vertex v2, Vertex v3, const FragmentShader& fragmentShader,
                           const RasterShader& rasterShader) {
        auto compareVertexY = [](Vertex& a, Vertex& b) {
            return std::get<PositionAttachment>(a)[1] <= std::get<PositionAttachment>(b)[1];
        };

        if (!compareVertexY(v1, v3)) {
            std::swap(v1, v3);
        }
        if (!compareVertexY(v1, v2)) {
            std::swap(v1, v2);
        }
        if (!compareVertexY(v2, v3)) {
            std::swap(v2, v3);
        }

        Vertex& topVertex = v1;
        Vertex& middleVertex = v2;
        Vertex& bottomVertex = v3;

        assert(compareVertexY(topVertex, middleVertex));
        assert(compareVertexY(middleVertex, bottomVertex));
        assert(compareVertexY(topVertex, bottomVertex));

        auto& topPos = std::get<PositionAttachment>(topVertex);
        auto& middlePos = std::get<PositionAttachment>(middleVertex);
        auto& bottomPos = std::get<PositionAttachment>(bottomVertex);

        if (topPos[1] == middlePos[1]) {
            // Flat top
            rasterizeTrianglePart<PositionAttachment, BOTTOM_PART_Y_STEP>(topVertex, middleVertex, bottomVertex,
                                                                          fragmentShader, rasterShader);
        } else if (middlePos[1] == bottomPos[1]) {
            // Flat bottom
            rasterizeTrianglePart<PositionAttachment, TOP_PART_Y_STEP>(bottomVertex, middleVertex, topVertex,
                                                                       fragmentShader, rasterShader);
        } else {
            TupleInterpolator<Vertex> inptb(topVertex, bottomVertex);
            Vertex split = inptb.run((middlePos[1] - topPos[1]) / (bottomPos[1] - topPos[1]));

            rasterizeTrianglePart<PositionAttachment, TOP_PART_Y_STEP>(middleVertex, split, topVertex, fragmentShader,
                                                                       rasterShader);
            rasterizeTrianglePart<PositionAttachment, BOTTOM_PART_Y_STEP>(middleVertex, split, bottomVertex,
                                                                          fragmentShader, rasterShader);
        }
    }

    template <class VertInType, class VertexShader, class FragmentShader, class RasterShader>
    void render(const std::vector<VertInType>& in, const VertexShader& vertexShader,
                const FragmentShader& fragmentShader, const RasterShader& rasterShader) {
        static_assert(std::is_same<VertInType, typename VertexShader::InType>::value, "Error");
        static_assert(std::is_same<typename VertexShader::OutType, typename FragmentShader::InType>::value, "Error");
        static_assert(std::is_same<typename FragmentShader::OutType, typename RasterShader::InType>::value, "Error");

        using VertOutFragInType = typename VertexShader::OutType;
        std::vector<VertOutFragInType> immStore;

        // Vertex stage
        std::transform(in.begin(), in.end(), std::back_inserter(immStore), vertexShader);

        // Now in clip space
        // Perspective divide
        constexpr auto POSITION_INDEX = static_cast<int>(VertexShader::Traits::POSITION_INDEX);
        using ScreenPosType = typename std::tuple_element<POSITION_INDEX, VertOutFragInType>::type;

        std::for_each(immStore.begin(), immStore.end(), [](VertOutFragInType& vert) {
            auto& pos = std::get<POSITION_INDEX>(vert);
            pos /= pos[3];
        });

        // Now in NDC

        const unsigned int xres = rasterShader.getXResolution();
        const unsigned int yres = rasterShader.getYResolution();
        std::for_each(immStore.begin(), immStore.end(), [this, xres, yres](VertOutFragInType& vert) {
            auto& pos = std::get<POSITION_INDEX>(vert);
            pos = pos + ScreenPosType(1.0, 1.0, 1.0, 0.0);  // TODO: Fix
            pos = pos / ScreenPosType(2.0, 2.0, 2.0, 1.0);
            pos[0] *= xres;
            pos[1] *= yres;
        });

        // Now in screen space

        rasterShader.clear();

        for (size_t i = 0; i < immStore.size(); i += 3) {
            // Primitive assembly
            const auto& v1 = immStore.at(i + 0);
            const auto& v2 = immStore.at(i + 1);
            const auto& v3 = immStore.at(i + 2);

            const auto& p1 = std::get<POSITION_INDEX>(v1);
            const auto& p2 = std::get<POSITION_INDEX>(v2);
            const auto& p3 = std::get<POSITION_INDEX>(v3);

            // Backface culling
            if (mCullingEnabled && isBackface(p1, p2, p3)) {
                continue;
            }

            // Rasterization
            rasterizeTriangle<POSITION_INDEX>(v1, v2, v3, fragmentShader, rasterShader);
        }
    }

    void setCulling(bool enabled) { mCullingEnabled = enabled; }
};

#endif
