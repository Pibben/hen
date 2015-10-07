#ifndef __HEN_H__
#define __HEN_H__

//TODO: Render depth only
//TODO: Unify empty uniforms
//TODO: Inheritance in shaders?

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <tuple>
#include <utility>
#include <vector>
#include <type_traits>

template<std::size_t I = 0, typename FuncT, typename ... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type
mytransform(const std::tuple<Tp...> &, const std::tuple<Tp...> &, std::tuple<Tp...> &, FuncT) // Unused arguments are given no names.
{}

template<std::size_t I = 0, typename FuncT, typename ... Tp>
inline typename std::enable_if<I < sizeof...(Tp), void>::type
mytransform(const std::tuple<Tp...>& t1, const std::tuple<Tp...>& t2, std::tuple<Tp...>& r, FuncT f)
{
    std::get<I>(r) = f(std::get<I>(t1), std::get<I>(t2));
    mytransform<I + 1, FuncT, Tp...>(t1, t2, r, f);
}

template <class Tuple>
Tuple tupleDiff(const Tuple& a, const Tuple& b) {
    Tuple c;
    mytransform(a, b, c, [](auto ta, auto tb) { return ta - tb; });
    return c;
}

template <class Tuple>
Tuple tupleAddScaled(const Tuple& a, const Tuple& b, float s) {
    Tuple c;
    mytransform(a, b, c, [s](auto ta, auto tb) { return ta + tb * s; });
    return c;
}

template <class VertOutFragIn>
class TupleInterpolator {
    typedef VertOutFragIn VertOutFragInType;
    VertOutFragIn mStart;
    VertOutFragIn mDelta;
public:
    TupleInterpolator(const VertOutFragIn& in1, const VertOutFragIn& in2) {
        mStart = in1;
        mDelta = tupleDiff(in2, in1);
    }
    
    VertOutFragIn run(float val) {
        assert(val >= 0.0);
        assert(val <= 1.0);
        
        return tupleAddScaled(mStart, mDelta, val);
    }
};



template <class FragOutType, class Traits, int RES_X, int RES_Y>
class Renderer {

private:
    template <class Vertex, class FragmentShader, class RasterShader>
    void rasterizeFragment(const Vertex& v, const FragmentShader& fragmentShader, const RasterShader& rasterShader,
                           unsigned int x, unsigned int y) {
        constexpr int ColorAttachment = FragmentShader::Traits::COLOR_ATTACHMENT;
        constexpr int DepthAttachment = FragmentShader::Traits::DEPTH_ATTACHMENT;

        const auto fragment = fragmentShader(v);
        const auto& depth = std::get<DepthAttachment>(fragment);
        const auto& color = std::get<ColorAttachment>(fragment);

        rasterShader(color, depth, x, y);
    }

    template <int PositionAttachment, class Vertex, class FragmentShader>
    void rasterizeLine(Vertex v1, Vertex v2, const FragmentShader& fragmentShader) {

        float x0 = std::get<PositionAttachment>(v1)[0];
        float y0 = std::get<PositionAttachment>(v1)[1];
        float x1 = std::get<PositionAttachment>(v2)[0];
        float y1 = std::get<PositionAttachment>(v2)[1];


        const bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

        if(steep) {
            std::swap(x0, y0);
            std::swap(x1, y1);
        }

        if(x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
            std::swap(v1, v2);
        }

        TupleInterpolator<Vertex> inp(v1, v2);

        const float dx = x1-x0;
        const float dy = std::abs(y1-y0);

        const float inpDist = std::sqrt(dx*dx+dy*dy);
        const float inpStep = 1.0 / inpDist;
        float inpPos = 0.0f;

        float error = dx / 2.0f;
        const int ystep = (y0 < y1) ? 1 : -1;
        int y = (int)y0;

        const int maxX = (int)x1;


        for (int x = (int) x0; x < maxX; x++) {

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

    enum {TOP_PART_Y_STEP = -1, BOTTOM_PART_Y_STEP = 1};

    template <int PositionAttachment, int yStep, class Vertex, class FragmentShader, class RasterShader>
    void rasterizeTrianglePart(Vertex v1, Vertex v2, Vertex v3,
                               const FragmentShader& fragmentShader, const RasterShader& rasterShader) {

        auto p1 = std::get<PositionAttachment>(v1);
        auto p2 = std::get<PositionAttachment>(v2);
        auto p3 = std::get<PositionAttachment>(v3);

        if(p1[0] > p2[0]) {
            std::swap(p1, p2);
            std::swap(v1, v2);
        }

        assert(roundAwayFromZero(p1[1]) == roundAwayFromZero(p2[1]));

        const int y0 = yStep == 1 ? roundTowardsZero(p1[1]) : roundAwayFromZero(p1[1]-1.0f);
        const int y2 = yStep == 1 ? roundTowardsZero(p3[1]) : roundAwayFromZero(p3[1]-1.0f);

        const float yDistF = p3[1] - p1[1];

        TupleInterpolator<Vertex> inpl(v1, v3);
        TupleInterpolator<Vertex> inpr(v2, v3);

        float ypos = (y0 - p1[1] + 0.5) / yDistF;
        float ystep = yStep / yDistF;

        for(int y = y0; yStep*(y - y2) < 0; y+=yStep) {
            Vertex vx0 = inpl.run(ypos);
            Vertex vx1 = inpr.run(ypos);

            TupleInterpolator<Vertex> inpx(vx0, vx1);
            int xBegin = roundTowardsZero(std::get<PositionAttachment>(vx0)[0]);
            int xEnd =   roundTowardsZero(std::get<PositionAttachment>(vx1)[0]);

            const float xstep = 1.0 / (xEnd - xBegin + 1);
            float xpos = 0.0f;

            for(int x = xBegin; x < xEnd; ++x) {
                rasterizeFragment(inpx.run(xpos), fragmentShader, rasterShader, x, y);
                xpos += xstep;
            }
            ypos += ystep;
        }
    }

    inline int roundAwayFromZero(float f) {
        return std::floor(f + 0.5f);
    }

    inline int roundTowardsZero(float f) {
        return std::ceil(f - 0.5f);
    }

    template <class Vertex>
    Eigen::Vector3f cross3(const Vertex& v1, const Vertex& v2) {
        Eigen::Vector3f p1 = v1.template block<3,1>(0,0);
        Eigen::Vector3f p2 = v2.template block<3,1>(0,0);
        return p1.cross(p2);
    }

    template <class Vertex>
    bool isBackface(const Vertex& v1, const Vertex& v2, const Vertex& v3) {
        auto c = cross3(v2-v1, v3-v1);
        return c[2] < 0.0;
    }

public:
    template <int PositionAttachment, class Vertex, class FragmentShader>
    void rasterizeTriangleLines(Vertex v1, Vertex v2, Vertex v3,
                                const FragmentShader& fragmentShader) {
        rasterizeLine<PositionAttachment>(v1, v2, fragmentShader);
        rasterizeLine<PositionAttachment>(v1, v3, fragmentShader);
        rasterizeLine<PositionAttachment>(v2, v3, fragmentShader);
    }


    template <int PositionAttachment, class Vertex, class FragmentShader, class RasterShader>
    void rasterizeTriangle(Vertex v1, Vertex v2, Vertex v3,
                           const FragmentShader& fragmentShader, const RasterShader& rasterShader) {

        auto compareVertexY = [](Vertex& a, Vertex& b) {
            return std::get<PositionAttachment>(a)[1] <= std::get<PositionAttachment>(b)[1];
        };

        if(!compareVertexY(v1, v3)) std::swap(v1, v3);
        if(!compareVertexY(v1, v2)) std::swap(v1, v2);
        if(!compareVertexY(v2, v3)) std::swap(v2, v3);

        Vertex& topVertex = v1;
        Vertex& middleVertex = v2;
        Vertex& bottomVertex = v3;

        assert(compareVertexY(topVertex, middleVertex));
        assert(compareVertexY(middleVertex, bottomVertex));
        assert(compareVertexY(topVertex, bottomVertex));

        auto& topPos    = std::get<PositionAttachment>(topVertex);
        auto& middlePos = std::get<PositionAttachment>(middleVertex);
        auto& bottomPos = std::get<PositionAttachment>(bottomVertex);

        if(topPos[1] == middlePos[1]) {
            //Flat top
            rasterizeTrianglePart<PositionAttachment, BOTTOM_PART_Y_STEP>(topVertex, middleVertex, bottomVertex, fragmentShader, rasterShader);
        } else if(middlePos[1] == bottomPos[1]) {
            //Flat bottom
            rasterizeTrianglePart<PositionAttachment, TOP_PART_Y_STEP>(bottomVertex, middleVertex, topVertex, fragmentShader, rasterShader);
        } else {
            TupleInterpolator<Vertex> inptb(topVertex, bottomVertex);
            Vertex split = inptb.run((middlePos[1] - topPos[1]) / (bottomPos[1] - topPos[1]));

            rasterizeTrianglePart<PositionAttachment, TOP_PART_Y_STEP>(middleVertex, split, topVertex, fragmentShader, rasterShader);
            rasterizeTrianglePart<PositionAttachment, BOTTOM_PART_Y_STEP>(middleVertex, split, bottomVertex, fragmentShader, rasterShader);
        }
    }

    template <class VertInType, class VertexShader, class FragmentShader, class RasterShader>
    void render(const std::vector<VertInType>& in, const VertexShader& vertexShader,
                const FragmentShader& fragmentShader, const RasterShader& rasterShader) {

        constexpr int POSITION_ATTACHMENT = VertexShader::Traits::POSITION_ATTACHMENT;

        typedef typename VertexShader::OutType VertOutFragInType;
        static_assert(std::is_same<FragOutType, typename FragmentShader::OutType>::value, "Error");

        typedef typename std::tuple_element<POSITION_ATTACHMENT, VertOutFragInType>::type ScreenPosType;

        std::vector<VertOutFragInType> immStore;

        //Vertex stage
        std::transform(in.begin(), in.end(), std::back_inserter(immStore), vertexShader);
        
        //Now in clip space
        //Perspective divide
        std::for_each(immStore.begin(), immStore.end(), [](VertOutFragInType& vert) {
            auto& pos = std::get<POSITION_ATTACHMENT>(vert);
            pos /= pos[3];
        });

        //Now in NDC

        std::for_each(immStore.begin(), immStore.end(), [this](VertOutFragInType& vert) {
            auto& pos = std::get<POSITION_ATTACHMENT>(vert);
            pos = pos + ScreenPosType(1.0, 1.0, 1.0, 0.0); //TODO: Fix
            pos = pos.cwiseQuotient(ScreenPosType(2.0, 2.0, 2.0, 1.0));
            pos[0] *= RES_X;
            pos[1] *= RES_Y;
        });

        //Now in screen space

        rasterShader.clear();

        for(size_t i = 0; i < immStore.size(); i+=3) {
            //Primitive assembly
            const auto& v1 = immStore.at(i+0);
            const auto& v2 = immStore.at(i+1);
            const auto& v3 = immStore.at(i+2);

            const auto& p1 = std::get<POSITION_ATTACHMENT>(v1);
            const auto& p2 = std::get<POSITION_ATTACHMENT>(v2);
            const auto& p3 = std::get<POSITION_ATTACHMENT>(v3);

            //Backface culling
            if(isBackface(p1, p2, p3)) {
                continue;
            }

            //Rasterization
            rasterizeTriangle<POSITION_ATTACHMENT>(v1, v2, v3, fragmentShader, rasterShader);
        }
    }

};

#endif
