#ifndef __HEN_H__
#define __HEN_H__

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <tuple>
#include <utility>
#include <vector>
#include <type_traits>

#include "CImg.h"
#undef Success

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


template <class Type, int RES_X, int RES_Y>
class FramebufferAdapter {
private:
    typedef std::vector<Type> ArrayType;
    ArrayType mArray;
    unsigned int mSizeX;
    unsigned int mSizeY;
    Type mTrash;
public:
    FramebufferAdapter() : mArray(RES_X*RES_Y), mSizeX(RES_X), mSizeY(RES_Y), mTrash() {}

    Type& operator()(unsigned int x, unsigned int y) {
        if(x >= 0 && x < mSizeX && y >= 0 && y < mSizeY) {
            return mArray[y*mSizeX+x];
        } else {
            return mTrash;
        }
    }

    const Type& operator()(unsigned int x, unsigned int y) const {
        if(x >= 0 && x < mSizeX && y >= 0 && y < mSizeY) {
            return mArray[y*mSizeX+x];
        } else {
            return mTrash;
        }
    }

    unsigned int sizeX() const { return mSizeX; }
    unsigned int sizeY() const { return mSizeY; }

    typename ArrayType::iterator begin() { return mArray.begin(); }
    typename ArrayType::iterator end() { return mArray.end(); }

    size_t size() const { return mArray.size(); }

    Type& at(int idx) { return mArray.at(idx); }
    const Type& at(int idx) const { return mArray.at(idx); }
};




template <class FragOutType, class Traits, int RES_X, int RES_Y>
class Renderer {
private:
    static constexpr int COLOR_ATTACHMENT = Traits::COLOR_ATTACHMENT;
    static constexpr int DEPTH_ATTACHMENT = Traits::DEPTH_ATTACHMENT;

    typedef typename std::tuple_element<COLOR_ATTACHMENT, FragOutType>::type DataType;
    typedef typename std::tuple_element<DEPTH_ATTACHMENT, FragOutType>::type DepthType;

    typedef FramebufferAdapter<DataType, RES_X, RES_Y> FrameBufferType;
    FrameBufferType frameBuffer;
    typedef FramebufferAdapter<DepthType, RES_X, RES_Y> DepthBufferType;
    DepthBufferType depthBuffer;
    
    template <class Vertex, class FragmentShader>
    void rasterizeFragment(const Vertex& v, const FragmentShader& fragmentShader, unsigned int x, unsigned int y) {
        static constexpr int ColorAttachment = FragmentShader::Traits::COLOR_ATTACHMENT;
        static constexpr int DepthAttachment = FragmentShader::Traits::DEPTH_ATTACHMENT;

        const auto fragment = fragmentShader(v);
        const float depth = std::get<DepthAttachment>(fragment);
        if(depth > 0.0 && depth < 1.0 && depth < depthBuffer(x, y)) {
            const auto color = std::get<ColorAttachment>(fragment);
            frameBuffer(x, y) = color;
            depthBuffer(x, y) = depth;
        }
    }

    template <class Vertex, class FragmentShader, int PositionAttachment>
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

    template <class Vertex, int PositionAttachment, class FragmentShader, int yStep>
    void rasterizeTrianglePart(Vertex v1, Vertex v2, Vertex v3,
                                      const FragmentShader& fragmentShader) {

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
                rasterizeFragment(inpx.run(xpos), fragmentShader, x, y);
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
    Renderer() {
        clear();
    }

    template <class Vertex, int PositionAttachment, class FragmentShader>
    void rasterizeTriangleLines(Vertex v1, Vertex v2, Vertex v3,
                                       const FragmentShader& fragmentShader) {
        rasterizeLine<Vertex, FragmentShader, PositionAttachment>(v1, v2, fragmentShader);
        rasterizeLine<Vertex, FragmentShader, PositionAttachment>(v1, v3, fragmentShader);
        rasterizeLine<Vertex, FragmentShader, PositionAttachment>(v2, v3, fragmentShader);
    }


    template <class Vertex, int PositionAttachment, class FragmentShader>
    void rasterizeTriangle(Vertex v1, Vertex v2, Vertex v3,
                                  const FragmentShader& fragmentShader) {

        Vertex& top = v1;
        Vertex& mid = v2;
        Vertex& bot = v3;

        auto criterion = [](Vertex& a, Vertex& b) {
            return std::get<PositionAttachment>(a)[1] <= std::get<PositionAttachment>(b)[1];
        };

        if(!criterion(top, bot)) std::swap(top, bot);
        if(!criterion(top, mid)) std::swap(top, mid);
        if(!criterion(mid, bot)) std::swap(mid, bot);

        assert(criterion(top, mid));
        assert(criterion(mid, bot));
        assert(criterion(top, bot));

        auto t = std::get<PositionAttachment>(top);
        auto m = std::get<PositionAttachment>(mid);
        auto b = std::get<PositionAttachment>(bot);

        if(t[1] == m[1]) {
            //Flat top
            rasterizeTrianglePart<Vertex, PositionAttachment, FragmentShader, 1>(top, mid, bot, fragmentShader);
        } else if(m[1] == b[1]) {
            //Flat bottom
            rasterizeTrianglePart<Vertex, PositionAttachment, FragmentShader, -1>(bot, mid, top, fragmentShader);
        } else {
            TupleInterpolator<Vertex> inptb(top, bot);
            Vertex split = inptb.run((m[1] - t[1]) / (b[1] - t[1]));

            rasterizeTrianglePart<Vertex, PositionAttachment, FragmentShader, -1>(mid, split, top, fragmentShader);
            rasterizeTrianglePart<Vertex, PositionAttachment, FragmentShader,  1>(mid, split, bot, fragmentShader);
        }
    }

    template <class VertInType, class VertexShader, class FragmentShader>
    void render(const std::vector<VertInType>& in, const VertexShader& vertexShader,
                                                   const FragmentShader& fragmentShader) {

        static constexpr int POSITION_ATTACHMENT = VertexShader::Traits::POSITION_ATTACHMENT;
        //static constexpr int COLOR_ATTACHMENT = FragmentShader::Traits::COLOR_ATTACHMENT;
        //static constexpr int DEPTH_ATTACHMENT = FragmentShader::Traits::DEPTH_ATTACHMENT;

        typedef typename VertexShader::OutType VertOutFragInType;
        static_assert(std::is_same<FragOutType, typename FragmentShader::OutType>::value, "Error");

        typedef typename VertexShader::VertUniformType VertUniformType;
        typedef typename FragmentShader::FragUniformType FragUniformType;

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
            pos[0] *= this->frameBuffer.sizeX();
            pos[1] *= this->frameBuffer.sizeY();
        });

        //Now in screen space

        for(int i = 0; i < immStore.size(); i+=3) {
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
            rasterizeTriangle<VertOutFragInType, POSITION_ATTACHMENT, FragmentShader>(v1, v2, v3, fragmentShader);
        }
    }

    void clear() {
        for(auto& d: depthBuffer) {
            //d = std::numeric_limits<DepthType>::lowest();
            d = std::numeric_limits<DepthType>::max();
        }
        for(auto& d: frameBuffer) {
            d = DataType(0,0,0,0);
        }
    }

    void readback(cimg_library::CImg<unsigned char>& img) {

        for(int i = 0; i < frameBuffer.size(); ++i) {
            for(int j = 0; j < 3; ++j) {
                img(i%frameBuffer.sizeX(), frameBuffer.sizeY() - 1 - i/frameBuffer.sizeX(), j) = frameBuffer.at(i)[j];
            }
        }
    }

    void readbackDepth(cimg_library::CImg<float>& img) {
        for(int i = 0; i < depthBuffer.size(); ++i) {
            img(i%frameBuffer.sizeX(), frameBuffer.sizeY() - 1 - i/frameBuffer.sizeX(), 0) = depthBuffer.at(i);
        }
    }
};

#endif
