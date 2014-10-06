#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <tuple>
#include <utility>
#include <vector>

#include "CImg.h"
#undef Success

#include <Eigen/Dense>

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

template <class Vertex, class Framebuffer, int PositionAttachment, int ColorAttachment>
static void rasterizeLine(Vertex v1, Vertex v2, Framebuffer framebuffer) {
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
        auto color = std::get<ColorAttachment>(inp.run(inpPos));
        inpPos += inpStep;
        if (steep) {
            framebuffer(y, x) = color;
        } else {
            framebuffer(x, y) = color;
        }

        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
}

template <class Vertex, class Framebuffer, int PositionAttachment, int ColorAttachment>
static void rasterizeTriangleLines(Vertex v1, Vertex v2, Vertex v3,
                                   Framebuffer framebuffer) {
    rasterizeLine<Vertex, Framebuffer, PositionAttachment, ColorAttachment>(v1, v2, framebuffer);
    rasterizeLine<Vertex, Framebuffer, PositionAttachment, ColorAttachment>(v1, v3, framebuffer);
    rasterizeLine<Vertex, Framebuffer, PositionAttachment, ColorAttachment>(v2, v3, framebuffer);
}

template <class Array>
class FramebufferAdapter {
private:
    Array& mArray;
    unsigned int mSizeX;
public:
    FramebufferAdapter(Array& array, unsigned int sizeX) : mArray(array), mSizeX(sizeX) {}

    typename Array::value_type& operator()(unsigned int x, unsigned int y) { return mArray[y*mSizeX+x]; }
    const typename Array::value_type& operator()(unsigned int x, unsigned int y) const { return mArray[y*mSizeX+x]; }
};

template <class In, class InTraits, class Uniform>
class MyVertexShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertInType;
    typedef Uniform VertUniformType;

    typedef std::tuple<Eigen::Vector4f, Eigen::Vector4f> OutType;

    struct Traits {
        static constexpr int POSITION_ATTACHMENT = 0;
        static constexpr int COLOR_ATTACHMENT = 1;
    };
    
    MyVertexShader(const Uniform& uniform) : mUniform(uniform) {}
    
    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector4f& color = std::get<InTraits::COLOR_ATTACHMENT>(in);

        Eigen::Vector4f outPos = mUniform.projMatrix * mUniform.modelViewMatrix * pos;

        return std::make_tuple(outPos, color);
    }
};

template <class In, class InTraits, class Uniform>
class MyFragmentShader {
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
    
    MyFragmentShader(const Uniform& uniform) : mUniform(uniform) {}

    OutType operator()(const In& in) const {
        const Eigen::Vector4f& pos   = std::get<InTraits::POSITION_ATTACHMENT>(in);
        const Eigen::Vector4f& color = std::get<InTraits::COLOR_ATTACHMENT>(in);

        return std::make_tuple(color, pos[2]);
    }
};

template <class Framebuffer>
static void display(const Framebuffer& framebuffer, unsigned int sizeX) {
    cimg_library::CImg<unsigned char> img(sizeX, 480, 1, 3);

    for(int i = 0; i < framebuffer.size(); ++i) {
        for(int j = 0; j < 3; ++j) {
            img(i%sizeX, i/sizeX, j) = framebuffer.at(i)[j];
        }
    }

    cimg_library::CImgDisplay disp(img);
    while(!disp.is_closed()) {
        disp.wait();
    }
}

template <class VertexShader, class FragmentShader>
class Renderer {
private:
    static constexpr int POSITION_ATTACHMENT = VertexShader::Traits::POSITION_ATTACHMENT;
    static constexpr int COLOR_ATTACHMENT = FragmentShader::Traits::COLOR_ATTACHMENT;
    static constexpr int DEPTH_ATTACHMENT = FragmentShader::Traits::DEPTH_ATTACHMENT;

    const VertexShader& mVertexShader;
    const FragmentShader& mFragmentShader;
    typedef typename VertexShader::OutType VertOutFragInType;
    typedef typename FragmentShader::OutType FragOutType;
    
    typedef typename VertexShader::VertUniformType VertUniformType;
    typedef typename FragmentShader::FragUniformType FragUniformType;
    
    typedef typename std::tuple_element<POSITION_ATTACHMENT, VertOutFragInType>::type ScreenPosType;
    typedef typename std::tuple_element<COLOR_ATTACHMENT, FragOutType>::type DataType;
    typedef typename std::tuple_element<DEPTH_ATTACHMENT, FragOutType>::type DepthType;

    std::vector<VertOutFragInType> immStore;
    typedef std::array<DataType, 640*480> FrameBufferType;
    FrameBufferType frameBuffer;
    typedef std::array<DepthType, 640*480> DepthBufferType;
    DepthBufferType depthBuffer;
    
public:
    Renderer(const VertexShader& vertexShader, const FragmentShader& fragmentShader) :
        mVertexShader(vertexShader), mFragmentShader(fragmentShader) {

        for(auto& d: depthBuffer) {
            d = std::numeric_limits<DepthType>::max();
        }
    }

    static void sort(VertOutFragInType& top, VertOutFragInType& mid, VertOutFragInType& bot) {
        auto criterion = [](VertOutFragInType& a, VertOutFragInType& b) {
            return std::get<POSITION_ATTACHMENT>(a)[1] < std::get<POSITION_ATTACHMENT>(b)[1];
        };

        if(!criterion(top, bot)) std::swap(top, bot);
        if(!criterion(top, mid)) std::swap(top, mid);
        if(!criterion(mid, bot)) std::swap(mid, bot);

        assert(criterion(top, mid) && criterion(mid, bot) && criterion(top, bot));
    }

    template <class VertInType>
    void render(const std::vector<VertInType>& in) {
        //Vertex stage
        std::transform(in.begin(), in.end(), std::back_inserter(immStore), mVertexShader);
        
        //Now in clip space
        //Perspective divide
        std::for_each(immStore.begin(), immStore.end(), [](VertOutFragInType& vert) {
            auto& pos = std::get<POSITION_ATTACHMENT>(vert);
            pos /= pos[3];
        });

        //Now in NDC

        std::for_each(immStore.begin(), immStore.end(), [](VertOutFragInType& vert) {
            auto& pos = std::get<POSITION_ATTACHMENT>(vert);
            pos = pos + Eigen::Vector4f::Ones();
            pos /= 2.0f;
            pos[0] *= 640;
            pos[1] *= 480;
        });

        //Now in screen space

        //Primitive assembly
        for(int i = 0; i < immStore.size(); i+=3) {
            //TODO: Back face culling
            
            //Rasterization

            rasterizeTriangleLines<VertOutFragInType, FramebufferAdapter<FrameBufferType>,
                                   VertexShader::Traits::POSITION_ATTACHMENT,
                                   VertexShader::Traits::COLOR_ATTACHMENT>(immStore.at(i+0), immStore.at(i+1), immStore.at(i+2),
                                                                       FramebufferAdapter<FrameBufferType>(frameBuffer, 640));

        }
        display(frameBuffer, 640);
    }
};

static Eigen::Matrix4f ortho(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m(0,0) = 2.0/(right - left);
    m(1,1) = 2.0/(top - bottom);
    m(2,2) = 2.0/(near - far);

    m(0,3) = (left+right)/(left-right);
    m(1,3) = (bottom+top)/(bottom-top);
    m(2,3) = (far+near)/(far-near);

    return m;
}

static Eigen::Matrix4f proj(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m(0,0) = 2.0f*near/(right - left);
    m(1,1) = 2.0f*near/(top - bottom);
    m(2,2) = 2.0f/(near - far);

    m(0,2) = (right+left)/(right-left);
    m(1,2) = (top+bottom)/(top-bottom);
    m(2,2) = (near+far)/(near-far);
    m(3,2) = 1.0f;

    m(2,3) = 2*near*far / (near-far);

    return m;
}


int main(int argc, char** argv) {
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

    vertUniform.projMatrix = Eigen::Matrix4f::Identity();
    vertUniform.modelViewMatrix = proj(-250, 250, -250, 250, 10, 50);

    typedef MyVertexShader<MyVertInType, InTraits, MyVertUniformType> MyVertexShaderType;
    MyVertexShaderType vertexShader(vertUniform);

    //Fragment shader
    struct MyFragUniformType {
    } fragUniform;

    typedef MyFragmentShader<typename MyVertexShaderType::OutType, typename MyVertexShaderType::Traits, MyFragUniformType> MyFragmentShaderType;
    MyFragmentShaderType fragmentShader(fragUniform);

    //Renderer
    Renderer<MyVertexShaderType, MyFragmentShaderType> renderer(vertexShader, fragmentShader);

    //Draw
    std::vector<Eigen::Vector4f> vertices = {Eigen::Vector4f(  0,   0, 12, 1),
                                             Eigen::Vector4f(200,   0, 12, 1),
                                             Eigen::Vector4f(  0, 200, 20, 1),
                                             Eigen::Vector4f(200, 200, 20, 1)};

    std::vector<Eigen::Vector4f> colors = {Eigen::Vector4f(255,0,0,1), //Red
                                           Eigen::Vector4f(0,255,0,1), //Green
                                           Eigen::Vector4f(0,0,255,1), //Blue
                                           Eigen::Vector4f(255,0,255,1)}; //Purple

    renderer.render<MyVertInType>({{vertices[0], colors[0]},
                                   {vertices[1], colors[1]},
                                   {vertices[2], colors[2]},

                                   {vertices[1], colors[1]},
                                   {vertices[2], colors[2]},
                                   {vertices[3], colors[3]}});
}
