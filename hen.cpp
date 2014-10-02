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

	printf("%f %f %f %f\n", x0, y0, x1, y1);

	TupleInterpolator<Vertex> inp(v1, v2);

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
			//std::cout << x << ", " << y << " = " << color << std::endl;
			framebuffer(y, x) = color;
		} else {
			//std::cout << y << ", " << x << " = " << color << std::endl;
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

template <class In, class Out, class Uniform, int InPositionAttachment = 0, int InColorAttachment = 1,
                                              int OutPositionAttachment = 0, int OutColorAttachment = 1>
class MyVertexShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertInType;
    typedef Out VertOutFragInType;
    typedef Uniform VertUniformType;
    static constexpr int OUT_POSITION_ATTACHMENT = OutPositionAttachment;
    static constexpr int OUT_COLOR_ATTACHMENT = OutColorAttachment;
    
    MyVertexShader(const Uniform& uniform) : mUniform(uniform) {}
    
    Out operator()(const In& in) const {
        const Eigen::Vector3f& pos   = std::get<InPositionAttachment>(in);
        const Eigen::Vector3f& color = std::get<InColorAttachment>(in);

        return std::make_tuple(pos, color);
    }
};

template <class In, class Out, class Uniform, int InPositionAttachment = 0, int InColorAttachment = 1,
                                              int OutColorAttachment = 0, int OutDepthAttachment = 1>
class MyFragmentShader {
private:
public:
    const Uniform& mUniform;
    typedef In VertOutFragInType;
    typedef Out FragOutType;
    typedef Uniform FragUniformType;
    static constexpr int OUT_COLOR_ATTACHMENT = OutColorAttachment;
    static constexpr int OUT_DEPTH_ATTACHMENT = OutDepthAttachment;
    
    MyFragmentShader(const Uniform& uniform) : mUniform(uniform) {}

    Out operator()(const In& in) const {
        const Eigen::Vector3f& pos   = std::get<InPositionAttachment>(in);
        const Eigen::Vector3f& color = std::get<InColorAttachment>(in);

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
    static constexpr int POSITION_ATTACHMENT = VertexShader::OUT_POSITION_ATTACHMENT;
    static constexpr int COLOR_ATTACHMENT = FragmentShader::OUT_COLOR_ATTACHMENT;
    static constexpr int DEPTH_ATTACHMENT = FragmentShader::OUT_DEPTH_ATTACHMENT;

    const VertexShader& mVertexShader;
    const FragmentShader& mFragmentShader;
    typedef typename VertexShader::VertInType VertInType;
    typedef typename VertexShader::VertOutFragInType VertOutFragInType;
    typedef typename FragmentShader::FragOutType FragOutType;
    
    typedef typename VertexShader::VertUniformType VertUniformType;
    typedef typename FragmentShader::FragUniformType FragUniformType;
    
#if 1
    typedef typename std::tuple_element<POSITION_ATTACHMENT, VertOutFragInType>::type ScreenPosType;
    typedef typename std::tuple_element<COLOR_ATTACHMENT, FragOutType>::type DataType;
    typedef typename std::tuple_element<DEPTH_ATTACHMENT, FragOutType>::type DepthType;
#else
    typedef float ScreenPosType;
    typedef float DepthType;
    typedef float DataType;
#endif
    
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

    void render(const std::vector<VertInType>& in) {
        //Vertex stage
        std::transform(in.begin(), in.end(), std::back_inserter(immStore), mVertexShader);
        
        for(int i = 0; i < immStore.size(); i+=3) {
            //Primitive assembly
            VertOutFragInType top = immStore.at(i+0);
            VertOutFragInType mid = immStore.at(i+1);
            VertOutFragInType bot = immStore.at(i+2);
#if 0
            sort(top, mid, bot);

            Interpolator i01(immStore.at(i+0), immStore.at(i+1));
            Interpolator i02(immStore.at(i+0), immStore.at(i+2));
            Interpolator i12(immStore.at(i+1), immStore.at(i+2));
#endif
            //TODO: Back face culling
            
            //Rasterization
#if 1
            rasterizeTriangleLines<VertOutFragInType, FramebufferAdapter<FrameBufferType>,
                                   VertexShader::OUT_POSITION_ATTACHMENT,
                                   VertexShader::OUT_COLOR_ATTACHMENT>(top, mid, bot,
                                		                               FramebufferAdapter<FrameBufferType>(frameBuffer, 640));

            display(frameBuffer, 640);
#else
            Interpolator inp(i01.run(0.3), i12.run(0.7));
            
            VertOutFragInType fragment = inp.run(0.4);

            const int x = std::get<POSITION_ATTACHMENT>(fragment)[0];
            const int y = std::get<POSITION_ATTACHMENT>(fragment)[1];
            
            //Fragment stage
            FragOutType pixel = mFragmentShader(fragment);

            auto depth = std::get<DEPTH_ATTACHMENT>(pixel);
            printf("Depth: %f\n", depth);

            //Frame buffer
            if(depth < depthBuffer.at(640*y+x)) {
                depthBuffer.at(640*y+x) = depth;
                auto data = std::get<COLOR_ATTACHMENT>(pixel);
                std::cout << "Data: " << data << std::endl;
                frameBuffer.at(640*y+x) = data;
            }
#endif
        }
    }
};



int main(int argc, char** argv) {

    typedef std::tuple<Eigen::Vector3f, Eigen::Vector3f> MyVertInType;
    typedef std::tuple<Eigen::Vector3f, Eigen::Vector3f> MyVertOutFragInType;
    typedef std::tuple<Eigen::Vector3f, float> MyFragOutType;

    typedef int MyVertUniformType;
    typedef int MyFragUniformType;

    typedef MyVertexShader<MyVertInType, MyVertOutFragInType, MyVertUniformType> MyVertexShaderType;
    typedef MyFragmentShader<MyVertOutFragInType, MyFragOutType, MyFragUniformType> MyFragmentShaderType;

    MyVertexShaderType vertexShader(0);
    MyFragmentShaderType fragmentShader(0);

    Renderer<MyVertexShaderType, MyFragmentShaderType> renderer(vertexShader, fragmentShader);
    renderer.render({{Eigen::Vector3f( 10, 10, 5), Eigen::Vector3f(255,0,0)},
                     {Eigen::Vector3f(300, 20, 5), Eigen::Vector3f(0,255,0)},
                     {Eigen::Vector3f(300,300, 5), Eigen::Vector3f(0,0,255)}});
}
