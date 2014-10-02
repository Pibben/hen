#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <tuple>
#include <utility>
#include <vector>

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
class MyInterpolator {
    typedef VertOutFragIn VertOutFragInType;
    VertOutFragIn mStart;
    VertOutFragIn mDelta;
public:
    MyInterpolator(const VertOutFragIn& in1, const VertOutFragIn& in2) {
        mStart = in1;
        mDelta = tupleDiff(in2, in1);
    }
    
    VertOutFragIn run(float val) {
        assert(val >= 0.0);
        assert(val <= 1.0);
        
        return tupleAddScaled(mStart, mDelta, val);
    }
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

template <class VertexShader, class FragmentShader, class Interpolator>
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
    std::array<DataType, 640*480> frameBuffer;
    std::array<DepthType, 640*480> depthBuffer;
    
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

            sort(top, mid, bot);

            Interpolator i01(immStore.at(i+0), immStore.at(i+1));
            Interpolator i02(immStore.at(i+0), immStore.at(i+2));
            Interpolator i12(immStore.at(i+1), immStore.at(i+2));
            
            //TODO: Back face culling
            
            //Rasterization
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
    typedef MyInterpolator<MyVertOutFragInType> MyInterpolatorType;

    MyVertexShaderType vertexShader(0);
    MyFragmentShaderType fragmentShader(0);

    Renderer<MyVertexShaderType, MyFragmentShaderType, MyInterpolatorType> renderer(vertexShader, fragmentShader);
    renderer.render({{Eigen::Vector3f(1,1,1), Eigen::Vector3f(100,100,100)},
                     {Eigen::Vector3f(2,2,2), Eigen::Vector3f(200,200,200)},
                     {Eigen::Vector3f(3,3,3), Eigen::Vector3f(300,300,300)}});
}
