hen
===

Header-only programmable C++14 software 3D renderer.

### New fold expression tuple transform
```
template <typename... Tp, typename Func, std::size_t... Idx>
std::tuple<Tp...> mytransformHelper(const std::tuple<Tp...>& t1, const std::tuple<Tp...>& t2, Func&& f, std::index_sequence<Idx...>) {
    return std::tuple<Tp...>((f(std::get<Idx>(t1), std::get<Idx>(t2)))...);
}

template<typename FuncT, typename... Tp>
inline void
mytransform(const std::tuple<Tp...>& t1, const std::tuple<Tp...>& t2, std::tuple<Tp...>& r, FuncT f) {
    r = mytransformHelper(t1, t2, f, std::index_sequence_for<Tp...>{});
}
```
