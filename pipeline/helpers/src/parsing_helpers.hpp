#pragma once

#include <sstream>
#include <string>
#include <vector>

// Idea: https://stackoverflow.com/questions/10014713/build-tuple-using-variadic-templates
// Fix for proper order: https://stackoverflow.com/questions/14056000/how-to-avoid-undefined-execution-order-for-the-constructors-when-using-stdmake
template<typename T> T read(std::istream& is) {
    T t; is >> t; return t;
}
template<typename T> T read(const std::string& str) {
    std::istringstream is(str);
    return read<T>(is);
}
template<typename... Args>
std::tuple<Args...> stream_to_tuple(std::istream& is) {
    return std::tuple<Args...>{read<Args>(is)...};
}
template<typename... Args>
std::tuple<Args...> string_to_tuple(const std::string& str) {
    std::istringstream is(str);
    return stream_to_tuple<Args...>(is);
}
template<typename T>
std::vector<T> stream_to_vector(std::istream& is) {
    std::vector<T> vals;
    bool first = true;
    while (!is.fail()) {
        T val;
        if (!first) { vals.push_back(val); }
        else { first = false; }
        val = read<T>(is);
    }
    return vals;
}
template<typename T>
std::vector<T> string_to_vector(const std::string& str) {
    std::istringstream is(str);
    return stream_to_vector<T>(is);
}