/*
 * io.h
 *
 *  Created on: Oct 8, 2014
 *      Author: per
 */

#ifndef IO_H_
#define IO_H_

#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include <boost/algorithm/string/predicate.hpp>

#include "veclib.h"

inline VecLib::Vector3f parseVertex(const std::string& str) {
    VecLib::Vector3f v;
    std::istringstream ss(str);

    ss >> v[0];
    ss >> v[1];
    ss >> v[2];

    return v;
}

inline VecLib::Vector2f parseTexcoord(const std::string& str) {
    VecLib::Vector2f v;
    std::istringstream ss(str);

    ss >> v[0];
    ss >> v[1];

    return v;
}

struct Face {
    std::array<unsigned int, 3> coords;
    std::array<unsigned int, 3> uvs;
    std::array<unsigned int, 3> normals;
};

inline Face parseFace(const std::string& str) {
    Face f{};

    const int numberOfSlashes = std::count_if(str.begin(), str.end(), [](const auto& c) { return c == '/'; });

    assert((numberOfSlashes % 3) == 0);

    const int numberOfComponents = (numberOfSlashes / 3) + 1;

    assert(numberOfComponents > 0 && numberOfComponents <= 3);

    std::istringstream ss(str);

    for (int i = 0; i < 3; ++i) {
        ss >> f.coords[i];
        --f.coords[i];

        if (numberOfComponents > 1) {
            ss.ignore();
            ss >> f.uvs[i];
            --f.uvs[i];

            if (numberOfComponents > 2) {
                ss.ignore();
                ss >> f.normals[i];
                --f.normals[i];
            }
        }
    }

    return f;
}

inline void parseObject(const std::string& /*str*/) {
    // std::cout << str << std::endl;
}

inline void parseMtllib(const std::string& /*str*/) {
    // std::cout << str << std::endl;
}

inline void parseMtl(const std::string& /*str*/) {
    // std::cout << str << std::endl;
}

inline void parseSmooth(const std::string& /*str*/) {
    // std::cout << str << std::endl;
}

inline void loadObj(const std::string& filename, std::vector<VecLib::Vector3f>* vertices,
                    std::vector<VecLib::Vector2f>* uvs, std::vector<VecLib::Vector3f>* normals,
                    std::vector<Face>* faces) {
    std::ifstream file(filename);

    std::string line;

    while (std::getline(file, line)) {
        if (line.empty()) {
            // Newline
        } else if (boost::starts_with(line, "#")) {
            // Comment
        }

        else if (boost::starts_with(line, "v ")) {
            vertices->push_back(parseVertex(line.substr(2)));
        }

        else if (boost::starts_with(line, "vn ")) {
            normals->push_back(parseVertex(line.substr(3)));
        }

        else if (boost::starts_with(line, "vt ")) {
            uvs->push_back(parseTexcoord(line.substr(3)));
        }

        else if (boost::starts_with(line, "f ")) {
            faces->push_back(parseFace(line.substr(2)));
        }

        else if (boost::starts_with(line, "o ")) {
            parseObject(line.substr(2));
        }

        else if (boost::starts_with(line, "mtllib ")) {
            parseMtllib(line.substr(7));
        }

        else if (boost::starts_with(line, "usemtl ")) {
            parseMtl(line.substr(7));
        }

        else if (boost::starts_with(line, "s ")) {
            parseSmooth(line.substr(2));
        }

        else {
            std::cout << "Unknown line: " << line << std::endl;
        }
    }
}

#endif /* IO_H_ */
