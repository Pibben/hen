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
#include <Eigen/Dense>

inline Eigen::Vector3f parseVertex(const std::string& str) {
    Eigen::Vector3f v;
    std::istringstream ss(str);

    ss >> v[0];
    ss >> v[1];
    ss >> v[2];

    return v;
}

inline Eigen::Vector2f parseTexcoord(const std::string& str) {
    Eigen::Vector2f v;
    std::istringstream ss(str);

    ss >> v[0];
    ss >> v[1];

    return v;
}

typedef std::pair<std::array<unsigned int, 3>, std::array<unsigned int, 3>> Face;

inline Face parseFace(const std::string& str) {
    Face f;
    std::istringstream ss(str);
#if 1
    for(int i = 0; i < 3; ++i) {
        ss >> f.first[i];
        ss.ignore();
        ss >> f.second[i];

        --f.first[i];
        --f.second[i];
    }
#else
    ss >> f.first[0];
    --f.first[0];
    ss >> f.first[1];
    --f.first[1];
    ss >> f.first[2];
    --f.first[2];
#endif

    return f;

}

inline void parseObject(const std::string& str) {
    //std::cout << str << std::endl;
}

inline void parseMtllib(const std::string& str) {
    //std::cout << str << std::endl;
}

inline void parseMtl(const std::string& str) {
    //std::cout << str << std::endl;
}

inline void parseSmooth(const std::string& str) {
    //std::cout << str << std::endl;
}


inline void loadObj(const std::string& filename,
                    std::vector<Eigen::Vector3f>& vertices,
                    std::vector<Eigen::Vector2f>& uvs,
                    std::vector<Face>& faces) {
    std::ifstream file(filename);


    std::string line;

    while(std::getline(file, line)) {
        if(boost::starts_with(line, "#")) {
            //Comment
        }

        else if(boost::starts_with(line, "v ")) {
            vertices.push_back(parseVertex(line.substr(2)));
        }
        else if(boost::starts_with(line, "vt ")) {
            uvs.push_back(parseTexcoord(line.substr(3)));
        }

        else if(boost::starts_with(line, "f ")) {
            faces.push_back(parseFace(line.substr(2)));
        }

        else if(boost::starts_with(line, "o ")) {
            parseObject(line.substr(2));
        }

        else if(boost::starts_with(line, "mtllib ")) {
            parseMtllib(line.substr(7));
        }

        else if(boost::starts_with(line, "usemtl ")) {
            parseMtl(line.substr(7));
        }

        else if(boost::starts_with(line, "s ")) {
            parseSmooth(line.substr(2));
        }

        else {
            //std::cout << "Unknown line: " << line << std::endl;
        }
    }
}



#endif /* IO_H_ */
