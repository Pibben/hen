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

#include "png.h"

#include "utils.h"
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

class FileWrapper {
public:
    explicit FileWrapper(const std::string& filename) {
        mFile = fopen(filename.c_str(), "rbe");  // NOLINT
    }
    ~FileWrapper() {
        if (mFile != nullptr) {
            fclose(mFile);  // NOLINT
        }
    }
    FileWrapper(const FileWrapper&) = delete;
    FileWrapper(FileWrapper&&) = delete;
    FileWrapper& operator=(const FileWrapper&) = delete;
    FileWrapper& operator=(FileWrapper&&) = delete;

    size_t read(char* ptr, size_t size) { return fread(ptr, 1, size, mFile); }

    FILE* getFile() { return mFile; }

    explicit operator bool() { return mFile != nullptr; }

private:
    FILE* mFile{nullptr};
};

inline PixelBuffer<uint8_t> loadPng(const std::string& filename) {
    FileWrapper ifs(filename);
    assert(ifs);
    std::vector<unsigned char> ingress(8);
    ifs.read(reinterpret_cast<char*>(ingress.data()), 8);
    assert(!png_sig_cmp(ingress.data(), 0, 8));

    png_voidp user_error_ptr = nullptr;
    png_error_ptr user_error_fn = nullptr;
    png_error_ptr user_warning_fn = nullptr;
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, user_error_ptr, user_error_fn, user_warning_fn);
    assert(png_ptr);

    png_infop info_ptr = png_create_info_struct(png_ptr);
    assert(info_ptr);

    png_infop end_info = png_create_info_struct(png_ptr);
    assert(end_info);

    const bool ok = !setjmp(png_jmpbuf(png_ptr));  // NOLINT
    assert(ok);

    png_init_io(png_ptr, ifs.getFile());
    png_set_sig_bytes(png_ptr, 8);

    png_read_info(png_ptr, info_ptr);
    png_uint_32 W, H;
    int bit_depth, color_type, interlace_type;
    bool is_gray = false;
    png_get_IHDR(png_ptr, info_ptr, &W, &H, &bit_depth, &color_type, &interlace_type, nullptr, nullptr);

    printf("%d x %d @ %d\n", W, H, bit_depth);

    if (color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(png_ptr);
        color_type = PNG_COLOR_TYPE_RGB;
        bit_depth = 8;
    } else if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) {
        png_set_expand_gray_1_2_4_to_8(png_ptr);
        is_gray = true;
        bit_depth = 8;
    }

    if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS) != 0u) {
        png_set_tRNS_to_alpha(png_ptr);
        color_type |= PNG_COLOR_MASK_ALPHA;
    }

    if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(png_ptr);
        color_type |= PNG_COLOR_MASK_COLOR;
        is_gray = true;
    }

    if (color_type == PNG_COLOR_TYPE_RGB) {
        png_set_filler(png_ptr, 0xffffU, PNG_FILLER_AFTER);
    }

    png_read_update_info(png_ptr, info_ptr);

    assert(bit_depth == 8);

    const int byte_depth = bit_depth >> 3;

    auto* const imgData = new png_bytep[H];  // NOLINT
    for (unsigned int row = 0; row < H; ++row) {
        imgData[row] = new png_byte[static_cast<size_t>(byte_depth) * 4 * W];  // NOLINT
    }
    png_read_image(png_ptr, imgData);
    png_read_end(png_ptr, end_info);

    assert(color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_RGB_ALPHA);

    assert(!is_gray);

    PixelBuffer<uint8_t> pb(W, H, byte_depth * 4);

    for (uint_fast16_t y = 0; y < H; ++y) {
        const unsigned char* ptrs = static_cast<unsigned char*>(imgData[y]);
        for (uint_fast16_t x = 0; x < W; ++x) {
            pb(x, y, 0) = *ptrs++;
            pb(x, y, 1) = *ptrs++;
            pb(x, y, 2) = *ptrs++;
            pb(x, y, 3) = *ptrs++;
        }
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);

    for (uint_fast16_t y = 0; y < H; ++y) {
        delete[] imgData[y];  // NOLINT
    }

    delete[] imgData;  // NOLINT

    return std::move(pb);
}

#endif /* IO_H_ */
