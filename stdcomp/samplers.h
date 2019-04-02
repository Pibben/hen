/*
 * samplers.h
 *
 *  Created on: Mar 31, 2015
 *      Author: per
 */

#ifndef STDCOMP_SAMPLERS_H_
#define STDCOMP_SAMPLERS_H_

// TODO: Share code

#include <io.h>

template <class OutType>
class RGBTextureSampler {
private:
    PixelBuffer<uint8_t> mImg;

public:
    explicit RGBTextureSampler(const std::string& filename) : mImg(loadPng(filename)) {}
    explicit RGBTextureSampler(uint8_t* pixels, uint16_t width, uint16_t height) : mImg(pixels, width, height, 4) {}

    OutType get(float u, float v) const {
        const uint_fast16_t sizeX = mImg.width();
        const uint_fast16_t sizeY = mImg.height();

        const uint_fast16_t x = static_cast<uint_fast16_t>(        u * sizeX) % sizeX;  // Wrap mode repeat
        const uint_fast16_t y = static_cast<uint_fast16_t>(sizeY - v * sizeY) % sizeY;  // Wrap mode repeat

        const float r = mImg(x, y, 0) / 255.0f;
        const float g = mImg(x, y, 1) / 255.0f;
        const float b = mImg(x, y, 2) / 255.0f;

        return OutType(r, g, b);
    }
};

static constexpr bool checkPow2(uint16_t n) {
    return (n & (n - 1)) == 0;
}

template <class OutType>
class RGBATextureSampler {
private:
    PixelBuffer<uint8_t> mImg;

public:
    explicit RGBATextureSampler(const std::string& filename) : mImg(loadPng(filename)) {}
    explicit RGBATextureSampler(uint8_t* pixels, uint16_t width, uint16_t height) : mImg(pixels, width, height, 4) {
        assert(checkPow2(width));
        assert(checkPow2(height));
    }

    OutType get(float u, float v) const {
        const uint_fast16_t sizeX = mImg.width();
        const uint_fast16_t sizeY = mImg.height();

        const uint_fast16_t x = static_cast<uint_fast16_t>(        u * sizeX) & (sizeX - 1);  // Wrap mode repeat
        const uint_fast16_t y = static_cast<uint_fast16_t>(sizeY - v * sizeY) & (sizeY - 1);  // Wrap mode repeat

        const float r = mImg(x, y, 0) / 255.0f;
        const float g = mImg(x, y, 1) / 255.0f;
        const float b = mImg(x, y, 2) / 255.0f;
        const float a = mImg(x, y, 3) / 255.0f;

        return OutType(r, g, b, a);
    }
};

template <class OutType, class StorageType = uint8_t>
class SingleChannelTextureSampler {
    PixelBuffer<StorageType> mImg;

public:
    explicit SingleChannelTextureSampler(const std::string& filename) : mImg(loadPng(filename)) {}

    OutType get(float u, float v) const {
        const uint_fast16_t sizeX = mImg.width();
        const uint_fast16_t sizeY = mImg.height();

        const uint_fast16_t x = static_cast<uint_fast16_t>(        u * sizeX) % sizeX;  // Wrap mode repeat
        const uint_fast16_t y = static_cast<uint_fast16_t>(sizeY - v * sizeY) % sizeY;  // Wrap mode repeat

        return static_cast<OutType>(mImg(x, y, 0)) / 255.0f;
    }
};

template <class OutType>
class CubeSampler {
private:
    RGBTextureSampler<OutType> mTextureMap;

public:
    explicit CubeSampler(const std::string& filename) : mTextureMap(filename) {}

    OutType get(float vx, float vy, float vz) const {
        const float ax = std::abs(vx);
        const float ay = std::abs(vy);
        const float az = std::abs(vz);

        float uOffset;
        float vOffset;

        float u = 0.0f;
        float v = 0.0f;

        if (ax >= ay && ax >= az) {
            // X biggest

            if (vx > 0.0f) {
                uOffset = 2.0f;
                vOffset = 1.0f;
            } else {
                uOffset = 0.0f;
                vOffset = 1.0f;
            }

            u = (vz / -vx + 1.0f + uOffset * 2.0f) / 8.0f;
            v = (vy / ax + 1.0f + vOffset * 2.0f) / 6.0f;
        } else if (ay >= ax && ay >= az) {
            // Y biggest

            uOffset = 1.0f;
            if (vy > 0.0f) {
                vOffset = 2.0f;
            } else {
                vOffset = 0.0f;
            }

            u = (vx / ay + 1.0f + uOffset * 2.0f) / 8.0f;
            v = (vz / -vy + 1.0f + vOffset * 2.0f) / 6.0f;
        } else if (az >= ax && az >= ay) {
            // Z biggest

            vOffset = 1.0f;
            if (vz > 0.0f) {
                uOffset = 1.0f;
            } else {
                uOffset = 3.0f;
            }

            u = (vx / vz + 1.0f + uOffset * 2.0f) / 8.0f;
            v = (vy / az + 1.0f + vOffset * 2.0f) / 6.0f;
        }

        return mTextureMap.get(u, v);
    }
};

#endif /* STDCOMP_SAMPLERS_H_ */
