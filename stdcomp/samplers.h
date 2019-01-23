/*
 * samplers.h
 *
 *  Created on: Mar 31, 2015
 *      Author: per
 */

#ifndef STDCOMP_SAMPLERS_H_
#define STDCOMP_SAMPLERS_H_

// TODO: Share code

template <class OutType>
class RGBATextureSampler {
    PixelBuffer<uint8_t> mImg;
    unsigned int mSizeX;
    unsigned int mSizeY;

public:
    //RGBATextureSampler() : mImg(), mSizeX(-1), mSizeY(-1) {}
    RGBATextureSampler(PixelBuffer<uint8_t>& img) : mImg(std::move(img)), mSizeX(img.width()), mSizeY(img.height()) {}
    RGBATextureSampler(const std::string& filename) : mImg(loadPng(filename)), mSizeX(mImg.width()), mSizeY(mImg.height()) {}

    OutType get(float u, float v) const {
        int x = u * (mSizeX-1) + 0.5f;
        int y = mSizeY - (v * (mSizeY-1) + 0.5f);

        assert(x >= 0 && x < (int)mSizeX);
        assert(y >= 0 && y < (int)mSizeY);

        const float r = mImg(x, y, 0) / 255.0f;
        const float g = mImg(x, y, 1) / 255.0f;
        const float b = mImg(x, y, 2) / 255.0f;
        const float a = 1.0f;

        return OutType(r, g, b, a);
    }

    unsigned int width() const { return mSizeX; }
    unsigned int height() const { return mSizeY; }
};
#if 0
template <class OutType, class StorageType=uint8_t>
class SingleChannelTextureSampler {
    cimg_library::CImg<StorageType> mImg;
    unsigned int mSizeX;
    unsigned int mSizeY;

public:
    SingleChannelTextureSampler() : mImg(), mSizeX(-1), mSizeY(-1) {}
    SingleChannelTextureSampler(unsigned int sizeX, unsigned int sizeY) : mImg(cimg_library::CImg<StorageType>(sizeX, sizeY)), mSizeX(sizeX), mSizeY(sizeY) {}
    SingleChannelTextureSampler(cimg_library::CImg<StorageType>& img) : mImg(img), mSizeX(img.width()), mSizeY(img.height()) {}
    SingleChannelTextureSampler(const std::string& filename) {
        mImg.load(filename.c_str());

        mSizeX = mImg.width();
        mSizeY = mImg.height();
    }

    OutType get(float u, float v) const {
        int x = u * (mSizeX-1) + 0.5;
        int y = mSizeY - (v * (mSizeY-1) + 0.5);

        assert(x >= 0 && x < (int)mSizeX);
        assert(y >= 0 && y < (int)mSizeY);

        return static_cast<OutType>(mImg(x, y, 0)) / 255.0f;
    }

    cimg_library::CImg<StorageType>& texture() { return mImg; }
};

template <class OutType>
class CubeSampler {
private:
    RGBATextureSampler<OutType> mTextureMap;

public:
    CubeSampler() : mTextureMap() {}
    CubeSampler(cimg_library::CImg<unsigned char>& img) : mTextureMap(img) {}
    CubeSampler(const std::string& filename) : mTextureMap(filename) {}

    OutType get(float vx, float vy, float vz) const {
        const float ax = std::abs(vx);
        const float ay = std::abs(vy);
        const float az = std::abs(vz);

        float uOffset = 0;
        float vOffset = 0;

        float u = 0;
        float v = 0;

        if(ax >= ay && ax >= az) {
            //X biggest

            if(vx > 0) {
                uOffset = 2.0;
                vOffset = 1.0 ;
            } else {
                uOffset = 0;
                vOffset = 1.0;
            }

            u = (vz / -vx + 1.0 + uOffset * 2.0) / 8.0;
            v = (vy / ax + 1.0 + vOffset * 2.0) / 6.0;
        } else if(ay >= ax && ay >= az) {
            //Y biggest

            uOffset = 1.0;
            if(vy > 0) {
                vOffset = 2.0;
            } else {
                vOffset = 0;
            }

            u = (vx / ay + 1.0 + uOffset * 2.0) / 8.0;
            v = (vz / -vy + 1.0 + vOffset * 2.0) / 6.0;
        } else if(az >= ax && az >= ay) {
            //Z biggest

            vOffset = 1.0;
            if(vz > 0) {
                uOffset = 1.0;
            } else {
                uOffset = 3.0;
            }

            u = (vx / vz + 1.0 + uOffset * 2.0) / 8.0;
            v = (vy / az + 1.0 + vOffset * 2.0) / 6.0;
        }

        return mTextureMap.get(u, v);
    }
};

#endif

#endif /* STDCOMP_SAMPLERS_H_ */
