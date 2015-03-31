/*
 * samplers.h
 *
 *  Created on: Mar 31, 2015
 *      Author: per
 */

#ifndef STDCOMP_SAMPLERS_H_
#define STDCOMP_SAMPLERS_H_

//TODO: Share code

template <class OutType>
class TextureSampler {
    cimg_library::CImg<unsigned char> mImg;
    unsigned int mSizeX;
    unsigned int mSizeY;

public:
    TextureSampler() : mImg(), mSizeX(-1), mSizeY(-1) {}
    TextureSampler(cimg_library::CImg<unsigned char>& img) : mImg(img), mSizeX(img.width()), mSizeY(img.height()) {}

    OutType get(float u, float v) const {
        int x = u * (mSizeX-1) + 0.5;
        int y = mSizeY - (v * (mSizeY-1) + 0.5);

        assert(x >= 0 && x < mSizeX);
        assert(y >= 0 && y < mSizeY);

        const float r = mImg(x, y, 0);
        const float g = mImg(x, y, 1);
        const float b = mImg(x, y, 2);
        const float a = 255.0;

        return OutType(r, g, b, a);
    }
};

template <class OutType>
class CubeSampler {
    cimg_library::CImg<unsigned char> mImg; //TODO
    unsigned int mSizeX;
    unsigned int mSizeY;

public:
    CubeSampler() : mImg(), mSizeX(-1), mSizeY(-1) {}
    CubeSampler(cimg_library::CImg<unsigned char>& img) : mImg(img), mSizeX(img.width()), mSizeY(img.height()) {}

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

        int x = u * (mSizeX-1) + 0.5;
        int y = mSizeY - (v * (mSizeY-1) + 0.5);

        assert(x >= 0 && x < mSizeX);
        assert(y >= 0 && y < mSizeY);

        const float r = mImg(x, y, 0);
        const float g = mImg(x, y, 1);
        const float b = mImg(x, y, 2);
        const float a = 255.0;

        return OutType(r, g, b, a);
    }
};




#endif /* STDCOMP_SAMPLERS_H_ */
