/*
 * utils.h
 *
 *  Created on: Oct 6, 2014
 *      Author: per
 */

#ifndef UTILS_H_
#define UTILS_H_


inline Eigen::Matrix4f ortho(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m(0,0) = 2.0/(right - left);
    m(1,1) = 2.0/(top - bottom);
    m(2,2) = 2.0/(near - far);

    m(0,3) = (left+right)/(left-right);
    m(1,3) = (bottom+top)/(bottom-top);
    m(2,3) = (far+near)/(far-near);

    return m;
}

inline Eigen::Matrix4f proj(float left, float right,
                             float bottom, float top,
                             float near, float far) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m(0,0) = 2.0f*near/(right - left);
    m(1,1) = 2.0f*near/(top - bottom);
    m(2,2) = 2.0f/(near - far);

    m(0,2) = (right+left)/(right-left);
    m(1,2) = (top+bottom)/(top-bottom);
    m(2,2) = (near+far)/(near-far);
    m(3,2) = -1.0f;

    m(2,3) = 2*near*far / (near-far);

    return m;
}



#endif /* UTILS_H_ */
