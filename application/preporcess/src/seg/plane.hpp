#pragma once
#include "../base/common.hpp"
struct Plane_{
    Eigen::Vector3f a;
    Eigen::Vector3f b;
    Eigen::Vector3f c;
    Eigen::Vector3f d;
};

struct Plane_1{
    float x, y, z;  // 位置
    float length, width; // 尺寸
    float nx, ny, nz;   // 法线向量
};