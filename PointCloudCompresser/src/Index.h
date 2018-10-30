#pragma once
#include <Eigen/dense>

namespace CPC
{
    typedef Eigen::Matrix<unsigned int, 3, 1> Vector3ui;

    struct Index : public Vector3ui
    {
        Index(const Vector3ui& index_) : Vector3ui(index_) {}
        Index(const unsigned int x, const unsigned int y, const unsigned int z) : Vector3ui(x, y, z) {}
        bool operator < (const Index &b) const
        {
            if (x() != b.x()) {
                return x() < b.x();
            }
            if (y() != b.y()) {
                return y() < b.y();
            }
            return  z() < b.z();
        }
    };
}