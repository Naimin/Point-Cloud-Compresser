#pragma once
#include <Eigen/dense>

namespace CPC
{
    typedef Eigen::Matrix<unsigned int, 3, 1> Vector3ui;

    struct Index
    {
        Index(const Vector3ui& index_) : index(index_) {}
        Index(const unsigned int x, const unsigned int y, const unsigned int z) : index(x, y, z) {}
        bool operator < (const Index &b) const
        {
            if (index.x() != b.index.x()) {
                return index.x() < b.index.x();
            }
            if (index.y() != b.index.y()) {
                return index.y() < b.index.y();
            }
            return  index.z() < b.index.z();
        }

        Vector3ui index;
    };
}