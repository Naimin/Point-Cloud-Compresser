#pragma once
#include <Eigen/dense>

namespace CPC
{
    struct BoundingBox
    {
        BoundingBox();
        void expand(const Eigen::Vector3f& point);

        Eigen::Vector3f min, max;
    };
}
