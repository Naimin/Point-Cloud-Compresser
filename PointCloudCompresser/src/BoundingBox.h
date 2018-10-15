#pragma once
#include <Eigen/dense>

namespace PCC
{
    struct BoundingBox
    {
        BoundingBox();
        void expand(const Eigen::Vector3f& point);

        Eigen::Vector3f min, max;
    };
}
