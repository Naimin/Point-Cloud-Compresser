#pragma once
#include <vector>
#include <Eigen/dense>

namespace PCC
{
    typedef Eigen::Matrix<unsigned char, 1, 1> Vector3u;
    typedef Eigen::Vector3f Vector3f;
    typedef Eigen::Matrix<unsigned int, 3, 1> Vector3ui;

    class PointCloud
    {
        public:
            PointCloud(bool hasNormal = false, bool hasColor = false);
            void resize(size_t size);

            bool isValid();

        public:
            bool hasNormal, hasColor;
            std::vector<Vector3f> positions;
            std::vector<Vector3f> normals;
            std::vector<Vector3u> colors;
    };
}