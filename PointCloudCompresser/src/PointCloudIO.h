#pragma once
#include <tinyply.h>
#include "PointCloud.h"

namespace PCC
{
    class PointCloudIO
    {
        public:
            PointCloudIO();
            ~PointCloudIO();

            PointCloud loadPly(const std::string& path);
            bool savePly(const std::string& path, PointCloud& pointCloud);

            PointCloud loadPcc(const std::string& path);
            bool savePcc(const std::string& path, PointCloud& pointCloud);
    };
}