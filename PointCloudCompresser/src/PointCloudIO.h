#pragma once
#include <tinyply.h>
#include "PointCloud.h"
#include "Encoder.h"

namespace CPC
{
    class PointCloudIO
    {
        public:
            PointCloudIO();
            ~PointCloudIO();

            PointCloud loadPly(const std::string& path);
            bool savePly(const std::string& path, PointCloud& pointCloud);

            PointCloud loadCpc(const std::string& path);
            bool saveCpc(const std::string& path, EncodedData& encodedData);

        protected:
            template<class T>
            void writeBinary(std::ofstream& fstream, T val)
            {
                fstream.write((char*)&val, sizeof(val));
            }
    };
}