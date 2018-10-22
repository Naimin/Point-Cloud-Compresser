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

            EncodedData loadCpc(const std::string& path);
            bool saveCpc(const std::string& path, EncodedData& encodedData);

        protected:
            template<class T>
            void writeBinary(std::ofstream& fstream, T val)
            {
                fstream.write((char*)&val, sizeof(val));
            }

            template<class T>
            void readBinary(std::ifstream& fstream, T& val)
            {
                fstream.read((char*)&val, sizeof(T));
            }
    };
}