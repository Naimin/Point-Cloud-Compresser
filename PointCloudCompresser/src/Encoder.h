#pragma once
#include "Octree.h"
#include <fstream>

namespace CPC
{
    // Data the help store and write the encoded data
    struct EncodedData
    {
        EncodedData() : maxDepth(0), currentSize(0) {};
        bool isValid();

        void add(unsigned char val)
        {
            encodedData[currentSize++] = val;
        }
        void add(char val)
        {
            encodedData[currentSize++] = val;
        }
        template <class T>
        void add(T val)
        {
            memcpy(&(encodedData[currentSize]), &val, sizeof(val));
            currentSize += sizeof(val);
        }

        BoundingBox sceneBoundingBox;
        unsigned char maxDepth;
        unsigned char subOctreeDepth;
        std::vector<unsigned char> encodedData;
        size_t currentSize;
    };

    struct TransversalData
    {
        TransversalData(unsigned char level_, Index index_, Node node_) : level(level_), index(index_), node(node_) {}

        unsigned char level;
        Index index;
        Node node;
    };

    struct BestStats
    {
        void checkAndUpdate(size_t size_, size_t level_)
        {
            tbb::mutex::scoped_lock lock(mutex);
            if (size_ < size)
            {
                size = size_;
                level = level_;
            }
        }

        size_t size = std::numeric_limits<size_t>::max();
        size_t level = 0;
        tbb::mutex mutex;
    };

    class Encoder
    {
        public:
            Encoder();
            virtual ~Encoder();

            EncodedData encode(Octree& octree);

        protected:
            void DepthFirstTransversal(Octree& octree, BestStats& bestStats, EncodedData& encodeData);
            BestStats computeBestSubOctreeLevel(Octree& octree);
    };
}