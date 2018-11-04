#pragma once
#include "Octree.h"
#include <fstream>
#include <limits>

//#define DEBUG_ENCODING

namespace CPC
{
    typedef unsigned long long FullAddress;
    typedef int OffsetAddress; // remember to update the MAX_OFFSET value
    const OffsetAddress MAX_OFFSET = 512;

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
            unsigned char* raw = (unsigned char*)&val;
            for (int i = sizeof(val)-1; i > -1; --i)
            {
                add(raw[i]);
            }
        }

        template <class T>
        void read(T& val)
        {
            T raw;
            memcpy(&(raw), &(encodedData[currentSize]), sizeof(val));
            unsigned char* rawPtr = (unsigned char*)&raw;
            unsigned char* valPtr = (unsigned char*)&val;
            for (int i = sizeof(val) - 1; i > -1; --i)
            {
                valPtr[i] = rawPtr[sizeof(val) - 1 - i];
            }
            currentSize += sizeof(val);
        }

        unsigned char readNext()
        {
            return encodedData[currentSize++];
        }

        bool checkFullAddressFlag()
        {
            // Only peek at the data, don't advance it
            return (encodedData[currentSize] & 0x80) != 0;
        }

        void resize(size_t size)
        {
            encodedData.resize(size);
        }

        void shrink()
        {
            if (encodedData.size() != currentSize)
            {
                encodedData.resize(currentSize);
                encodedData.shrink_to_fit();
            }
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
        BestStats() : size(ULLONG_MAX), level(0) {}
        BestStats(size_t size_, unsigned level_) : size(size_), level(level_) {}
        BestStats(const BestStats& right) : size(right.size), level(right.level) {}
        BestStats& operator=(const BestStats& right)
        {
            this->size = right.size;
            this->level = right.level;
            return *this;
        }

        void checkAndUpdate(size_t size_, unsigned char level_)
        {
            tbb::mutex::scoped_lock lock(mutex);
            if (size_ < size)
            {
                size = size_;
                level = level_;
            }
        }

        size_t size;
        unsigned char level;
        tbb::mutex mutex;
    };

    class Encoder
    {
        public:
            Encoder();
            virtual ~Encoder();

            EncodedData encode(Octree& octree, unsigned char forceSubOctreeLevel = (unsigned char)-1);

        protected:
            void DepthFirstTransversal(Octree& octree, BestStats& bestStats, EncodedData& encodeData);
            BestStats computeBestSubOctreeLevel(Octree& octree);
            size_t computeSubOctreeSize(Octree & octree, unsigned char level);
            FullAddress getEncodedFullAddress(const Index& index);
            OffsetAddress getEncodedOffsetAddress(const Index& index);
    };
}
