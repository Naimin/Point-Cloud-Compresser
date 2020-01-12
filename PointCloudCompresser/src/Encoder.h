#pragma once
#include "Octree.h"
#include <fstream>
#include <limits>

//#define DEBUG_ENCODING
#define AddressLength64

namespace CPC
{
#ifdef AddressLength64
    typedef unsigned long long FullAddress;
    typedef int OffsetAddress; // remember to update the MAX_OFFSET value
    const OffsetAddress MAX_OFFSET = 512;
#endif
#ifdef AddressLength32
    typedef unsigned int FullAddress;
    typedef short OffsetAddress; // remember to update the MAX_OFFSET value
    const OffsetAddress MAX_OFFSET = 16;
#endif
#ifdef AddressLength16
    typedef unsigned short FullAddress;
    typedef char OffsetAddress; // remember to update the MAX_OFFSET value
    const OffsetAddress MAX_OFFSET = 2;
#endif

    // Data the help store and write the encoded data
    struct EncodedData
    {
        EncodedData() : maxDepth(0), currentSize(0) {};
        bool isValid();

        template <class T>
        void add(size_t& pos, T& val)
        {
            memcpy(&(encodedData[pos]), &(val), sizeof(val));
            pos += sizeof(val);
        }

        template <class T>
        void add(T& val)
        {
            add(currentSize, val);
        }

        // return the pointer to the encoded data and advance the size
        unsigned int* addNodeSize()
        {
            unsigned int* nodeSizePtr = (unsigned int*)&(encodedData[currentSize]);
            *nodeSizePtr = 0;
            currentSize += sizeof(unsigned int);
            return nodeSizePtr;
        }

        template <class T>
        void read(size_t& pos, T& val)
        {
            memcpy(&(val), &(encodedData[pos]), sizeof(val));
            pos += sizeof(val);
        }

        template <class T>
        void read(T& val)
        {
            read(currentSize, val);
        }

        unsigned char readNext()
        {
            unsigned char next;
            read(currentSize, next);
            return next;
        }

        bool checkFullAddressFlag()
        {
            // Only peek at the data, don't advance it
            OffsetAddress offsetAddress = *((OffsetAddress*)&(encodedData[currentSize]));
            return (offsetAddress & 0x80000000) != 0;
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
            int getOptimalCodeLength(unsigned char level, size_t & offsetLength, long long & maxOffset);
            FullAddress getEncodedFullAddress(const Index& index);
            OffsetAddress getEncodedOffsetAddress(const Index& index);
    };
}
