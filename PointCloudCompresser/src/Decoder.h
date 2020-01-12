#pragma once
#include "Encoder.h"
#include <set>

namespace CPC
{
    struct DecoderTransversalData : TransversalData
    {
        DecoderTransversalData(unsigned char level_, Index index_, Node node_) : TransversalData(level_, index_, node_) {}
        void processChild(unsigned char child);
    };

    enum intersectionState
    {
        ALREADY_EXIST = 0,
        SUBNODE_NOT_FOUND,
        DECODE_NOT_FOUND,
        DECODE_FOUND
    };

    class Decoder
    {
        public:
            Decoder();
            virtual ~Decoder();

            bool intersect(const Eigen::Vector3f& point, EncodedData& data, Octree& octree, std::map<Index, size_t>& subNodePos, intersectionState& state);

            Octree decode(EncodedData& data);
            std::map<Index, size_t> decodeNodeHeaders(EncodedData& data);
            void decodeNodeHeader(size_t& pos, Index& index, EncodedData& data, size_t& nodeSize);
            void decodeNode(size_t& pos, const Index& index, EncodedData& data, Octree& octree);

        protected:
            void DepthFirstTransversal(EncodedData& data, Octree& octree);
            Index decodedFullAddress(const FullAddress& index);
            Eigen::Vector3i decodedOffsetAddress(const OffsetAddress& index);
            
            std::set<Index> decodedNodes;
    };

}