#pragma once
#include "Encoder.h"

namespace CPC
{

    struct DecoderTransversalData : TransversalData
    {
        DecoderTransversalData(unsigned char level_, Index index_, Node node_) : TransversalData(level_, index_, node_) {}
        void processChild(unsigned char child);
    };

    class Decoder
    {
        public:
            Decoder();
            virtual ~Decoder();

            Octree decode(EncodedData& data);
            void decodeNodeHeader(size_t& pos, Index& index, EncodedData& data, size_t& nodeSize);
            void decodeNode(size_t& pos, const Index& index, EncodedData& data, Octree& octree);

        protected:
            void DepthFirstTransversal(EncodedData& data, Octree& octree);
            Index decodedFullAddress(const FullAddress& index);
            Eigen::Vector3i decodedOffsetAddress(const OffsetAddress& index);
            
            std::set<Index> decodedNodes;
    };

}