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

        protected:
            void DepthFirstTransversal(EncodedData& data, Octree& octree);
    };

}