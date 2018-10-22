#pragma once
#include "Encoder.h"

namespace CPC
{

    struct DecoderTransversalData : TransversalData
    {
        
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