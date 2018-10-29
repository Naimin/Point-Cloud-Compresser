#pragma once
#include "Index.h"

namespace CPC
{
    class MortonCode
    {
        public:
            static unsigned int Encode(const Index& index);
            static Index Decode(const unsigned int code);

        protected:
            // helper function
            static unsigned int part1By2(unsigned int val);
            static unsigned int compact1By2(unsigned int val);
    };
}