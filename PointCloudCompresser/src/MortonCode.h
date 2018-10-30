#pragma once
#include "Index.h"
#include <stdint.h>

namespace CPC
{
    class MortonCode
    {
        public:
            static unsigned int encode32(const Index & index);
            static unsigned long long encode64(const Index & index);
            static Index decode32(const unsigned int code);
            static Index decode64(const unsigned long long code);    
    };
}