#pragma once
#include "Index.h"
#include <stdint.h>

namespace CPC
{
#define AddressLength32
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

    class MortonCode
    {
        public:
            static FullAddress encode(const Index& index);
            static Index decode(FullAddress address);

            static unsigned char encode8(const Index& index);
            static unsigned int encode32(const Index & index);
            static unsigned long long encode64(const Index & index);
            static Index decode8(const unsigned char code);
            static Index decode32(const unsigned int code);
            static Index decode64(const unsigned long long code);
    };
}