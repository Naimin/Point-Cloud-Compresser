#include "MortonCode.h"
#include "libmorton/morton.h"

using namespace CPC;

unsigned int CPC::MortonCode::encode32(const Index & index)
{
    return libmorton::morton3D_32_encode(index.index.x(), index.index.y(), index.index.z());
}

unsigned long long CPC::MortonCode::encode64(const Index & index)
{
    return libmorton::morton3D_64_encode(index.index.x(), index.index.y(), index.index.z());
}

Index CPC::MortonCode::decode32(const unsigned int code)
{
    unsigned int x, y, z;
    libmorton::morton3D_32_decode(code, x, y, z);
    return Index(x,y,z);
}

Index CPC::MortonCode::decode64(const unsigned long long code)
{
    unsigned int x, y, z;
    libmorton::morton3D_64_decode(code, x, y, z);
    return Index(x, y, z);
}