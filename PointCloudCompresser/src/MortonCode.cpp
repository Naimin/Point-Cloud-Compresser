#include "MortonCode.h"
#include "libmorton/morton.h"

using namespace CPC;

unsigned char split(unsigned char val)
{
    val = val & 0x3F; // only try to split the first 6 bit
    val = (val ^ (val << 2) & 0x49); // 0x49 = 0100 1001
    return val;
}

unsigned char combine(unsigned char val)
{
    val &= 0x49; // 0x49 = 01001001, only keep the relevant bits
    val = (val ^ (val >> 2) & 0xc3); // 0xc3 = 1100 0011
    return val;
}

FullAddress CPC::MortonCode::encode(const Index & index)
{
#ifdef AddressLength64
    return encode64(index);
#endif
#ifdef AddressLength32
    return encode32(index);
#endif
#ifdef AddressLength16
    // not implemented yet
#endif
    return FullAddress();
}

Index CPC::MortonCode::decode(FullAddress address)
{
#ifdef AddressLength64
    return decode64(address);
#endif
#ifdef AddressLength32
    return decode32(address);
#endif
#ifdef AddressLength16
    // not implemented yet
#endif
    return Index(0,0,0);
}

unsigned char CPC::MortonCode::encode8(const Index & index)
{
    return (split(index.x()) | split(index.y() << 1) | split(index.z() << 2));
}

unsigned int CPC::MortonCode::encode32(const Index & index)
{
    return libmorton::morton3D_32_encode(index.x(), index.y(), index.z());
}

unsigned long long CPC::MortonCode::encode64(const Index & index)
{
    return libmorton::morton3D_64_encode(index.x(), index.y(), index.z());
}

Index CPC::MortonCode::decode8(const unsigned char code)
{
    return Index(combine(code), combine(code >> 1), combine(code >> 2));
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