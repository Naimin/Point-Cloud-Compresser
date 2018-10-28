#include "MortonCode.h"

using namespace CPC;

unsigned int CPC::MortonCode::Encode(Index & index)
{
    return (part1By2(index.index.z()) << 2) + (part1By2(index.index.y()) << 1) + part1By2(index.index.x());
}

Index CPC::MortonCode::Decode(unsigned int code)
{
    return Index(compact1By2(code >> 0), compact1By2(code >> 1), compact1By2(code >> 2));
}

unsigned int CPC::MortonCode::part1By2(unsigned int val)
{
    val &= 0x55555555;                  // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
    val = (val ^ (val >> 1)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
    val = (val ^ (val >> 2)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
    val = (val ^ (val >> 4)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
    val = (val ^ (val >> 8)) & 0x0000ffff; // x = ---- ---- ---- ---- fedc ba98 7654 3210
    return val;
}

unsigned int CPC::MortonCode::compact1By2(unsigned int val)
{
    val &= 0x09249249;                  // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
    val = (val ^ (val >> 2)) & 0x030c30c3; // x = ---- --98 ---- 76-- --54 ---- 32-- --10
    val = (val ^ (val >> 4)) & 0x0300f00f; // x = ---- --98 ---- ---- 7654 ---- ---- 3210
    val = (val ^ (val >> 8)) & 0xff0000ff; // x = ---- --98 ---- ---- ---- ---- 7654 3210
    val = (val ^ (val >> 16)) & 0x000003ff; // x = ---- ---- ---- ---- ---- --98 7654 3210
    return val;
}
