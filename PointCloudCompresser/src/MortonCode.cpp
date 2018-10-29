#include "MortonCode.h"

using namespace CPC;

unsigned int CPC::MortonCode::Encode(const Index & index)
{
    return (part1By2(index.index.z()) << 2) + (part1By2(index.index.y()) << 1) + part1By2(index.index.x());
}

Index CPC::MortonCode::Decode(const unsigned int code)
{
    return Index(compact1By2(code >> 0), compact1By2(code >> 1), compact1By2(code >> 2));
}

unsigned int CPC::MortonCode::part1By2(unsigned int val)
{
    val &= 0x000003ff;                      // val = ---- ---- ---- ---- ---- --98 7654 3210
    val = (val ^ (val << 16)) & 0xff0000ff; // val = ---- --98 ---- ---- ---- ---- 7654 3210
    val = (val ^ (val << 8))  & 0x0300f00f; // val = ---- --98 ---- ---- 7654 ---- ---- 3210
    val = (val ^ (val << 4))  & 0x030c30c3; // val = ---- --98 ---- 76-- --54 ---- 32-- --10
    val = (val ^ (val << 2))  & 0x09249249; // val = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
    return val;
}

unsigned int CPC::MortonCode::compact1By2(unsigned int val)
{
    val &= 0x09249249;                      // val = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
    val = (val ^ (val >> 2))  & 0x030c30c3; // val = ---- --98 ---- 76-- --54 ---- 32-- --10
    val = (val ^ (val >> 4))  & 0x0300f00f; // val = ---- --98 ---- ---- 7654 ---- ---- 3210
    val = (val ^ (val >> 8))  & 0xff0000ff; // val = ---- --98 ---- ---- ---- ---- 7654 3210
    val = (val ^ (val >> 16)) & 0x000003ff; // val = ---- ---- ---- ---- ---- --98 7654 3210
    return val;
}
