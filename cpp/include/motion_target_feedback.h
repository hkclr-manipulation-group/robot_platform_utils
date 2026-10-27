#pragma once
#include "platform_state.h"
#include <array>
#include <cstring>
#include <cmath>
#include <vector>

namespace robot::platform::tcp_data {
// System method 12. Versioned, little-endian; no native struct layout on wire.
inline constexpr uint32_t kGetMotionTarget = 12;
struct MotionTargetFeedback {
    uint16_t client = 0;
    uint32_t session = 0, sequence = 0;
    bool accepted = false, clamped = false;
    uint8_t arm_count = 0;
    std::array<uint8_t,MAX_ARM_SIZE> joints{};
    std::array<std::array<float,MAX_ARM_JOINT_SIZE>,MAX_ARM_SIZE> target{}; // joint radians
};
inline std::vector<uint8_t> encodeMotionTarget(const MotionTargetFeedback& m) {
    static_assert(sizeof(float)==4,"motion target wire format requires 32-bit floats");
    if(!m.arm_count || m.arm_count>MAX_ARM_SIZE)return {};
    for(int a=0;a<m.arm_count;++a) {
        if(m.joints[a]>MAX_ARM_JOINT_SIZE)return {};
        for(int j=0;j<m.joints[a];++j)if(!std::isfinite(m.target[a][j]))return {};
    }
    std::vector<uint8_t> b;
    auto put=[&](uint32_t v,int n) { for(int i=0;i<n;++i)b.push_back(uint8_t(v>>(8*i))); };
    put(1,1);put(m.client,2);put(m.session,4);put(m.sequence,4);
    put((m.accepted?1:0)|(m.clamped?2:0),1);put(m.arm_count,1);
    for(int a=0;a<m.arm_count;++a) {
        put(m.joints[a],1);
        for(int j=0;j<m.joints[a];++j) { uint32_t v;std::memcpy(&v,&m.target[a][j],4);put(v,4); }
    }
    return b;
}
inline bool decodeMotionTarget(const std::vector<uint8_t>& b,MotionTargetFeedback& out) {
    if(b.size()<13 || b[0]!=1)return false;
    size_t at=1;
    auto get=[&](int n) { uint32_t v=0;for(int i=0;i<n;++i)v|=uint32_t(b[at++])<<(8*i);return v; };
    MotionTargetFeedback m;
    m.client=uint16_t(get(2));m.session=get(4);m.sequence=get(4);
    const auto flags=get(1);if(flags&~3u)return false;
    m.accepted=flags&1;m.clamped=flags&2;m.arm_count=uint8_t(get(1));
    if(!m.arm_count || m.arm_count>MAX_ARM_SIZE)return false;
    for(int a=0;a<m.arm_count;++a) {
        if(at>=b.size())return false;
        m.joints[a]=uint8_t(get(1));
        if(m.joints[a]>MAX_ARM_JOINT_SIZE || b.size()-at<4*size_t(m.joints[a]))return false;
        for(int j=0;j<m.joints[a];++j) {
            uint32_t v=get(4);std::memcpy(&m.target[a][j],&v,4);
            if(!std::isfinite(m.target[a][j]))return false;
        }
    }
    if(at!=b.size())return false;
    out=m;return true;
}
}
