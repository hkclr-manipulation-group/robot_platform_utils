#pragma once
#include "tcp_recovery_protocol.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace robot::platform::tcp_data::encoder_baseline {
constexpr uint32_t kVersion = 1;
constexpr std::size_t kJoints = 64, kMaxBody = 8192;
constexpr uint32_t kGetStatus = 13; // System service (4), read-only
using BootId = recovery::BootId;
enum class Phase : uint8_t { Idle=0, WaitingForHold=1, WaitingForStable=2, Committing=3, Succeeded=4, Failed=5 };
enum class Reason : uint8_t { None=0, NotRecovery=1, Hardware=2, ActiveOutput=3,
    Feedback=4, Moving=5, ContextChanged=6, Persist=7, Timeout=8 };
struct State {
    uint32_t version=kVersion; BootId boot{};
    uint64_t configuration=0, fault=0, stop=0, attempt=0, commit_sequence=0, sampled_us=0;
    Phase phase=Phase::Idle; Reason reason=Reason::None;
    uint8_t required=0, saved_valid=0, count=0, arm_count=0;
    std::array<uint8_t,2> joint_counts{};
    std::array<uint8_t,kJoints> present{};
    std::array<float,kJoints> positions{}, saved{};
    std::array<uint64_t,kJoints> sample_us{};
};
template<class V> void fields(V& v, State& x) {v(x.version,x.boot,x.configuration,x.fault,x.stop,
    x.attempt,x.commit_sequence,x.sampled_us,x.phase,x.reason,x.required,x.saved_valid,x.count,x.arm_count,
    x.joint_counts,x.present,x.positions,x.saved,x.sample_us);}
namespace codec {
struct Writer {
    std::vector<uint8_t>& out;
    template<class... T> void operator()(T&... x) {(value(x),...);}
    template<class T,std::size_t N> void value(std::array<T,N>& a) {for(auto& x:a)value(x);}
    void value(BootId& x) {value(x.high);value(x.low);}
    template<class T> void value(T& x) {
        if constexpr(std::is_same_v<T,float>) {uint32_t n;std::memcpy(&n,&x,4);value(n);}
        else if constexpr(std::is_enum_v<T>) {auto n=static_cast<std::underlying_type_t<T>>(x);value(n);}
        else if constexpr(std::is_integral_v<T>) {for(size_t i=0;i<sizeof(T);++i)out.push_back(uint8_t(x>>(8*i)));}
        else fields(*this,x);
    }
};
struct Reader {
    const uint8_t* data;size_t size,pos=0;bool ok=true;
    template<class... T> void operator()(T&... x) {(value(x),...);}
    template<class T,std::size_t N> void value(std::array<T,N>& a) {for(auto& x:a)value(x);}
    void value(BootId& x) {value(x.high);value(x.low);}
    template<class T> void value(T& x) {
        if constexpr(std::is_same_v<T,float>) {uint32_t n=0;value(n);std::memcpy(&x,&n,4);}
        else if constexpr(std::is_enum_v<T>) {std::underlying_type_t<T> n{};value(n);x=static_cast<T>(n);}
        else if constexpr(std::is_integral_v<T>) {
            if(pos+sizeof(T)>size){ok=false;return;}x=0;
            for(size_t i=0;i<sizeof(T);++i)x|=T(data[pos++])<<(8*i);
        } else fields(*this,x);
    }
};
inline bool valid(const State& x) {
    if(x.version!=kVersion || !x.boot.valid() || x.count>kJoints || x.arm_count>2 ||
        x.required>1 || x.saved_valid>1 || unsigned(x.phase)>5 || unsigned(x.reason)>8)return false;
    if(x.count!=x.joint_counts[0]+x.joint_counts[1])return false;
    if(x.joint_counts[0]>MAX_ARM_JOINT_SIZE || x.joint_counts[1]>MAX_ARM_JOINT_SIZE ||
        (x.arm_count<2 && x.joint_counts[1]) || (!x.arm_count && x.joint_counts[0]))return false;
    for(size_t i=0;i<kJoints;++i)if(x.present[i]>1 || !std::isfinite(x.positions[i]) || !std::isfinite(x.saved[i]))return false;
    return true;
}
}
template<class T> bool encode(const T& value,std::vector<uint8_t>& out) {
    out.clear();if(!codec::valid(value))return false;
    auto copy=value;codec::Writer w{out};w.value(copy);return out.size()<=kMaxBody;
}
template<class T> bool decode(const uint8_t* data,size_t size,T& out) {
    if(!data || size>kMaxBody)return false;
    T temp;codec::Reader r{data,size};r.value(temp);
    if(!r.ok || r.pos!=size || !codec::valid(temp))return false;
    out=temp;return true;
}
}
