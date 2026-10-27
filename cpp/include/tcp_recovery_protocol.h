#pragma once

#include "platform_state.h"
#include "tcp_data_service_ids.h"
#include <array>
#include <cstdint>
#include <type_traits>
#include <vector>

namespace robot::platform::tcp_data::recovery {
constexpr uint32_t kVersion=1;
static_assert(MAX_ARM_SIZE==2 && MAX_ARM_JOINT_SIZE==7 && MAX_GRIPPER_SIZE==2 && MAX_GRIPPER_JOINT_SIZE==6,
              "recovery v1 fixed slot counts changed; introduce a new schema version");
constexpr std::size_t kMaxBody=16384;
constexpr uint32_t kRequestCapacity=64, kOperationCapacity=16, kClientCapacity=64;
struct BootId {
    uint64_t high=0,low=0;
    bool valid() const { return high || low; }
    bool operator==(const BootId& b) const { return high==b.high && low==b.low; }
    bool operator!=(const BootId& b) const { return !(*this==b); }
};
struct Capabilities {
    uint32_t version=kVersion; BootId boot{};
    uint32_t request_capacity=kRequestCapacity,operation_capacity=kOperationCapacity,client_capacity=kClientCapacity;
    // bit0: fault binding, bit1: request deduplication, bit2: result query.
    // Cancellation and continuation of old motion are not supported.
    uint32_t features=7;
};
struct Request {
    uint32_t version=kVersion; BootId boot{};
    uint64_t fault_id=0,request_id=0,stop_generation=0;
    uint32_t options=0; // reserved, MUST be zero
};
struct FaultQuery { uint32_t version=kVersion; BootId boot{}; uint64_t fault_id=0; };
struct OperationQuery {
    uint32_t version=kVersion; BootId boot{};
    uint64_t operation_id=0,request_id=0; // mutually exclusive; both zero = latest
};
enum class Outcome : uint8_t { Accepted=0, Running=1, Succeeded=2, Failed=3 };
// Phase and reason values deliberately independent from RT enum layout.
enum class Phase : uint8_t { FaultLatched=0, Precheck=1, Hardware=2, AwaitOperator=3, VerifyPosition=4, VerifyHold=5, Ready=6, Failed=7 };
enum class Reason : uint8_t { None=0, Configuration=1, Hardware=2, Feedback=3, Position=4, Safety=5, Planner=6, OwnerLost=7, NewFault=8, Baseline=9, Operator=10, Hold=11, Superseded=12, AdmissionRejected=13 };
struct ArmResult {
    uint8_t hardware=0; // 0 NotRequested, 1 Pending, 2 Running, 3 Verified, 4 Unverified
    std::array<uint8_t,MAX_ARM_JOINT_SIZE> joints{};
    uint8_t hold_verified=0,rollback=0; // rollback: 0 not attempted, 1 success, 2 failure
};
struct Operation {
    uint32_t version=kVersion; BootId boot{};
    uint64_t operation_id=0,fault_id=0,attempt=0,stop_generation=0;
    uint16_t initiator_client=0,operator_client=0;
    uint32_t initiator_session=0,operator_session=0;
    uint64_t created_us=0,updated_us=0;
    Outcome outcome=Outcome::Accepted; Phase phase=Phase::Precheck; Reason reason=Reason::None;
    uint8_t legacy=0,arm_count=0;
    std::array<ArmResult,MAX_ARM_SIZE> arms{};
};
struct Diagnostic {
    uint64_t system=0;
    std::array<std::array<uint64_t,MAX_ARM_JOINT_SIZE>,MAX_ARM_SIZE> arm{};
    std::array<std::array<uint64_t,MAX_GRIPPER_JOINT_SIZE>,MAX_GRIPPER_SIZE> gripper{};
};
struct DriverSample {
    uint8_t kind=0; // 0 unknown, 1 TL, 2 RMD
    uint32_t code=0,status=0;
    uint64_t sampled_us=0,sequence=0; // zero sequence means not sampled
    uint8_t fault_present=0;
};
struct DriverRecord { DriverSample first_fault{},latest_fault{},latest_sample{}; };
struct FaultStatus {
    uint32_t version=kVersion; BootId boot{};
    uint64_t fault_id=0,stop_generation=0,entered_us=0,cleared_us=0,diagnostic_sample_us=0,first_diagnostic_sample_us=0;
    uint8_t active=0,trigger=0,reason_truncated=0;
    uint32_t observed_triggers=0,categories=0,requirements=0;
    std::array<char,256> first_reason{},latest_reason{};
    Diagnostic first{},current{},latched{};
    std::array<std::array<DriverRecord,MAX_ARM_JOINT_SIZE>,MAX_ARM_SIZE> driver{};
};
// Explicit field traversal is the version-1 wire schema. All integers are LE;
// no raw structs, padding, native bools or platform byte order enter the wire.
template<class V> void fields(V& v,BootId& x){v(x.high,x.low);}
template<class V> void fields(V& v,Capabilities& x){v(x.version,x.boot,x.request_capacity,x.operation_capacity,x.client_capacity,x.features);}
template<class V> void fields(V& v,Request& x){v(x.version,x.boot,x.fault_id,x.request_id,x.stop_generation,x.options);}
template<class V> void fields(V& v,FaultQuery& x){v(x.version,x.boot,x.fault_id);}
template<class V> void fields(V& v,OperationQuery& x){v(x.version,x.boot,x.operation_id,x.request_id);}
template<class V> void fields(V& v,ArmResult& x){v(x.hardware,x.joints,x.hold_verified,x.rollback);}
template<class V> void fields(V& v,Operation& x){v(x.version,x.boot,x.operation_id,x.fault_id,x.attempt,x.stop_generation,x.initiator_client,x.operator_client,x.initiator_session,x.operator_session,x.created_us,x.updated_us,x.outcome,x.phase,x.reason,x.legacy,x.arm_count,x.arms);}
template<class V> void fields(V& v,Diagnostic& x){v(x.system,x.arm,x.gripper);}
template<class V> void fields(V& v,DriverSample& x){v(x.kind,x.code,x.status,x.sampled_us,x.sequence,x.fault_present);}
template<class V> void fields(V& v,DriverRecord& x){v(x.first_fault,x.latest_fault,x.latest_sample);}
template<class V> void fields(V& v,FaultStatus& x){v(x.version,x.boot,x.fault_id,x.stop_generation,x.entered_us,x.cleared_us,x.diagnostic_sample_us,x.first_diagnostic_sample_us,x.active,x.trigger,x.reason_truncated,x.observed_triggers,x.categories,x.requirements,x.first_reason,x.latest_reason,x.first,x.current,x.latched,x.driver);}
namespace detail {
struct Writer {
    std::vector<uint8_t>& bytes;
    template<class... T> void operator()(T&... x){(value(x),...);}
    template<class T,std::size_t N> void value(std::array<T,N>& a){for(auto& x:a)value(x);}
    template<class T> void value(T& x) {
        if constexpr(std::is_enum_v<T>) { auto n=static_cast<std::underlying_type_t<T>>(x);value(n); }
        else if constexpr(std::is_integral_v<T>) {
            auto n=static_cast<std::make_unsigned_t<T>>(x);
            for(std::size_t i=0;i<sizeof(T);++i)bytes.push_back(static_cast<uint8_t>(n>>(i*8)));
        } else fields(*this,x);
    }
};
struct Reader {
    const uint8_t* data;std::size_t size,pos=0;bool ok=true;
    template<class... T> void operator()(T&... x){(value(x),...);}
    template<class T,std::size_t N> void value(std::array<T,N>& a){for(auto& x:a)value(x);}
    template<class T> void value(T& x) {
        if constexpr(std::is_enum_v<T>) { std::underlying_type_t<T> n{};value(n);x=static_cast<T>(n); }
        else if constexpr(std::is_integral_v<T>) {
            if(!ok || size-pos<sizeof(T)){ok=false;return;}
            uint64_t n=0;for(std::size_t i=0;i<sizeof(T);++i)n|=uint64_t(data[pos++])<<(i*8);
            x=static_cast<T>(n);
        } else fields(*this,x);
    }
};
inline bool valid(const Capabilities& x){return x.boot.valid() && (x.features&7)==7 && x.request_capacity && x.operation_capacity && x.client_capacity;}
inline bool valid(const Request& x){return x.boot.valid() && x.fault_id && x.request_id && !x.options;}
inline bool valid(const FaultQuery& x){return x.boot.valid();}
inline bool valid(const OperationQuery& x){return x.boot.valid() && !(x.operation_id && x.request_id);}
inline bool valid(const Operation& x){
    if(!x.boot.valid() || !x.operation_id || !x.fault_id || x.legacy>1 || x.arm_count>MAX_ARM_SIZE ||
       static_cast<unsigned>(x.outcome)>3 || static_cast<unsigned>(x.phase)>7 || static_cast<unsigned>(x.reason)>13)return false;
    for(const auto& a:x.arms){if(a.hardware>4 || a.hold_verified>1 || a.rollback>2)return false;for(auto j:a.joints)if(j>4)return false;}
    return true;
}
inline bool valid(const FaultStatus& x){
    if(!x.boot.valid() || x.active>1 || x.trigger>5 || x.reason_truncated>1 || x.first_reason.back() || x.latest_reason.back())return false;
    for(const auto& a:x.driver)for(const auto& d:a)for(const auto* s:{&d.first_fault,&d.latest_fault,&d.latest_sample})
        if(s->kind>2 || s->fault_present>1)return false;
    return true;
}
}
template<class T> bool encode(const T& source,std::vector<uint8_t>& out){
    out.clear();if(source.version!=kVersion || !detail::valid(source))return false;
    auto copy=source;detail::Writer w{out};w.value(copy);return out.size()<=kMaxBody;
}
template<class T> bool decode(const uint8_t* data,std::size_t size,T& out){
    if(!data || size>kMaxBody)return false;
    T temporary{};detail::Reader r{data,size};r.value(temporary);
    if(!r.ok || r.pos!=size || temporary.version!=kVersion || !detail::valid(temporary))return false;
    out=temporary;return true;
}
} // namespace robot::platform::tcp_data::recovery
