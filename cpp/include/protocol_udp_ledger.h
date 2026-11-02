#pragma once
#include "protocol_udp.h"
#include <array>
#include <atomic>
#include <mutex>
#include <optional>

namespace robot::platform::protocol_udp {
class Ledger {
public:
    static constexpr std::size_t kCapacity = 64, kPending = 8;
    struct Record { Packet request; Status status; bool pending = false; std::uint64_t age = 0; unsigned settled = 0; };
    // A concurrent arrival may wait until the next control tick; idle ticks must
    // not wait for the network mutex or construct protocol descriptions.
    bool hasWork() const noexcept { return work_.load(std::memory_order_acquire); }
    void expirePending(std::uint64_t now) {
        if (!hasWork()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        expirePendingLocked(now);
        refreshWorkLocked();
    }
    Status submit(const Packet& packet, std::uint64_t now) {
        std::lock_guard<std::mutex> lock(mutex_);
        expirePendingLocked(now);
        // Authentication is a caller precondition: a fresh session for a client
        // invalidates that client's previous session, without unbounded growth.
        for (auto& s : sessions_) if (s.client == packet.context.client && s.session != packet.context.session) {
            for (auto& r : records_) if (r.age && r.request.context.client == s.client && r.request.context.session == s.session && !terminal(r.status.phase)) {
                r.pending = false; r.status = {Phase::Superseded, Reason::Ownership};
            }
            s = {};
        }
        refreshWorkLocked();
        for (const auto& r : records_) if (r.age && sameOperation(r.request.context, packet.context))
            return sameIntent(r.request, packet) ? r.status : Status{Phase::Failed, Reason::Conflict};
        unsigned pending = 0;
        for (const auto& r : records_) {
            if (r.pending) ++pending;
            if (r.age && !terminal(r.status.phase) && r.request.context.client == packet.context.client &&
                r.request.context.session == packet.context.session) return {Phase::Failed, Reason::Capacity};
        }
        if (pending >= kPending) return {Phase::Failed, Reason::Capacity};
        Session* session = nullptr;
        for (auto& s : sessions_) if (s.client == packet.context.client && s.session == packet.context.session) { session = &s; break; }
        if (!session) for (auto& s : sessions_) if (!s.client) { session = &s; break; }
        if (!session) return {Phase::Failed, Reason::Capacity};
        if (session->client && packet.context.command <= session->high_water) return {Phase::Unknown, Reason::NotFound};
        Record* slot = nullptr;
        for (auto& r : records_) if (!r.age) { slot = &r; break; }
        if (!slot) for (auto& r : records_) if (terminal(r.status.phase) && (!slot || r.age < slot->age)) slot = &r;
        if (!slot) return {Phase::Failed, Reason::Capacity};
        *session = {packet.context.client, packet.context.session, packet.context.command};
        *slot = {}; slot->request = packet; slot->age = ++age_;
        slot->status = expired(packet.context, now) ? Status{Phase::Expired, Reason::Expired} : Status{};
        slot->pending = !terminal(slot->status.phase);
        refreshWorkLocked();
        return slot->status;
    }
    Status query(const Context& key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& r : records_) if (r.age && sameOperation(r.request.context, key)) return r.status;
        return {Phase::Unknown, Reason::NotFound};
    }
    std::optional<Record> take() {
        if (!hasWork()) return {};
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& r : records_) if (r.pending) { r.pending = false; return r; }
        return {};
    }
    void update(const Context& key, Status status) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& r : records_) if (r.age && sameOperation(r.request.context, key) && !terminal(r.status.phase)) {
            r.status = status; refreshWorkLocked(); return;
        }
    }
    std::vector<Record> active() const {
        if (!hasWork()) return {};
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Record> out;
        for (const auto& r : records_) if (r.age && !r.pending && !terminal(r.status.phase) && r.status.phase != Phase::Received) out.push_back(r);
        return out;
    }
    void observe(const Context& key, bool running, bool settled, std::uint64_t diagnostics = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& r : records_) if (r.age && sameOperation(r.request.context, key) && !terminal(r.status.phase)) {
            r.status.diagnostics = diagnostics;
            if (running) r.status.phase = Phase::Running;
            r.settled = settled ? r.settled + 1 : 0;
            if (r.settled >= 3) { r.status.phase = Phase::Succeeded; refreshWorkLocked(); }
            return;
        }
    }
private:
    void expirePendingLocked(std::uint64_t now) {
        for (auto& r : records_) if (r.pending && expired(r.request.context, now)) {
            r.pending = false;
            r.status = {Phase::Expired, Reason::Expired};
        }
    }
    void refreshWorkLocked() {
        bool work = false;
        for (const auto& r : records_) work |= r.age && !terminal(r.status.phase);
        work_.store(work, std::memory_order_release);
    }
    struct Session { std::uint16_t client = 0; std::uint32_t session = 0; std::uint64_t high_water = 0; };
    mutable std::mutex mutex_;
    std::array<Record, kCapacity> records_{};
    std::array<Session, 32> sessions_{};
    std::uint64_t age_ = 0;
    std::atomic<bool> work_{false};
};
} // namespace robot::platform::protocol_udp
