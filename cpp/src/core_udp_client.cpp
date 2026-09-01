#include "core_udp_client.h"

#include <chrono>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include "platform_serialization.h"

namespace robot::platform {
namespace {

bool ackMatches(const CoreResponseVariantPtr& ack, uint16_t client_id, uint32_t sequence_id) {
    if (!ack) return false;

    return std::visit([&](const auto& alt) -> bool {
        using T = std::decay_t<decltype(alt)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            return false;
        } else {
            return alt.request_client_id == client_id && alt.request_sequence_id == sequence_id;
        }
    }, *ack);
}

std::size_t srvStateBufferSize() {
    return sizeof(SrvState) + MAX_PROTOCOL_EXTENSION_SIZE + HMAC_KEY_SIZE;
}

std::size_t requestBufferSize() {
    return sizeof(CoreRequestVariant) + MAX_PROTOCOL_EXTENSION_SIZE + HMAC_KEY_SIZE;
}

std::size_t responseBufferSize() {
    return sizeof(CoreResponseVariant) + MAX_PROTOCOL_EXTENSION_SIZE + HMAC_KEY_SIZE;
}

bool extractStatus(const CoreResponseVariantPtr& ack, CommandResponseStatus& status) {
    if (!ack) return false;

    return std::visit([&](const auto& variant_item) -> bool {
        using T = std::decay_t<decltype(variant_item)>;

        if constexpr (std::is_same_v<T, std::monostate>) {
            return false;
        } else {
            status = variant_item.payload.status;
            return true;
        }
    }, *ack);
}

}  // namespace

CoreUdpClient::CoreUdpClient(const std::string& server_ip, int server_port,
    int telemetry_port, const int receive_ack_port,
    bool (*unpack_func)(const std::uint8_t*, std::size_t, SrvState&),
    bool (*pack_func)(const CoreRequestVariantPtr&, std::uint8_t*, std::size_t, std::size_t&),
    bool (*ack_unpack_func)(const std::uint8_t*, std::size_t, CoreResponseVariantPtr&),
    float ack_dt_ms){
    receive_buffer_size_ = srvStateBufferSize();
    send_buffer_size_ = requestBufferSize();
    ack_buffer_size_ = responseBufferSize();
    telemetry_udp_node_ptr_ = std::make_unique<udp_node>();
    send_udp_node_ptr_ = std::make_unique<udp_node>();

    //Initialize for UDP send and ack
    char ip[] = "0.0.0.0";
    int ret = udp_init_share_fd(send_udp_node_ptr_.get(), ip, receive_ack_port, server_ip.c_str(), server_port, ack_buffer_size_, send_buffer_size_);
    if (ret != 0){
        throw std::runtime_error("Failed to initialize send_udp_node_ptr_, return code: " + std::to_string(ret));
    }

    int ret1 = udp_init(telemetry_udp_node_ptr_.get(), ip, telemetry_port, "", -1, receive_buffer_size_, 0);
    if (ret1 != 0){
        throw std::runtime_error("Failed to initialize telemetry_udp_node_ptr_, return code: " + std::to_string(ret1));
    }

    if (ack_dt_ms > 0 && ack_unpack_func) {
        ack_dt_us_ = static_cast<int>(ack_dt_ms * 1000.0f);
        ack_thread_ = std::thread(&CoreUdpClient::ackThreadTask, this, ack_dt_us_);
        ack_unpack_function_ = ack_unpack_func;
    }
    pack_function_ = pack_func;
    unpack_function_ = unpack_func;
}

CoreUdpClient::~CoreUdpClient(){
    close();
}

void CoreUdpClient::send(const CoreRequestVariantPtr& data){
    if (!pack_function_) {
        throw std::runtime_error("Pack function is not defined.");
    }

    std::size_t written_size;
    pack_function_(data, send_udp_node_ptr_->send_buffer, send_buffer_size_, written_size);
    udp_send(send_udp_node_ptr_.get(), written_size);
}

bool CoreUdpClient::receive(SrvState& data, int timeout_us){
    if (!unpack_function_) {
        throw std::runtime_error("Unpack function is not defined.");
    }

    std::size_t written_buffer_size = udp_select(telemetry_udp_node_ptr_.get(), timeout_us, receive_buffer_size_);
    if (written_buffer_size > 0) {
        unpack_function_(telemetry_udp_node_ptr_->receive_buffer, written_buffer_size, data);
        return true;
    }
    return false;
}

CoreResponseVariantPtr CoreUdpClient::waitAck(uint16_t client_id, uint32_t sequence_id, float timeout_ms){
    if (!ack_unpack_function_) {
        throw std::runtime_error("Ack function is not defined.");
    }

    std::unique_lock<std::mutex> lock(ack_mutex_);

    // wait_for releases the lock and puts this thread to sleep entirely.
    bool cv_status = ack_cv_.wait_for(lock, std::chrono::milliseconds(static_cast<long>(timeout_ms)), [&]() {
        if (ack_data_) {
            if (ackMatches(ack_data_, client_id, sequence_id)) {
                return true;
            }
            ack_data_ = CoreResponseVariantPtr{};
        }
        return false;
    });

    if (cv_status) {
        CoreResponseVariantPtr matched = ack_data_;
        ack_data_ = CoreResponseVariantPtr{}; // Reset
        return matched;
    }

    return CoreResponseVariantPtr{};
}

void CoreUdpClient::close(){
    is_running_ = false;
    ack_cv_.notify_all();
    if (ack_thread_.joinable()) {
        ack_thread_.join();
    }
    if (telemetry_udp_node_ptr_) udp_close(telemetry_udp_node_ptr_.get());
    if (send_udp_node_ptr_) udp_close(send_udp_node_ptr_.get());
}

CommandResponseStatus CoreUdpClient::getLastStatus() const{
    return last_status_;
}

void CoreUdpClient::ackThreadTask(int ack_dt_us){
    while (is_running_) {
        std::size_t written_buffer_size = udp_select(send_udp_node_ptr_.get(), ack_dt_us, ack_buffer_size_);

        if (is_running_ && written_buffer_size > 0) {
            {
                std::lock_guard<std::mutex> lock(ack_mutex_);
                ack_data_ = CoreResponseVariantPtr{};
                ack_unpack_function_(send_udp_node_ptr_->receive_buffer, written_buffer_size, ack_data_);
                extractStatus(ack_data_, last_status_);
            }
            ack_cv_.notify_one();
        }
    }
}

}  // namespace robot::platform
