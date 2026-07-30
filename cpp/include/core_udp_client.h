#ifndef CORE_UDP_CLIENT_H
#define CORE_UDP_CLIENT_H

#if defined(_WIN32) || defined(_WIN64)
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    // Note: On Windows, you also need to link against ws2_32.lib in CMake
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
#endif

#include <iostream>
#include <memory>
#include <cstdint>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include "curi_udp/c/curi_udp.h"
#include "platform_state.h"
#include "platform_serialization.h"
#include "time_utils.h"

namespace robot::platform {
    /**
    * @brief Handles passive UDP telemetry and active command-response cycles.
    * 
    * @tparam UnpackT Incoming robot state/telemetry from the telemetry port.
    * @tparam PackT   Outgoing command/request payload sent to the server.
    * @tparam AckT    Synchronous acknowledgment response returned by the server.
    */
    template <typename UnpackT, typename PackT, typename AckT>
    class CoreUdpClient{
        public:
            CoreUdpClient(const std::string& server_ip, int server_port, 
                int telemetry_port, const int receive_ack_port,
                bool (*unpack_func)(const std::uint8_t*, std::size_t, UnpackT&) = nullptr,
                bool (*pack_func)(const PackT&, std::uint8_t*, std::size_t, std::size_t&) = nullptr,
                bool (*ack_unpack_func)(const std::uint8_t*, std::size_t, AckT&) = nullptr,
                float ack_dt_ms = 0.0f);
            ~CoreUdpClient();
            void send(const PackT& data);
            bool receive(UnpackT& data, int timeout_us);
            AckT waitAck(uint16_t client_id, uint32_t sequence_id, float timeout_ms);
            void close();
            CommandResponseStatus getLastStatus() const;

        private:
            void ackThreadTask(int ack_dt_us);
            bool (*pack_function_)(const PackT&, std::uint8_t*, std::size_t, std::size_t&) = nullptr;
            bool (*unpack_function_)(const std::uint8_t*, std::size_t, UnpackT&) = nullptr;
            bool (*ack_unpack_function_)(const std::uint8_t*, std::size_t, AckT&) = nullptr;
            std::size_t receive_buffer_size_;
            std::size_t send_buffer_size_;
            std::size_t ack_buffer_size_;
            std::unique_ptr<udp_node> send_udp_node_ptr_;
            std::unique_ptr<udp_node> telemetry_udp_node_ptr_;
            std::atomic<bool> is_running_ = true;

            //Need for ack thread
            std::mutex ack_mutex_;
            std::condition_variable ack_cv_;
            std::thread ack_thread_;
            AckT ack_data_;
            int ack_dt_us_ = 0;
            CommandResponseStatus last_status_ = CommandResponseStatus::kSuccess;
    };

    namespace core_udp_client_detail {

        template <typename T>
        struct is_smart_pointer : std::false_type {};
        template <typename T>
        struct is_smart_pointer<std::unique_ptr<T>> : std::true_type {};
        template <typename T>
        struct is_smart_pointer<std::shared_ptr<T>> : std::true_type {};

        template<typename AckT>
        bool ackMatches(const AckT& ack, uint16_t client_id, uint32_t sequence_id) {
            if constexpr (std::is_same_v<AckT, robot::platform::CoreResponseVariantPtr>) {
                if (!ack) return false;

                return std::visit([&](const auto& alt) -> bool {
                    using T = std::decay_t<decltype(alt)>;
                    if constexpr (std::is_same_v<T, std::monostate>) {
                        return false;
                    } else if constexpr (std::is_same_v<T, robot::platform::SdkHandshakeRes>) {
                        return alt.request_client_id == client_id;
                    } else {
                        return alt.request_client_id == client_id && alt.request_sequence_id == sequence_id;
                    }
                }, *ack);
            } else {
                return ack.request_client_id == client_id && ack.request_sequence_id == sequence_id;
            }
        }

        template<typename PackT>
        std::size_t packBufferSize() {
            if constexpr (std::is_same_v<PackT, robot::platform::CoreRequestVariantPtr>) {
                using VariantT = robot::platform::CoreRequestVariantPtr::element_type;
                return sizeof(VariantT);
            } else {
                return sizeof(PackT);
            }
        }

        template<typename AckT>
        std::size_t ackBufferSize() {
            if constexpr (std::is_same_v<AckT, robot::platform::CoreResponseVariantPtr>) {
                using VariantT = robot::platform::CoreResponseVariantPtr::element_type;
                return sizeof(VariantT);
            } else {
                return sizeof(AckT);
            }
        }

        template<typename T>
        std::size_t wireBufferSize() {
            if constexpr (std::is_same_v<T, robot::platform::CoreRequestVariantPtr>) {
                return packBufferSize<T>() + HMAC_KEY_SIZE;
            } else if constexpr (std::is_same_v<T, robot::platform::CoreResponseVariantPtr>) {
                return ackBufferSize<T>() + HMAC_KEY_SIZE;
            } else {
                return sizeof(T) + HMAC_KEY_SIZE;
            }
        }

        template<typename AckT>
        bool extractStatus(const AckT& ack, CommandResponseStatus& status) {
            using CleanAckT = std::decay_t<AckT>;
            if constexpr (std::is_same_v<CleanAckT, robot::platform::CoreResponseVariantPtr>) {
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
            } else {
                return false;
            }
        }
    }  // namespace core_udp_client_detail

    template <typename UnpackT, typename PackT, typename AckT>
    CoreUdpClient<UnpackT, PackT, AckT>::CoreUdpClient(const std::string& server_ip, int server_port,
        int telemetry_port, const int receive_ack_port,
        bool (*unpack_func)(const std::uint8_t*, std::size_t, UnpackT&),
        bool (*pack_func)(const PackT&, std::uint8_t*, std::size_t, std::size_t&),
        bool (*ack_unpack_func)(const std::uint8_t*, std::size_t, AckT&),
        float ack_dt_ms){
        receive_buffer_size_ = core_udp_client_detail::wireBufferSize<UnpackT>();
        send_buffer_size_ = core_udp_client_detail::wireBufferSize<PackT>();
        ack_buffer_size_ = core_udp_client_detail::wireBufferSize<AckT>();
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

        if constexpr (!std::is_void_v<AckT>) {
            if (ack_dt_ms > 0 && ack_unpack_func) {
                ack_dt_us_ = static_cast<int>(ack_dt_ms * 1000.0f);
                ack_thread_ = std::thread(&CoreUdpClient<UnpackT, PackT, AckT>::ackThreadTask, this, ack_dt_us_);
                ack_unpack_function_ = ack_unpack_func;
            }
        }
        pack_function_ = pack_func;
        unpack_function_ = unpack_func;
    }

    template <typename UnpackT, typename PackT, typename AckT>
    CoreUdpClient<UnpackT, PackT, AckT>::~CoreUdpClient(){
        close();
    }

    template <typename UnpackT, typename PackT, typename AckT>
    void CoreUdpClient<UnpackT, PackT, AckT>::send(const PackT& data){
        if (!pack_function_) {
            throw std::runtime_error("Pack function is not defined.");
        }
        
        std::size_t written_size;
        pack_function_(data, send_udp_node_ptr_->send_buffer, send_buffer_size_, written_size);
        udp_send(send_udp_node_ptr_.get(), written_size);
    }

    template <typename UnpackT, typename PackT, typename AckT>
    bool CoreUdpClient<UnpackT, PackT, AckT>::receive(UnpackT& data, int timeout_us){
        if (!unpack_function_) {
            throw std::runtime_error("Unpack function is not defined.");
        }

        std::size_t written_buffer_size = udp_select(telemetry_udp_node_ptr_.get(), timeout_us, receive_buffer_size_);
        if (written_buffer_size > 0) {
            bool success = unpack_function_(telemetry_udp_node_ptr_->receive_buffer, written_buffer_size, data);
            return true;
        }
        return false;
    }

    template <typename UnpackT, typename PackT, typename AckT>
    AckT CoreUdpClient<UnpackT, PackT, AckT>::waitAck(uint16_t client_id, uint32_t sequence_id, float timeout_ms){
        static_assert(!std::is_void_v<AckT>, "AckT cannot be void if using Ack features.");
        if (!ack_unpack_function_) {
            throw std::runtime_error("Ack function is not defined.");
        }
    
        std::unique_lock<std::mutex> lock(ack_mutex_);
    
        // wait_for releases the lock and puts this thread to sleep entirely.
        bool cv_status = ack_cv_.wait_for(lock, std::chrono::milliseconds(static_cast<long>(timeout_ms)), [&]() {
            if (ack_data_) {
                if (core_udp_client_detail::ackMatches(ack_data_, client_id, sequence_id)) {
                    return true; 
                }
                ack_data_ = AckT{}; 
            }
            return false;
        });
    
        if (cv_status) {
            AckT matched = ack_data_;
            ack_data_ = AckT{}; // Reset
            return matched;
        }
    
        return AckT{};
    }

    template <typename UnpackT, typename PackT, typename AckT>
    void CoreUdpClient<UnpackT, PackT, AckT>::close(){
        is_running_ = false;
        // Stop ack polling before freeing UDP buffers/sockets it uses.
        if (ack_thread_.joinable()) {
            ack_thread_.join();
        }
        if (telemetry_udp_node_ptr_) udp_close(telemetry_udp_node_ptr_.get());
        if (send_udp_node_ptr_) udp_close(send_udp_node_ptr_.get());
    }

    
    template <typename UnpackT, typename PackT, typename AckT>
    CommandResponseStatus CoreUdpClient<UnpackT, PackT, AckT>::getLastStatus() const{
        return last_status_;
    }

    template <typename UnpackT, typename PackT, typename AckT>
    void CoreUdpClient<UnpackT, PackT, AckT>::ackThreadTask(int ack_dt_us){
        while (is_running_) {
            std::size_t written_buffer_size = udp_select(send_udp_node_ptr_.get(), ack_dt_us, ack_buffer_size_);
            
            if (is_running_ && written_buffer_size > 0) {
                {
                    std::lock_guard<std::mutex> lock(ack_mutex_);
                    ack_data_ = AckT{};
                    ack_unpack_function_(send_udp_node_ptr_->receive_buffer, written_buffer_size, ack_data_);
                    bool success = core_udp_client_detail::extractStatus(ack_data_, last_status_);
                }
                ack_cv_.notify_one(); 
            }
        }
    }
} // namespace robot::platform
#endif