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

#include <memory>
#include <cstdint>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <string>

#include "curi_udp/c/curi_udp.h"
#include "platform_state.h"

namespace robot::platform {
    /**
    * @brief Handles passive UDP telemetry and active command-response cycles.
    *
    * Fixed wire types: SrvState telemetry, CoreRequestVariantPtr commands,
    * CoreResponseVariantPtr acknowledgments.
    */
    class CoreUdpClient{
        public:
            CoreUdpClient(const std::string& server_ip, int server_port,
                int telemetry_port, const int receive_ack_port,
                bool (*unpack_func)(const std::uint8_t*, std::size_t, SrvState&) = nullptr,
                bool (*pack_func)(const CoreRequestVariantPtr&, std::uint8_t*, std::size_t, std::size_t&) = nullptr,
                bool (*ack_unpack_func)(const std::uint8_t*, std::size_t, CoreResponseVariantPtr&) = nullptr,
                float ack_dt_ms = 0.0f);
            ~CoreUdpClient();
            void send(const CoreRequestVariantPtr& data);
            bool receive(SrvState& data, int timeout_us);
            CoreResponseVariantPtr waitAck(uint16_t client_id, uint32_t sequence_id, float timeout_ms);
            void close();
            CommandResponseStatus getLastStatus() const;

        private:
            void ackThreadTask(int ack_dt_us);
            bool (*pack_function_)(const CoreRequestVariantPtr&, std::uint8_t*, std::size_t, std::size_t&) = nullptr;
            bool (*unpack_function_)(const std::uint8_t*, std::size_t, SrvState&) = nullptr;
            bool (*ack_unpack_function_)(const std::uint8_t*, std::size_t, CoreResponseVariantPtr&) = nullptr;
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
            CoreResponseVariantPtr ack_data_;
            int ack_dt_us_ = 0;
            CommandResponseStatus last_status_ = CommandResponseStatus::kSuccess;
    };
} // namespace robot::platform
#endif
