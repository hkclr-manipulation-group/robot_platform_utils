#ifndef CUARM_UDP_H
#define CUARM_UDP_H

#include <iostream>
#include <memory>

extern "C" {
    #include "curi_udp/c/curi_udp.h"
}

namespace robot::platform {
    template <typename UnpackT, typename PackT>
    class CuarmUdp{
        public:
            CuarmUdp(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port,
                bool (*unpack_func)(const std::uint8_t*, std::size_t, UnpackT&) = nullptr,
                bool (*pack_func)(const PackT&, std::uint8_t*, std::size_t, std::size_t&) = nullptr);
            ~CuarmUdp();
            void send(const PackT& data);
            bool receive(UnpackT& data, int timeout_us);
            void close();

        private:
            bool (*pack_function_)(const PackT&, std::uint8_t*, std::size_t, std::size_t&) = nullptr;
            bool (*unpack_function_)(const std::uint8_t*, std::size_t, UnpackT&) = nullptr;
            std::unique_ptr<udp_node> udp_node_ptr_;
            std::size_t receive_buffer_size_;
            std::size_t send_buffer_size_;
    };

    namespace detail {
        // Primary template: Handles standard types and raw pointers
        template <typename T, typename = void>
        struct underlying_size {
            // std::remove_pointer_t strips raw pointers (e.g., Int* -> Int)
            using base_type = std::remove_pointer_t<T>;
            static constexpr size_t value = sizeof(base_type);
        };

        // SFINAE Specialization: Detects smart pointers by looking for a nested ::element_type
        template <typename T>
        struct underlying_size<T, std::void_t<typename T::element_type>> {
            // Both std::unique_ptr and std::shared_ptr define ::element_type
            using base_type = typename T::element_type;
            static constexpr size_t value = sizeof(base_type);
        };

        // Convenient helper variable template (C++14 and newer)
        template <typename T>
        inline constexpr size_t underlying_size_v = underlying_size<T>::value;
    }

    template <typename UnpackT, typename PackT>
    CuarmUdp<UnpackT, PackT>::CuarmUdp(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port, 
        bool (*unpack_func)(const std::uint8_t*, std::size_t, UnpackT&),
        bool (*pack_func)(const PackT&, std::uint8_t*, std::size_t, std::size_t&)){
        receive_buffer_size_ = detail::underlying_size_v<UnpackT> > 1 ? detail::underlying_size_v<UnpackT> : 0;
        send_buffer_size_ = detail::underlying_size_v<PackT> > 1 ? detail::underlying_size_v<PackT> : 0;
        udp_node_ptr_ = std::make_unique<udp_node>();
        
        //Initialize UDP node and Sockets
        int ret = udp_init(udp_node_ptr_.get(), local_ip.c_str(), local_port, remote_ip.c_str(), remote_port, receive_buffer_size_, send_buffer_size_);
        if (ret != 0){
            throw std::runtime_error("Failed to initialize UDP node, return code: " + std::to_string(ret));
        }

        pack_function_ = pack_func;
        unpack_function_ = unpack_func;
    }

    template <typename UnpackT, typename PackT>
    CuarmUdp<UnpackT, PackT>::~CuarmUdp(){
        close();
    }

    template <typename UnpackT, typename PackT>
    void CuarmUdp<UnpackT, PackT>::send(const PackT& data){
        if (!pack_function_) {
            throw std::runtime_error("Pack function is not defined.");
        }
        
        std::size_t written_size;
        bool success = pack_function_(data, udp_node_ptr_->send_buffer, send_buffer_size_, written_size);
        if (!success){
            throw std::runtime_error("CuarmUdp::send: Failed to pack data.");
        }
        udp_send(udp_node_ptr_.get(), written_size);
    }

    template <typename UnpackT, typename PackT>
    bool CuarmUdp<UnpackT, PackT>::receive(UnpackT& data, int timeout_us){
        if (!unpack_function_) {
            throw std::runtime_error("Unpack function is not defined.");
        }

        std::size_t written_buffer_size = udp_select(udp_node_ptr_.get(), timeout_us, receive_buffer_size_);
        if (written_buffer_size > 0) {
            unpack_function_(udp_node_ptr_->receive_buffer, written_buffer_size, data);
            return true;
        }
        return false;
    }

    template <typename UnpackT, typename PackT>
    void CuarmUdp<UnpackT, PackT>::close(){
        udp_close(udp_node_ptr_.get());
    }
}
#endif //CUARM_UDP_H