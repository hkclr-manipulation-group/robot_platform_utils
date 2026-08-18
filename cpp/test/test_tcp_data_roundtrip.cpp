#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

#include "tcp_data_client.h"
#include "tcp_data_server.h"

namespace {

constexpr int kPort = 38990;
constexpr const char* kHost = "127.0.0.1";

class UploadHandler final : public robot::platform::TcpDataServer::DataChannelHandler {
public:
    explicit UploadHandler(std::vector<std::uint8_t>& received) : received_(received) {}

    bool onBlobComplete(const robot::platform::TcpDataServer::BlobReceipt& receipt,
                        std::string& error) override {
        if (receipt.name != "demo.teach") {
            error = "unexpected blob name";
            return false;
        }
        received_ = receipt.data;
        stop_server_.store(true);
        return true;
    }

    robot::platform::TcpDataServer::RpcResult onRpcRequest(std::uint32_t, std::uint32_t, const std::uint8_t*,
                                                           std::size_t) override {
        robot::platform::TcpDataServer::RpcResult result;
        result.ok = true;
        result.status_code = 404;
        return result;
    }

    std::atomic<bool> stop_server_{false};
    std::vector<std::uint8_t>& received_;
};

}  // namespace

int main() {
    using robot::platform::TcpDataClient;
    using robot::platform::TcpDataServer;

    std::vector<std::uint8_t> received;
    UploadHandler handler(received);

    std::thread server_thread([&]() {
        TcpDataServer server;
        if (!server.listen(kHost, kPort)) {
            std::cerr << "TcpDataServer listen failed\n";
            std::exit(1);
        }

        server.serveForever(handler, [&]() { return handler.stop_server_.load(); });
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    TcpDataClient client;
    if (!client.connect(kHost, kPort)) {
        std::cerr << "TcpDataClient connect failed\n";
        handler.stop_server_.store(true);
        server_thread.join();
        return 1;
    }

    auto ping = client.ping();
    if (!ping.ok) {
        std::cerr << "ping failed: " << ping.message << '\n';
        handler.stop_server_.store(true);
        server_thread.join();
        return 1;
    }
    std::cout << "ping ok\n";

    const std::vector<std::uint8_t> payload = {
        'S', 'P', '2', ' ', 'c', 'o', 'l', 'd', ' ', 'p', 'a', 't', 'h'};
    auto upload = client.uploadBlob("demo.teach", payload);
    if (!upload.ok) {
        std::cerr << "upload failed: " << upload.message << '\n';
        handler.stop_server_.store(true);
        server_thread.join();
        return 1;
    }
    std::cout << "upload ok\n";

    client.close();
    server_thread.join();

    if (received != payload) {
        std::cerr << "payload mismatch\n";
        return 1;
    }

    std::cout << "tcp data channel roundtrip ok\n";
    return 0;
}
