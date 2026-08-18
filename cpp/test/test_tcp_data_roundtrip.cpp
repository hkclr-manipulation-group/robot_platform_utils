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

}  // namespace

int main()
{
    using robot::platform::TcpDataClient;
    using robot::platform::TcpDataServer;

    std::atomic<bool> stop_server{false};
    std::vector<std::uint8_t> received;

    std::thread server_thread([&]() {
        TcpDataServer server;
        if (!server.listen(kHost, kPort)) {
            std::cerr << "TcpDataServer listen failed\n";
            std::exit(1);
        }

        server.serveForever(
            [&](const TcpDataServer::BlobReceipt& receipt, std::string& error) -> bool {
                if (receipt.name != "demo.teach") {
                    error = "unexpected blob name";
                    return false;
                }
                received = receipt.data;
                stop_server.store(true);
                return true;
            },
            [&]() { return stop_server.load(); });
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    TcpDataClient client;
    if (!client.connect(kHost, kPort)) {
        std::cerr << "TcpDataClient connect failed\n";
        stop_server.store(true);
        server_thread.join();
        return 1;
    }

    auto ping = client.ping();
    if (!ping.ok) {
        std::cerr << "ping failed: " << ping.message << '\n';
        stop_server.store(true);
        server_thread.join();
        return 1;
    }
    std::cout << "ping ok\n";

    const std::vector<std::uint8_t> payload = {
        'S', 'P', '2', ' ', 'c', 'o', 'l', 'd', ' ', 'p', 'a', 't', 'h'};
    auto upload = client.uploadBlob("demo.teach", payload);
    if (!upload.ok) {
        std::cerr << "upload failed: " << upload.message << '\n';
        stop_server.store(true);
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

    std::cout << "tcp cold path roundtrip ok\n";
    return 0;
}
