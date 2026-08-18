#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include "tcp_data_client.h"
#include "tcp_data_server.h"
#include "tcp_data_services.h"

namespace {

constexpr int kPort = 38991;
constexpr const char* kHost = "127.0.0.1";

class RpcStorageHandler final : public robot::platform::TcpDataServer::DataChannelHandler {
public:
    explicit RpcStorageHandler(std::string storage_dir) : storage_dir_(std::move(storage_dir)) {}

    bool onBlobComplete(const robot::platform::TcpDataServer::BlobReceipt& receipt,
                        std::string& error) override {
        const auto path = std::filesystem::path(storage_dir_) / receipt.name;
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out) {
            error = "failed to write blob";
            return false;
        }
        if (!receipt.data.empty()) {
            out.write(reinterpret_cast<const char*>(receipt.data.data()),
                      static_cast<std::streamsize>(receipt.data.size()));
        }
        stop_server_.store(true);
        return static_cast<bool>(out);
    }

    robot::platform::TcpDataServer::RpcResult onRpcRequest(std::uint32_t service_id, std::uint32_t method_id,
                                                            const std::uint8_t* request_body,
                                                            std::size_t request_size) override {
        using namespace robot::platform::tcp_data;
        robot::platform::TcpDataServer::RpcResult result;
        result.ok = true;

        if (service_id != static_cast<std::uint32_t>(ServiceId::kStorage)) {
            result.status_code = 404;
            return result;
        }

        switch (static_cast<StorageMethod>(method_id)) {
            case StorageMethod::kList: {
                std::vector<FileEntry> entries;
                for (const auto& entry : std::filesystem::directory_iterator(storage_dir_)) {
                    if (!entry.is_regular_file()) {
                        continue;
                    }
                    FileEntry file_entry;
                    file_entry.name = entry.path().filename().string();
                    file_entry.size = static_cast<std::uint64_t>(entry.file_size());
                    entries.push_back(std::move(file_entry));
                }
                encodeFileList(entries, result.response_body);
                result.status_code = 0;
                return result;
            }
            case StorageMethod::kDelete: {
                std::string name;
                if (!decodeNameRequest(request_body, request_size, name)) {
                    result.status_code = 400;
                    return result;
                }
                std::filesystem::remove(std::filesystem::path(storage_dir_) / name);
                result.status_code = 0;
                return result;
            }
            case StorageMethod::kStat: {
                std::string name;
                if (!decodeNameRequest(request_body, request_size, name)) {
                    result.status_code = 400;
                    return result;
                }
                const auto path = std::filesystem::path(storage_dir_) / name;
                if (!std::filesystem::exists(path)) {
                    result.status_code = 404;
                    return result;
                }
                StatResult stat;
                stat.size = static_cast<std::uint64_t>(std::filesystem::file_size(path));
                encodeStatResult(stat, result.response_body);
                result.status_code = 0;
                return result;
            }
            case StorageMethod::kDownload: {
                std::string name;
                if (!decodeNameRequest(request_body, request_size, name)) {
                    result.status_code = 400;
                    return result;
                }
                const auto path = std::filesystem::path(storage_dir_) / name;
                if (!std::filesystem::exists(path)) {
                    result.status_code = 404;
                    return result;
                }
                std::ifstream in(path, std::ios::binary);
                result.download_name = name;
                result.download_data.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
                encodeDownloadMeta(result.download_data.size(), result.response_body);
                result.status_code = 0;
                return result;
            }
            default:
                result.status_code = 404;
                return result;
        }
    }

    std::atomic<bool> stop_server_{false};

private:
    std::string storage_dir_;
};

}  // namespace

int main() {
    using robot::platform::TcpDataClient;
    using robot::platform::TcpDataServer;

    const auto storage_dir = std::filesystem::temp_directory_path() / "tcp_data_rpc_test";
    std::filesystem::remove_all(storage_dir);
    std::filesystem::create_directories(storage_dir);

    RpcStorageHandler handler(storage_dir.string());

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
        std::cerr << "connect failed\n";
        handler.stop_server_.store(true);
        server_thread.join();
        return 1;
    }

    const std::vector<std::uint8_t> payload = {'r', 'p', 'c', '-', 't', 'e', 's', 't'};
    if (auto upload = client.uploadBlob("rpc_demo.teach", payload); !upload.ok) {
        std::cerr << "upload failed: " << upload.message << '\n';
        handler.stop_server_.store(true);
        server_thread.join();
        return 1;
    }

    std::vector<robot::platform::tcp_data::FileEntry> entries;
    if (auto listed = client.listFiles("", entries); !listed.ok || entries.size() != 1 ||
                                                    entries[0].name != "rpc_demo.teach") {
        std::cerr << "list failed\n";
        handler.stop_server_.store(true);
        server_thread.join();
        return 1;
    }

    robot::platform::tcp_data::StatResult stat{};
    if (auto stat_result = client.statFile("rpc_demo.teach", stat); !stat_result.ok ||
                             stat.size != payload.size()) {
        std::cerr << "stat failed\n";
        handler.stop_server_.store(true);
        server_thread.join();
        return 1;
    }

    std::vector<std::uint8_t> downloaded;
    if (auto download = client.downloadBlob("rpc_demo.teach", downloaded); !download.ok ||
                          downloaded != payload) {
        std::cerr << "download failed\n";
        handler.stop_server_.store(true);
        server_thread.join();
        return 1;
    }

    if (auto deleted = client.deleteFile("rpc_demo.teach"); !deleted.ok) {
        std::cerr << "delete failed\n";
        handler.stop_server_.store(true);
        server_thread.join();
        return 1;
    }

    handler.stop_server_.store(true);
    client.close();
    server_thread.join();

    std::cout << "tcp data channel rpc ok\n";
    return 0;
}
