#pragma once
#include <string>
#include <vector>

enum class ClientTransportType : unsigned char
{
    Tcp,
    Udp,
    Zmq
};

class IClientTransport
{
public:
    virtual ~IClientTransport() = default;

    virtual ClientTransportType type() const = 0;

    virtual void connect(const std::string& endpoint) = 0;
    virtual void disconnect() = 0;

    virtual void send(const void* data, size_t len, bool more) = 0;
    virtual void send_text(const std::string& s, bool more) = 0;

    virtual void recv(void* data, size_t len) = 0;
    virtual std::string recv_text() = 0;
    virtual bool recv_multipart(std::vector<std::string>& parts) = 0;
};
