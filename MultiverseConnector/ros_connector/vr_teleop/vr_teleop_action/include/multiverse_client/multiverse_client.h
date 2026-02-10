#pragma once
#include <map>
#include <string>
#include <atomic>
#include <cstdlib>
#include "transport/client_transport.hpp"

template<class T>
struct TypedBuffer
{
    T* data = nullptr;
    size_t size = 0;
};

struct Buffer
{
    TypedBuffer<double> buffer_double;
    TypedBuffer<uint8_t> buffer_uint8_t;
    TypedBuffer<uint16_t> buffer_uint16_t;
};

enum class EMultiverseClientState : unsigned char;

class MultiverseClient
{
public:
    virtual ~MultiverseClient() = default;
    void set_transport(ClientTransportType transport_type, IClientTransport* transport = nullptr);

    void connect(const std::string& in_host, const std::string& in_server_port, const std::string& in_client_port);
    void connect();
    void start();
    virtual bool communicate(const bool resend_request_meta_data = false);
    void disconnect();
    virtual double get_time_now() const;

protected:
    void send_and_receive_meta_data();
    void connect_to_server();

    virtual void start_connect_to_server_thread() = 0;
    virtual void wait_for_connect_to_server_thread_finish() = 0;
    virtual void start_meta_data_thread() = 0;
    virtual void wait_for_meta_data_thread_finish() = 0;

    virtual bool init_objects(bool from_request_meta_data = false) = 0;
    virtual void bind_request_meta_data() = 0;
    virtual bool compute_request_and_response_meta_data() = 0;
    virtual void compute_request_buffer_sizes(
        std::map<std::string, size_t>& send_sz, std::map<std::string, size_t>& recv_sz) const = 0;
    virtual void compute_response_buffer_sizes(
        std::map<std::string, size_t>& send_sz, std::map<std::string, size_t>& recv_sz) const = 0;
    virtual void bind_response_meta_data() = 0;
    virtual void bind_api_callbacks() = 0;
    virtual void bind_api_callbacks_response() = 0;
    virtual void init_send_and_receive_data() = 0;
    virtual void bind_send_data() = 0;
    virtual void bind_receive_data() = 0;
    virtual void clean_up() = 0;
    virtual void reset() = 0;

private:
    void run();

    // Transport-agnostic send/recv helpers that mirror your old calls
    void send_request_meta_data();
    void send_send_data();
    void receive_data();
    void check_response_meta_data();
    bool check_buffer_size();
    void init_buffer();

protected:
    std::string host;
    std::string server_port = "7000";
    std::string client_port;

    Buffer send_buffer;
    Buffer receive_buffer;

    std::string request_meta_data_str;
    std::string response_meta_data_str;

    std::atomic<EMultiverseClientState> flag;

    double* world_time = (double*)calloc(1, sizeof(double));

private:
    std::string socket_addr;
    ClientTransportType transport_type_ = ClientTransportType::Zmq;
    IClientTransport* transport_ = nullptr;
    double reset_cool_down = 1.0;
    double reset_time = 0.0;

    // helpers
    void ensure_transport_allocated();
};
