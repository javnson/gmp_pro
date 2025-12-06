
// Headers
#include <exception>
#include <fstream>
#include <iostream>
#include <string>

// json library
#include <nlohmann/json.hpp>

#include <thread>

#include <mutex>

#include <algorithm>

using json = nlohmann::json;

#if !defined __linux__
// Need to define WIN32 because each version of windows has slightly different ways of handling networking
#include <SDKDDKVer.h>
#endif



#include <asio.hpp>
using namespace asio;

// ASIO library
//#if defined __linux__
//#define ASIO_STANDALONE
//#endif // __linux__
//#if defined(ASIO_STANDALONE)
//#include <asio.hpp>
//using namespace asio;
//#else
//#include <boost/asio.hpp>
//using namespace boost::asio;
//#endif

using udp = ip::udp;

#if !defined __linux__
// Other Windows functions
#include <Windows.h>
#endif 

#include <time.h>

//using boost::asio;

class asio_udp_helper
{
  public:
    // error_code ecVAR;
    udp::endpoint recv_terminal;
    udp::endpoint tran_terminal;
    udp::endpoint cmd_recv_terminal;
    udp::endpoint cmd_tran_terminal;
    io_context recv_context, tran_context;
    io_context cmd_recv_context, cmd_tran_context;
    udp::socket recv_socket, tran_socket;
    udp::socket cmd_recv_socket, cmd_tran_socket;
    uint32_t recv_port;
    uint32_t trans_port;
    uint32_t cmd_recv_port;
    uint32_t cmd_trans_port;
    std::string ip_addr;

    std::string cmd_recv_buf;

    uint32_t stop_cmd_received;

    uint32_t start_cmd_received;

    uint32_t recv_counter;
    uint32_t tran_counter;

  public:
    asio_udp_helper(const std::string ip_addr, uint32_t recv_port, uint32_t trans_port, uint32_t cmd_recv_port,
                    uint32_t cmd_trans_port)
        : /*ecVAR(),*/ recv_terminal(ip::make_address(ip_addr),
                                     static_cast<ip::port_type>(recv_port)),
          tran_terminal(ip::make_address(ip_addr), static_cast<ip::port_type>(trans_port)),
          cmd_recv_terminal(ip::make_address(ip_addr),
                            static_cast<ip::port_type>(cmd_recv_port)),
          cmd_tran_terminal(ip::make_address(ip_addr),
                            static_cast<ip::port_type>(cmd_trans_port)),
          recv_context(), tran_context(), cmd_recv_context(), cmd_tran_context(), recv_socket(recv_context),
          tran_socket(tran_context), cmd_recv_socket(cmd_recv_context), cmd_tran_socket(cmd_tran_context),
          recv_port(recv_port), trans_port(trans_port), cmd_recv_port(cmd_recv_port), cmd_trans_port(cmd_trans_port),
          ip_addr(ip_addr), cmd_recv_buf(1024, '\0'), stop_cmd_received(0), start_cmd_received(0), recv_counter(0),
          tran_counter(0)
    {
    }

    // Create link
    void connect_to_target()
    {
        tran_socket.connect(tran_terminal);

        recv_socket.open(ip::udp::v4());
        recv_socket.bind(recv_terminal);

        cmd_tran_socket.connect(cmd_tran_terminal);

        cmd_recv_socket.open(ip::udp::v4());
        cmd_recv_socket.bind(cmd_recv_terminal);

        stop_cmd_received = 0;
        start_cmd_received = 0;

        recv_counter = 0;
        tran_counter = 0;
    }

    void release_connect()
    {
        tran_socket.cancel();
        tran_socket.close();
        recv_socket.cancel();
        recv_socket.close();
        cmd_tran_socket.cancel();
        cmd_tran_socket.close();
        cmd_recv_context.stop();
        cmd_recv_socket.cancel();
        cmd_recv_socket.close();

        // stop_cmd_received = 0;
        // start_cmd_received = 0;
    }

    void send_msg(const char *msg, uint32_t len)
    {
        try
        {

            tran_socket.send(buffer(msg, len));

            tran_counter += len;
        }
        catch (std::exception &e)
        {
            std::cout << "Exception: " << e.what() << std::endl;
            return;
        }
    }

    void send_cmd(const char *msg, uint32_t len)
    {
        tran_counter += len;

        cmd_tran_socket.send(buffer(msg, len));
    }

    int recv_msg(char *msg, uint32_t len)
    {
        try
        {
#ifndef DISABLE_ASIO_HELPER_TIMEOUT_OPTION
            // if use this macro will wait permanently.

#ifdef ASIO_PLAIN_TIMEOUT
            // Fixed timeout mode
            // somewhere in your headers to be used everywhere you need it
            //typedef boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO> rcv_timeout_option;
            //recv_socket.set_option(rcv_timeout_option{2000});

            typedef detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO> rcv_timeout_option;
            recv_socket.set_option(rcv_timeout_option{GMP_ASIO_UDP_LINK_TIMEOUT});

#else

#ifdef PC_SIMULATE_STOP_CONDITION
            // Server mode

            // if receive has setup, start timeout
            if (this->recv_counter > 100)
            {
                // somewhere in your headers to be used everywhere you need it
                typedef detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO> rcv_timeout_option;
                recv_socket.set_option(rcv_timeout_option{GMP_ASIO_UDP_LINK_TIMEOUT}); // 2000 s \approx 33 min
            }
#else
            // Default Mode: client mode

            // if receive has setup, stop timeout
            if (this->recv_counter > 100)
            {
                // somewhere in your headers to be used everywhere you need it
                typedef detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO> rcv_timeout_option;
                recv_socket.set_option(rcv_timeout_option{2000000}); // 2000 s \approx 33 min
            }
            else
            {
                // somewhere in your headers to be used everywhere you need it
                typedef detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO> rcv_timeout_option;
                recv_socket.set_option(rcv_timeout_option{GMP_ASIO_UDP_LINK_TIMEOUT});
            }

#endif // PC_SIMULATE_STOP_CONDITION

#endif // ASIO_PLAIN_TIMEOUT

            // Disable timeout mode

#endif // DISABLE_ASIO_HELPER_TIMEOUT_OPTION

            // recv_socket.receive(boost::asio::buffer((char *)&data_t, sizeof(double)));
            recv_socket.receive_from(buffer(msg, len), recv_terminal);

            recv_counter += len;
        }
        catch (std::exception &e)
        {
            std::cout << "Exception: " << e.what() << std::endl;
            return 1;
        }
        return 0;
    }

    void server_ack_cmd()
    {
        // recv_socket.receive(boost::asio::buffer((char *)&data_t, sizeof(double)));
        // cmd_recv_socket.receive_from(boost::asio::buffer(msg, len), cmd_recv_terminal);

        // Bug fix this function may trigger exception

        cmd_recv_socket.async_receive_from(
            asio::buffer(cmd_recv_buf), cmd_recv_terminal, [this](std::error_code ec, std::size_t bytes_recvd) {
                // std::cout << "this function is reached, byte received:" << bytes_recvd << ".\r\n";
                // std::cout << "content:" << cmd_recv_buf << std::endl;
                // std::cout << "error code:" << ec.message() << std::endl;

                if ((!ec.value()) && (bytes_recvd > 0))
                {
                    // judge if this is a stop Command
                    if (!strcmp(cmd_recv_buf.c_str(), "Stop"))
                    {
                        stop_cmd_received = 1;

                        // Stop the whole process
                        std::cout << "[ASIO-UDP Helper] Simulation Stop Command is received, and connection would be "
                                     "released.\r\n";
#if defined __linux__
                        sleep(1);
#else // Windows platform
                        Sleep(1);
#endif // platform selection
                        this->release_connect();
                    }
                    // judge if this is a Start Command
                    else if (!strcmp(cmd_recv_buf.c_str(), "Start"))
                    {
                        // Start the whole process
                        std::cout << "[ASIO-UDP Helper] Simulation Start Command is received.\r\n";

                        start_cmd_received = 1;
                    }

                    std::fill(cmd_recv_buf.begin(), cmd_recv_buf.end(), static_cast<std::string::value_type>(0));
                }

                this->server_ack_cmd();
            });

        std::thread recv_content([this]() { this->cmd_recv_context.run_one(); });

        recv_content.detach();
    }

    void register_start_callback()
    {
        // Receive "start" string to judge simulation is start
    }

    void regiser_stop_callback()
    {
        // Receive "stop" string to judge simulation is stop
    }

    void set_overtime()
    {
        // recv_socket.set_option(asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO>{15});

        // Not work
        struct timeval tv = {1, 0};
        setsockopt(recv_socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv));
    }

    static asio_udp_helper *parse_network_config(std::string config_file)
    {
        // file objects
        std::fstream network_json(config_file, std::fstream::in | std::fstream::out | std::fstream::app);

        if (!network_json.is_open())
        {
            std::cout << "[WARN] Cannot find network config file." << std::endl;

            return nullptr;
        }

        // config json
        json config;

        try
        {
            config = json::parse(network_json);
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERRO] Cannot parse .json file, " << config_file << std::endl;
            std::cerr << e.what() << std::endl;

            return nullptr;
        }

        // get result
        try
        {
            std::string target_addr = config["target_address"];
            uint32_t recv_port = config["receive_port"];
            uint32_t trans_port = config["transmit_port"];
            uint32_t cmd_recv_port = config["command_recv_port"];
            uint32_t cmd_trans_port = config["command_trans_port"];

            // create objects
            return new asio_udp_helper(target_addr, recv_port, trans_port, cmd_recv_port, cmd_trans_port);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;

            return nullptr;
        }
    }

    // Data receive and transmit, exchange two ports
    static asio_udp_helper *parse_network_config_server(std::string config_file)
    {
        // file objects
        std::fstream network_json(config_file, std::fstream::in | std::fstream::out | std::fstream::app);

        if (!network_json.is_open())
        {
            std::cout << "[WARN] Cannot find network config file." << std::endl;

            return nullptr;
        }

        // config json
        json config;

        try
        {
            config = json::parse(network_json);
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERRO] Cannot parse .json file, " << config_file << std::endl;
            std::cerr << e.what() << std::endl;

            return nullptr;
        }

        // get result
        try
        {
            std::string target_addr = config["target_address"];
            uint32_t recv_port = config["receive_port"];
            uint32_t trans_port = config["transmit_port"];
            uint32_t cmd_recv_port = config["command_recv_port"];
            uint32_t cmd_trans_port = config["command_trans_port"];

            // create objects
            // The function is different from parse_network_config
            return new asio_udp_helper(target_addr, trans_port, recv_port, cmd_trans_port, cmd_recv_port);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;

            return nullptr;
        }
    }
};