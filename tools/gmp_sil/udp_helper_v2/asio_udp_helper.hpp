/**
 * @file asio_udp_helper.hpp
 * @brief 改进后的 ASIO UDP 通信辅助类，支持跨设备通信与多网卡监听
 * @version 0.2
 * @date 2026-01-29
 */

#pragma once

#include <algorithm>
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>

#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

// ASIO 包含
#if !defined __linux__
#include <SDKDDKVer.h>
#endif

#define ASIO_STANDALONE
#include <asio.hpp>

#if !defined __linux__
#include <Windows.h>
#endif

using json = nlohmann::json;
using namespace asio;
using udp = ip::udp;

class asio_udp_helper
{
  public:
    // 远程端点（发送目标）
    udp::endpoint tran_terminal;
    udp::endpoint cmd_tran_terminal;

    // 本地端点（监听接口）
    udp::endpoint recv_terminal;
    udp::endpoint cmd_recv_terminal;

    io_context recv_context, tran_context;
    io_context cmd_recv_context, cmd_tran_context;

    udp::socket recv_socket, tran_socket;
    udp::socket cmd_recv_socket, cmd_tran_socket;

    uint32_t recv_port, trans_port;
    uint32_t cmd_recv_port, cmd_trans_port;
    std::string target_ip;

    std::string cmd_recv_buf;
    uint32_t stop_cmd_received;
    uint32_t start_cmd_received;
    uint32_t recv_counter;
    uint32_t tran_counter;

  public:
    asio_udp_helper(
        // remote connect target
        const std::string ip_addr,
        // receive port
        uint32_t r_port,
        // transmit port
        uint32_t t_port,
        // command receive port
        uint32_t cr_port,
        // command transmit port
        uint32_t ct_port)
        :
#ifdef ASIO_UDP_HELPER_SERVER_MODE
          // 1. 构造远程目标地址 (Target Device IP)
          tran_terminal(ip::make_address(ip_addr), static_cast<ip::port_type>(r_port)),
          cmd_tran_terminal(ip::make_address(ip_addr), static_cast<ip::port_type>(cr_port)),

          // 2. 构造本地监听地址 (绑定 0.0.0.0 以支持所有网卡接收)
          recv_terminal(udp::v4(), static_cast<ip::port_type>(t_port)),
          cmd_recv_terminal(udp::v4(), static_cast<ip::port_type>(ct_port)),
#else  // client mode                                                                                                  \
       // 1. 构造远程目标地址 (Target Device IP)
          tran_terminal(ip::make_address(ip_addr), static_cast<ip::port_type>(t_port)),
          cmd_tran_terminal(ip::make_address(ip_addr), static_cast<ip::port_type>(ct_port)),

          // 2. 构造本地监听地址 (绑定 0.0.0.0 以支持所有网卡接收)
          recv_terminal(udp::v4(), static_cast<ip::port_type>(r_port)),
          cmd_recv_terminal(udp::v4(), static_cast<ip::port_type>(cr_port)),
#endif // ASIO_UDP_HELPER_SERVER_MODE

          recv_socket(recv_context), tran_socket(tran_context), cmd_recv_socket(cmd_recv_context),
          cmd_tran_socket(cmd_tran_context),

          recv_port(r_port), trans_port(t_port), cmd_recv_port(cr_port), cmd_trans_port(ct_port), target_ip(ip_addr),
          cmd_recv_buf(1024, '\0'), stop_cmd_received(0), start_cmd_received(0), recv_counter(0), tran_counter(0)
    {
    }

    /**
     * @brief 建立物理连接逻辑
     * @note 在 Windows 平台上，reuse_address 的行为与 Linux 略有不同。
     * 在 Linux 下这通常是为了快速重启服务（处于 TIME_WAIT 的端口），但在 Windows 下可能允许两个进程绑定完全相同的端口，
     * 这在调试时可能导致难以发现的数据包被“抢占”的问题。
     */
    void connect_to_target()
    {
        try
        {
            // 数据通道：接收端绑定本地端口
            if (!recv_socket.is_open())
                recv_socket.open(udp::v4());
            recv_socket.set_option(udp::socket::reuse_address(true));
            recv_socket.bind(recv_terminal);

            // 数据通道：发送端连接到远程 IP [修正点]
            if (!tran_socket.is_open())
                tran_socket.open(udp::v4());
            tran_socket.connect(tran_terminal);

            // 控制通道：同样解耦绑定
            if (!cmd_recv_socket.is_open())
                cmd_recv_socket.open(udp::v4());
            cmd_recv_socket.set_option(udp::socket::reuse_address(true));
            cmd_recv_socket.bind(cmd_recv_terminal);

            if (!cmd_tran_socket.is_open())
                cmd_tran_socket.open(udp::v4());
            cmd_tran_socket.connect(cmd_tran_terminal);

            std::cout << "[ASIO-UDP] Link established. Remote: " << target_ip << std::endl;
        }
        catch (std::exception& e)
        {
            std::cerr << "[ASIO-UDP] Connection Error: " << e.what() << std::endl;
        }
    }

    void release_connect()
    {
        try
        {
            // 停止所有异步操作
            cmd_recv_context.stop();

            auto close_socket = [](udp::socket& s) {
                if (s.is_open())
                {
                    s.cancel();
                    s.close();
                }
            };
            close_socket(recv_socket);
            close_socket(tran_socket);
            close_socket(cmd_recv_socket);
            close_socket(cmd_tran_socket);
        }
        catch (...)
        {
        }
    }

    void send_cmd(const char* msg, uint32_t len)
    {
        tran_counter += len;

        cmd_tran_socket.send(buffer(msg, len));
    }

    void send_msg(const char* msg, uint32_t len)
    {
        try
        {

            // 因为使用了 connect()，这里可以直接使用 send
            tran_socket.send(buffer(msg, len));
            tran_counter += len;
        }
        catch (std::exception& e)
        {
            std::cerr << "[ASIO-UDP] Send Error: " << e.what() << std::endl;
        }
    }

    int recv_msg(char* msg, uint32_t len)
    {
        try
        {
            // 接收逻辑保持，由于 bind 了本地端口，它能接收任何发往此端口的数据
            size_t bytes = recv_socket.receive(buffer(msg, len));
            recv_counter += (uint32_t)bytes;
            return 0;
        }
        catch (std::exception& e)
        {
            std::cerr << "[ASIO-UDP] Recv Error: " << e.what() << std::endl;
            return 1;
        }
    }

    void server_ack_cmd()
    {
        cmd_recv_socket.async_receive_from(
            // target
            asio::buffer(cmd_recv_buf), cmd_recv_terminal,
            // listening function
            [this](std::error_code ec, std::size_t bytes_recvd) {
                if (!ec && bytes_recvd > 0)
                {
                    if (cmd_recv_buf.find("Stop") != std::string::npos)
                    {
                        stop_cmd_received = 1;
                        this->release_connect();
                    }
                    else if (cmd_recv_buf.find("Start") != std::string::npos)
                    {
                        start_cmd_received = 1;
                    }
                    std::fill(cmd_recv_buf.begin(), cmd_recv_buf.end(), static_cast<char>(0));
                }
                // 继续监听
                if (!cmd_recv_context.stopped())
                    this->server_ack_cmd();
            });

        // 在独立线程运行，不阻塞 MATLAB 主回路
        std::thread([this]() {
            io_context::work work(cmd_recv_context);
            cmd_recv_context.run_one();
        }).detach();
    }

    void set_overtime()
    {
#if defined __linux__

#if !defined(DISABLE_ASIO_HELPER_TIMEOUT_OPTION)
        // Linux 下推荐使用 native_handle 直接设置，以确保兼容性
        struct timeval tv;
        tv.tv_sec = GMP_ASIO_UDP_LINK_TIMEOUT / 1000;           // 秒
        tv.tv_usec = (GMP_ASIO_UDP_LINK_TIMEOUT % 1000) * 1000; // 微秒

        if (setsockopt(recv_socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0)
        {
            std::cerr << "setsockopt SO_RCVTIMEO failed\r\n";
        }
#endif

#else // Windows 平台

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

#endif // __linux__
    }

    // JSON 解析部分保持逻辑，但内部构造函数已解耦 IP
    static asio_udp_helper* parse_network_config(std::string config_file)
    {
        std::ifstream f(config_file);

        if (!f.is_open())
        {
            std::cout << "[WARN] Cannot find network config file." << std::endl;
            return nullptr;
        }

        // config json
        json config;

        try
        {
            config = json::parse(f);
        }
        catch (const std::exception& e)
        {
            std::cout << "[ERRO] Cannot parse .json file, " << config_file << std::endl;
            std::cerr << e.what() << std::endl;

            return nullptr;
        }

        // read config and create an object
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
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;

            return nullptr;
        }
    }

    //return new asio_udp_helper(config["target_address"], config["receive_port"], config["transmit_port"],
    //                           config["command_recv_port"], config["command_trans_port"]);
};
