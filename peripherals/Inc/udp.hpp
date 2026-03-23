/*
 * udp.hpp
 *
 *  Created on: 2026/03/23
 *      Author: tako
 */
#pragma once
#ifndef LWIP_UDP

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include <functional>
#include <cstring>

class UdpSocket {
public:
    // コンストラクタ：ポート番号と、受信時のコールバックを受け取る
    // コールバックには「誰から来たか（IP/Port）」も渡せるように拡張しています
    using RecvCallback = std::function<void(pbuf* p, const ip_addr_t* addr, uint16_t port)>;

    UdpSocket(uint16_t local_port, RecvCallback callback) : callback_(callback) {
        pcb_ = udp_new();
        if (pcb_ != nullptr) {
            // 自分のポートをバインド（送受信共通で使う）
            udp_bind(pcb_, IP_ADDR_ANY, local_port);
            
            // 受信コールバックの登録
            if (callback_) {
                udp_recv(pcb_, UdpSocket::recv_callback_static, this);
            }
        }
    }

    ~UdpSocket() {
        if (pcb_ != nullptr) {
            udp_remove(pcb_); // 必須：PCBの解放
        }
    }

    // 特定の相手に送るための関数（udp_sendtoを使用）
    bool sendTo(const char* data, size_t len, const ip_addr_t* dest_ip, uint16_t dest_port) {
        if (pcb_ == nullptr) return false;

        struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        if (p == nullptr) {
            return false; // メモリ枯渇対策
        }
        
        memcpy(p->payload, data, len);
        
        // udp_send ではなく udp_sendto を使うことで、都度宛先を指定できる
        err_t err = udp_sendto(pcb_, p, dest_ip, dest_port);
        
        pbuf_free(p); // メモリ解放
        return (err == ERR_OK);
    }

private:
    // LwIPから呼ばれるC言語スタイルの静的コールバック
    static void recv_callback_static(void* arg, udp_pcb* upcb, pbuf* p, const ip_addr_t* addr, u16_t port) {
        if (p == nullptr) return;

        UdpSocket* socket = reinterpret_cast<UdpSocket*>(arg);
        if (socket && socket->callback_) {
            // 受信したデータと共に、送信元のIPとポート番号をユーザーに渡す
            socket->callback_(p, addr, port);
        }
        
        pbuf_free(p); // ここで必ず解放する
    }
    
    udp_pcb* pcb_;
    RecvCallback callback_;
};

#endif 
