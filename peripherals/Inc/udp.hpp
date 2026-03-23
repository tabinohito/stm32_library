/*
 * udp.hpp
 *
 * Created on: 2026/03/23
 * Author: tako
 */
#pragma once

#include "main.h"
#ifdef HAL_ETH_MODULE_ENABLED

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include <functional>
#include <cstring>
#include <vector> // 追加

namespace stm32_library::stm32_peripherals {

class UdpSocket {
public:
    using RecvCallback = std::function<void(pbuf* p, const ip_addr_t* addr, uint16_t port)>;

    // コンストラクタ：ポートのバインドのみを行うようにシンプル化
    UdpSocket(uint16_t local_port) {
        pcb_ = udp_new();
        if (pcb_ != nullptr) {
            udp_bind(pcb_, IP_ADDR_ANY, local_port);
            
            // LwIPへの静的コールバック登録はここで済ませておく
            udp_recv(pcb_, UdpSocket::recv_callback_static, this);
        }
    }

    ~UdpSocket() {
        if (pcb_ != nullptr) {
            udp_remove(pcb_); // 必須：PCBの解放
        }
    }

    // コールバックを追加するメソッド（Tickerクラスの設計を踏襲）
    void attach(RecvCallback callback) {
        if (callback) {
            callbacks_.push_back(callback);
        }
    }

    // コールバックを全てクリアするメソッド（必要に応じて）
    void clear_callbacks() {
        callbacks_.clear();
    }

    // 特定の相手に送るための関数
    bool sendTo(const char* data, size_t len, const ip_addr_t* dest_ip, uint16_t dest_port) {
        if (pcb_ == nullptr) return false;

        struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        if (p == nullptr) {
            return false; // メモリ枯渇対策
        }
        
        memcpy(p->payload, data, len);
        err_t err = udp_sendto(pcb_, p, dest_ip, dest_port);
        pbuf_free(p); // メモリ解放
        
        return (err == ERR_OK);
    }

private:
    // LwIPから呼ばれるC言語スタイルの静的コールバック
    static void recv_callback_static(void* arg, udp_pcb* upcb, pbuf* p, const ip_addr_t* addr, u16_t port) {
        if (p == nullptr) return;

        UdpSocket* socket = reinterpret_cast<UdpSocket*>(arg);
        if (socket) {
            // 登録されている全てのコールバック関数にパケットを分配
            for (auto& cb : socket->callbacks_) {
                cb(p, addr, port);
            }
        }
        
        // ⚠️ 超重要 ⚠️
        // 全てのユーザーコールバックが完了した後に、ここで1回だけ解放する
        pbuf_free(p); 
    }
    
    udp_pcb* pcb_;
    std::vector<RecvCallback> callbacks_; // 配列化
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_ETH_MODULE_ENABLED
