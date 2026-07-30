/*
 * prindog_hardware.hpp
 *
 *  Created on: 2026/03/12
 *      Author: tako
 */

#pragma once

#include "main.h"
#include <array>
#include <memory>
#include <string.h>
#include "stm32_library/peripherals/stm32_peripherals.hpp"
#include "stm32_library/modules/stm32_modules.hpp"

extern "C" {
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "ethernetif.h"
#include "lwip/timeouts.h"
#include "lwip.h"
}

/* ST-Link */
extern UART_HandleTypeDef huart6;

/* USB Serial */
extern UART_HandleTypeDef huart1;

/* RS485 */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;

/* CAN */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

/* Voltage and Current sensor */
extern I2C_HandleTypeDef hi2c3;

/* ADC for thermistor */
extern SPI_HandleTypeDef hspi4;

/* Buzzer */
extern TIM_HandleTypeDef htim1;

/* Timer 10kHz */
extern TIM_HandleTypeDef htim6;

/* Ethernet */
extern ETH_HandleTypeDef heth;
extern netif gnetif;

/* Timer 100Hz */
//extern TIM_HandleTypeDef htim13;

namespace stm32_library::boards::prindog {
    namespace stm32_peripherals = stm32_library::stm32_peripherals;
    namespace stm32_modules = stm32_library::stm32_modules;

    struct Peripherals {
    private:
        Peripherals()
        {
        }
        Peripherals(Peripherals const &) = delete;
        void operator=(Peripherals const &) = delete;

    public:
        static Peripherals &get_instance() {
            static Peripherals instance;
            return instance;
        }

        static void enable_std_printf() {
            stm32_peripherals::enable_std_printf(&huart1);
        }

        /* USB-UART */
        stm32_peripherals::Uart usb_uart{&huart1};

        /* ST-Link UART */
        stm32_peripherals::Uart stlink_uart{&huart6};

        /* RS485-UART */
        stm32_peripherals::Uart rs485_uart2{&huart2};
        stm32_peripherals::DigitalOut rs485_uart2_de{GPIOD, GPIO_PIN_4};

        stm32_peripherals::Uart rs485_uart3{&huart3};
        stm32_peripherals::DigitalOut rs485_uart3_de{GPIOD, GPIO_PIN_12};

        stm32_peripherals::Uart rs485_uart4{&huart4};
        stm32_peripherals::DigitalOut rs485_uart4_de{GPIOA, GPIO_PIN_15};

        stm32_peripherals::Uart rs485_uart5{&huart5};
        stm32_peripherals::DigitalOut rs485_uart5_de{GPIOC, GPIO_PIN_8};

        stm32_peripherals::Uart rs485_uart7{&huart7};
        stm32_peripherals::DigitalOut rs485_uart7_de{GPIOE, GPIO_PIN_9};

        stm32_peripherals::Uart rs485_uart8{&huart8};
        stm32_peripherals::DigitalOut rs485_uart8_de{GPIOD, GPIO_PIN_15};

        /* gpio */
		std::array<stm32_peripherals::DigitalOut,8> leds = {
                stm32_peripherals::DigitalOut{GPIOB, GPIO_PIN_7},
                stm32_peripherals::DigitalOut{GPIOD, GPIO_PIN_7},
                stm32_peripherals::DigitalOut{GPIOD, GPIO_PIN_3},
                stm32_peripherals::DigitalOut{GPIOD, GPIO_PIN_2},
                stm32_peripherals::DigitalOut{GPIOC, GPIO_PIN_12},
                stm32_peripherals::DigitalOut{GPIOB, GPIO_PIN_14},
                stm32_peripherals::DigitalOut{GPIOB, GPIO_PIN_15},
                stm32_peripherals::DigitalOut{GPIOA, GPIO_PIN_12},
        };

        /* Buzzer */
        stm32_peripherals::PwmOut timer_buzzer{&htim1, TIM_CHANNEL_2};

        /* timer */
        stm32_peripherals::Ticker timer_10khz{&htim6};

        /* CAN */
        stm32_peripherals::Can can1{&hcan1};
        stm32_peripherals::Can can2{&hcan2};
        stm32_peripherals::Can can3{&hcan3};

        /* Estop */
	    std::array<stm32_peripherals::DigitalOut,3> estop = {
	            stm32_peripherals::DigitalOut{GPIOD, GPIO_PIN_1},
				stm32_peripherals::DigitalOut{GPIOB, GPIO_PIN_6},
				stm32_peripherals::DigitalOut{GPIOB, GPIO_PIN_4},
		};

	    /* I2C */
	    stm32_peripherals::I2c i2c{&hi2c3};
	    std::array<stm32_peripherals::DigitalIn,3> alert = {
				stm32_peripherals::DigitalIn{GPIOE, GPIO_PIN_10},
				stm32_peripherals::DigitalIn{GPIOE, GPIO_PIN_15},
				stm32_peripherals::DigitalIn{GPIOE, GPIO_PIN_12},
	    };

	    std::array<stm32_peripherals::DigitalOut,3> emergency = {
	            stm32_peripherals::DigitalOut{GPIOE, GPIO_PIN_14},
				stm32_peripherals::DigitalOut{GPIOB, GPIO_PIN_10},
				stm32_peripherals::DigitalOut{GPIOE, GPIO_PIN_13},
		};

	    /* SPI */
	    stm32_peripherals::Spi spi{&hspi4};
	    stm32_peripherals::DigitalOut nss{GPIOE,GPIO_PIN_4};

        /* UDP */
        const uint16_t local_udp_port = 5005;
        static constexpr size_t RealtimeUdpPayloadSize = 512;

        stm32_peripherals::UdpSocket udp_socket{local_udp_port};

        stm32_peripherals::UdpRxQueue<128, RealtimeUdpPayloadSize> udp_rx_queue;
        stm32_peripherals::UdpTxQueue<16, RealtimeUdpPayloadSize> udp_tx_queue;

	};

    struct Modules {
		private:
			Modules(){
				using namespace stm32_modules;
				auto &peri = Peripherals::get_instance();
			    stm32_modules::dwt_init();

				auto &timer_buzzer = peri.timer_buzzer;
				buzzer = new stm32_modules::Tone(timer_buzzer);

			    auto &i2c = peri.i2c;
			    for (size_t i = 0; i < ina238.size(); i++) {
			      ina238[i] = new stm32_modules::INA238(i2c, kIna238Addresses[i]);
				  if (ina238[i] != nullptr && ina238[i]->ok()) {
					  ina238[i]->setShunt(0.001f, 50.0f);
					  ina238[i]->setAveragingCount(stm32_modules::INA2xx::AveragingCount::Count16);
					  ina238[i]->setVoltageConversionTime(stm32_modules::INA2xx::ConversionTime::Time150us);
					  ina238[i]->setCurrentConversionTime(stm32_modules::INA2xx::ConversionTime::Time280us);

			      }
			    }

			    static uint8_t rs485_uart2_tx_queue[32768] __attribute__((aligned(32))) = {};
			    static uint8_t rs485_uart3_tx_queue[32768] __attribute__((aligned(32))) = {};
			    static uint8_t rs485_uart4_tx_queue[32768] __attribute__((aligned(32))) = {};
			    static uint8_t rs485_uart5_tx_queue[8192] __attribute__((aligned(32))) = {};
			    static uint8_t rs485_uart7_tx_queue[8192] __attribute__((aligned(32))) = {};
			    static uint8_t rs485_uart8_tx_queue[8192] __attribute__((aligned(32))) = {};

			    peri.rs485_uart2.use_dma_transmit(true);
			    peri.rs485_uart2.use_dma_transmit_queue(
			        rs485_uart2_tx_queue,
			        sizeof(rs485_uart2_tx_queue)
			    );
			    peri.rs485_uart2.reset_tx_counters();

			    peri.rs485_uart3.use_dma_transmit(true);
			    peri.rs485_uart3.use_dma_transmit_queue(
			        rs485_uart3_tx_queue,
			        sizeof(rs485_uart3_tx_queue)
			    );
			    peri.rs485_uart3.reset_tx_counters();

			    peri.rs485_uart4.use_dma_transmit(true);
			    peri.rs485_uart4.use_dma_transmit_queue(
			        rs485_uart4_tx_queue,
			        sizeof(rs485_uart4_tx_queue)
			    );
			    peri.rs485_uart4.reset_tx_counters();

			    peri.rs485_uart5.use_dma_transmit(true);
			    peri.rs485_uart5.use_dma_transmit_queue(
			        rs485_uart5_tx_queue,
			        sizeof(rs485_uart5_tx_queue)
			    );
			    peri.rs485_uart5.reset_tx_counters();

			    peri.rs485_uart7.use_dma_transmit(true);
			    peri.rs485_uart7.use_dma_transmit_queue(
			        rs485_uart7_tx_queue,
			        sizeof(rs485_uart7_tx_queue)
			    );
			    peri.rs485_uart7.reset_tx_counters();

			    peri.rs485_uart8.use_dma_transmit(true);
			    peri.rs485_uart8.use_dma_transmit_queue(
			        rs485_uart8_tx_queue,
			        sizeof(rs485_uart8_tx_queue)
			    );
			    peri.rs485_uart8.reset_tx_counters();

			    auto &spi = peri.spi;
			    auto &nss = peri.nss;
			    mcp3204 = new stm32_modules::MCP3204(vref,nss,spi);
			}

			Modules(Modules const &) = delete;
			void operator=(Modules const &) = delete;

		public:
			static Modules &get_instance() {
				static Modules modules;
				return modules;
			}

			stm32_library::stm32_modules::Tone* buzzer;
			std::array<stm32_library::stm32_modules::INA238*,3> ina238;
			stm32_library::stm32_modules::MCP3204* mcp3204;

		private:
			static constexpr std::array<uint8_t, 3> kIna238Addresses{
			  0x41, 0x44, 0x40
			};

			static constexpr float vref = 2.495;

		};

} // namespace stm32_library::boards::prindog
