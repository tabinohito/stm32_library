#pragma once

#include <array>

#include "stm32_library/boards/prindog/profiles.hpp"
#include "realtime_bridge_interface/adapters/stm32/common/comm_ports.hpp"
#include "realtime_bridge_interface/components/comm_port_ref.hpp"
#include "stm32_library/boards/prindog/hardware.hpp"

namespace stm32_library::boards::prindog {

using realtime_bridge_interface::components::BasicCommPortRegistry;
using realtime_bridge_interface::adapters::stm32::common::CanCommPort;
using realtime_bridge_interface::components::CommPortRef;
using realtime_bridge_interface::components::SafetyPins;
using realtime_bridge_interface::adapters::stm32::common::UartCommPort;

template <BuildMode Mode>
struct CommPorts;

template <>
struct CommPorts<BuildMode::UsbDebug> {
    UartCommPort<> uart0;
    UartCommPort<> uart1;
    BasicCommPortRegistry<BoardConfig<BuildMode::UsbDebug>::PortNum> registry;

    explicit CommPorts(stm32_library::boards::prindog::Peripherals& peripherals)
        : uart0(peripherals.usb_uart),
          uart1(peripherals.stlink_uart),
          registry(std::array<CommPortRef, 2>{
              CommPortRef::make_uart("usb_uart", 0, uart0),
              CommPortRef::make_uart("stlink_uart", 1, uart1),
          })
    {
    }

    void setup() { registry.setup_all(); }
};

template <>
struct CommPorts<BuildMode::Production> {
    UartCommPort<> uart0;
    UartCommPort<> uart1;
    UartCommPort<> uart2;
    BasicCommPortRegistry<BoardConfig<BuildMode::Production>::PortNum> registry;

    explicit CommPorts(stm32_library::boards::prindog::Peripherals& peripherals)
        : uart0(peripherals.rs485_uart2),
          uart1(peripherals.rs485_uart3),
          uart2(peripherals.rs485_uart4),
          registry(std::array<CommPortRef, 3>{
              CommPortRef::make_uart(
                  "prindog_uart0",
                  0,
                  uart0,
                  SafetyPins{
                      &peripherals.emergency[0],
                      &peripherals.estop[0]
                  }
              ),
              CommPortRef::make_uart(
                  "prindog_uart1",
                  1,
                  uart1,
                  SafetyPins{
                      &peripherals.emergency[1],
                      &peripherals.estop[1]
                  }
              ),
              CommPortRef::make_uart(
                  "prindog_uart2",
                  2,
                  uart2,
                  SafetyPins{
                      &peripherals.emergency[2],
                      &peripherals.estop[2]
                  }
              ),
          })
    {
    }

    void setup() { registry.setup_all(); }
};

template <>
struct CommPorts<BuildMode::Dynamixel> {
    UartCommPort<> uart0;
    UartCommPort<> uart1;
    UartCommPort<> uart2;
    UartCommPort<> uart3;
    UartCommPort<> uart4;
    UartCommPort<> uart5;
    BasicCommPortRegistry<BoardConfig<BuildMode::Dynamixel>::PortNum> registry;

    explicit CommPorts(stm32_library::boards::prindog::Peripherals& peripherals)
        : uart0(peripherals.rs485_uart2),
          uart1(peripherals.rs485_uart3),
          uart2(peripherals.rs485_uart4),
          uart3(peripherals.rs485_uart5),
          uart4(peripherals.rs485_uart7),
          uart5(peripherals.rs485_uart8),
          registry(std::array<CommPortRef, 6>{
              CommPortRef::make_uart("dynamixel_uart0", 0, uart0),
              CommPortRef::make_uart("dynamixel_uart1", 1, uart1),
              CommPortRef::make_uart("dynamixel_uart2", 2, uart2),
              CommPortRef::make_uart("dynamixel_uart3", 3, uart3),
              CommPortRef::make_uart("dynamixel_uart4", 4, uart4),
              CommPortRef::make_uart("dynamixel_uart5", 5, uart5),
          })
    {
    }

    void setup() { registry.setup_all(); }
};

template <>
struct CommPorts<BuildMode::CanBridge> {
    CanCommPort<> can1;
    CanCommPort<> can2;
    CanCommPort<> can3;
    BasicCommPortRegistry<BoardConfig<BuildMode::CanBridge>::PortNum> registry;

    explicit CommPorts(stm32_library::boards::prindog::Peripherals& peripherals)
        : can1(peripherals.can1, 0, true, true),
          can2(peripherals.can2, 0, true, true),
          can3(peripherals.can3, 0, true, true),
          registry(std::array<CommPortRef, 3>{
              CommPortRef::make_can(
                  "can1",
                  0,
                  can1,
                  SafetyPins{&peripherals.emergency[0]}
              ),
              CommPortRef::make_can(
                  "can2",
                  1,
                  can2,
                  SafetyPins{&peripherals.emergency[1]}
              ),
              CommPortRef::make_can(
                  "can3",
                  2,
                  can3,
                  SafetyPins{&peripherals.emergency[2]}
              ),
          })
    {
    }

    void setup() { registry.setup_all(); }
};

} // namespace stm32_library::boards::prindog
