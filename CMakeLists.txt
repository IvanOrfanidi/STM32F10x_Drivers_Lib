# Version CMake
cmake_minimum_required(VERSION 3.13)

add_library(STM32F10x_Drivers_Lib INTERFACE)

target_sources(STM32F10x_Drivers_Lib INTERFACE
    # CMSIS
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/cmsis_armcc.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/cmsis_compiler.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/cmsis_gcc.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/cmsis_iccarm.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/cmsis_version.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_armv8mbl.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_armv8mml.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_cm0.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_cm0plus.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_cm1.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_cm3.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_cm4.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_cm7.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_cm23.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_cm33.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_sc000.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/core_sc300.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/mpu_armv7.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/mpu_armv8.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/stm32f10x.h
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include/tz_context.h

    # Utils
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/clock_setup.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/non_copyable.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/non_movable.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/observer.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/pin_interface.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/ring_buffer.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/singleton_dynamic.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/singleton_static.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/soft_timer.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/spi_interface.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/virtual_port.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/utils/lock_interrupt.h

    # Include
    ${CMAKE_CURRENT_LIST_DIR}/inc/clock.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/dwt_timer.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/exti.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/flash.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/interrupt.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/rtc.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/spi.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/systick.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/uart.h
    ${CMAKE_CURRENT_LIST_DIR}/inc/watchdog.h

    # Source
    ${CMAKE_CURRENT_LIST_DIR}/src/clock.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/dwt_timer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/exti.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/flash.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/gpio.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/interrupt.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/rtc.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/spi.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/systick.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/uart.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/watchdog.cpp
)

# Need to access files by < >
target_include_directories(STM32F10x_Drivers_Lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
