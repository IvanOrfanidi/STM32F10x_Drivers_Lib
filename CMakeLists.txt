# Version CMake
cmake_minimum_required(VERSION 3.13)

add_library(stm32f10x_drivers_lib INTERFACE)

target_sources(stm32f10x_drivers_lib INTERFACE
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
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/clock_setup.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/non_copyable.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/non_movable.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/observer.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/pin_interface.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/ring_buffer.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/singleton_dynamic.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/singleton_static.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/soft_timer.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/spi_interface.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/virtual_port.h
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/lock_interrupt.h

    # Include
    ${CMAKE_CURRENT_LIST_DIR}/include/clock.h
    ${CMAKE_CURRENT_LIST_DIR}/include/dwt_timer.h
    ${CMAKE_CURRENT_LIST_DIR}/include/exti.h
    ${CMAKE_CURRENT_LIST_DIR}/include/flash.h
    ${CMAKE_CURRENT_LIST_DIR}/include/interrupt.h
    ${CMAKE_CURRENT_LIST_DIR}/include/rtc.h
    ${CMAKE_CURRENT_LIST_DIR}/include/spi.h
    ${CMAKE_CURRENT_LIST_DIR}/include/i2c.h
    ${CMAKE_CURRENT_LIST_DIR}/include/systick.h
    ${CMAKE_CURRENT_LIST_DIR}/include/uart.h
    ${CMAKE_CURRENT_LIST_DIR}/include/watchdog.h

    # Source
    ${CMAKE_CURRENT_LIST_DIR}/source/clock.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/dwt_timer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/exti.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/flash.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/gpio.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/interrupt.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/rtc.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/spi.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/i2c.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/systick.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/uart.cpp
    ${CMAKE_CURRENT_LIST_DIR}/source/watchdog.cpp
)

# Need to access files by < >
target_include_directories(stm32f10x_drivers_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
