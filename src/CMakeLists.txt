# Add all the source efiles 
file(GLOB CXX_SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
file(GLOB C_SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.c)

add_executable(${NAME}
    ${CXX_SOURCES}
    ${C_SOURCES}
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
    pico_stdlib
    hardware_watchdog
    micro_ros
    FreeRTOS-Kernel-Static
    FreeRTOS-Kernel-Heap4 
    etl::etl
    pico_motor
    pico_encoder_substep_class
    pico-stepper
)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/include
)
     
# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)