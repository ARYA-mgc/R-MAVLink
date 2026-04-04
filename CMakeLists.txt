cmake_minimum_required(VERSION 3.16)
project(R-MAVLink VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler Flags ─────────────────────────────────────────────────────────
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── Include Dirs ───────────────────────────────────────────────────────────
include_directories(${CMAKE_SOURCE_DIR}/include)

# ── Core Library ───────────────────────────────────────────────────────────
add_library(rmavlink_core STATIC
    core/reliable_protocol.cpp
    core/packet_manager.cpp
    transport/udp.cpp
    transport/uart.cpp
    mavlink/mavlink_parser.cpp
)

target_include_directories(rmavlink_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# ── Main Demo Executable ───────────────────────────────────────────────────
add_executable(rmavlink main.cpp)
target_link_libraries(rmavlink rmavlink_core)

# ── Packet Loss Simulation ─────────────────────────────────────────────────
add_executable(packet_loss_sim
    tests/packet_loss_sim.cpp
)
target_include_directories(packet_loss_sim PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# ── Link threading library ────────────────────────────────────────────────
find_package(Threads REQUIRED)
target_link_libraries(rmavlink Threads::Threads)
target_link_libraries(packet_loss_sim Threads::Threads)

# ── Optional: Windows socket library ─────────────────────────────────────
if(WIN32)
    target_link_libraries(rmavlink ws2_32)
    target_link_libraries(rmavlink_core ws2_32)
endif()

# ── Install ───────────────────────────────────────────────────────────────
install(TARGETS rmavlink DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/rmavlink)

message(STATUS "")
message(STATUS "  R-MAVLink ${PROJECT_VERSION} — Build configuration")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler:   ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")
message(STATUS "  Targets:")
message(STATUS "    rmavlink          — main demo (send/recv/loopback)")
message(STATUS "    packet_loss_sim   — stress test with simulated loss")
message(STATUS "")
