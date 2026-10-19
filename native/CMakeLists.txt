cmake_minimum_required(VERSION 3.20)
project(autohaul_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Static CRT: the DLL must not depend on a VC++ redistributable inside the game process.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(SCS_SDK_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/scs_sdk_1_15/include)

add_library(autohaul_bridge SHARED
    bridge/src/bridge.cpp
    bridge/autohaul_bridge.def
)
target_include_directories(autohaul_bridge PRIVATE ${SCS_SDK_INCLUDE} bridge)
target_compile_definitions(autohaul_bridge PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
target_compile_options(autohaul_bridge PRIVATE /W4 /permissive- /utf-8)
target_link_options(autohaul_bridge PRIVATE /DEBUG /OPT:REF /OPT:ICF)

# Test harness: loads the DLL and plays the game's role (telemetry + input SDK).
add_executable(fake_game tests/fake_game.cpp)
target_include_directories(fake_game PRIVATE ${SCS_SDK_INCLUDE})
target_compile_definitions(fake_game PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
target_compile_options(fake_game PRIVATE /W4 /permissive- /utf-8)
