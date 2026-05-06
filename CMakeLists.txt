# Hyundai Motor Company ART Gripper — standalone EtherCAT daemon.
#
# This is the ROS-free reorganisation of the official `art_gripper` ROS 2
# package. It reuses the original EtherCAT driver and control thread sources
# (under src/device, src/thread, src/util — MIT, HMC) and replaces the ROS
# service/topic layer with a small TCP binary protocol so the gripper can be
# driven from any host language without ROS.
cmake_minimum_required(VERSION 3.10)
project(art_gripper_daemon LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unused-but-set-variable -O2 -g)

# IgH EtherLab master (https://etherlab.org). Installed system-wide at
# /opt/etherlab by `scripts/install_etherlab.sh`.
list(APPEND CMAKE_PREFIX_PATH "/opt/etherlab")
find_package(EtherCAT REQUIRED)

file(GLOB DEVICE_SRC src/device/*.cpp)
file(GLOB THREAD_SRC src/thread/*.cpp)
file(GLOB UTIL_SRC   src/util/*.cpp)

add_executable(art_gripper_daemon
    src/main.cpp
    src/server.cpp
    ${DEVICE_SRC}
    ${THREAD_SRC}
    ${UTIL_SRC}
)
target_include_directories(art_gripper_daemon PRIVATE
    include
    src
    src/device
    src/thread
)
target_link_libraries(art_gripper_daemon PRIVATE
    EtherLab::ethercat
    pthread
)

install(TARGETS art_gripper_daemon RUNTIME DESTINATION bin)
install(FILES python/art_gripper_client.py
        DESTINATION share/art_gripper/python)
install(FILES systemd/art-gripper-daemon.service
        DESTINATION share/art_gripper/systemd)
