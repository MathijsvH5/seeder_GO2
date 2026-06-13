cmake_minimum_required(VERSION 3.10)
project(Go2CustomProject)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(unitree_sdk2 REQUIRED)

add_executable(test1_real test1_real.cpp)
target_link_libraries(test1_real unitree_sdk2)

add_executable(final_movement final_movement.cpp)
target_link_libraries(final_movement unitree_sdk2)

add_executable(test2_real test2_real.cpp)
target_link_libraries(test2_real unitree_sdk2)

add_executable(test3_real test3_real.cpp)
target_link_libraries(test3_real unitree_sdk2)

add_executable(test4_real test4_real.cpp)
target_link_libraries(test4_real unitree_sdk2)

add_executable(final_movement_v2 final_movement_v2.cpp)
target_link_libraries(final_movement_v2 unitree_sdk2)

add_executable(final_movement_v3 final_movement_v3.cpp)
target_link_libraries(final_movement_v3 unitree_sdk2)

add_executable(test_esp test_esp.cpp)
target_link_libraries(test_esp unitree_sdk2)

add_executable(final_with_ll_v1 final_with_ll_v1.cpp)
target_link_libraries(final_with_ll_v1 unitree_sdk2)

add_executable(final_with_ll_v2 final_with_ll_v2.cpp)
target_link_libraries(final_with_ll_v2 unitree_sdk2)

add_executable(test1_lowlevel test1_lowlevel.cpp)
target_link_libraries(test1_lowlevel unitree_sdk2)

add_executable(try try.cpp)
target_link_libraries(try unitree_sdk2)

add_executable(try2 try2.cpp)
target_link_libraries(try2 unitree_sdk2)

add_executable(try3 try3.cpp)
target_link_libraries(try3 unitree_sdk2)

add_executable(try4 try4.cpp)
target_link_libraries(try4 unitree_sdk2)

add_executable(body_height body_height.cpp)
target_link_libraries(body_height unitree_sdk2)

add_executable(remote_emulator remote_emulator.cpp)
target_link_libraries(remote_emulator unitree_sdk2)

add_executable(fast_planting fast_planting.cpp)
target_link_libraries(fast_planting unitree_sdk2)