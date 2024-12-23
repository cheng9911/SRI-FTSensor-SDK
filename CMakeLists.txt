cmake_minimum_required(VERSION 3.10)

project(SRI-FTSensor-SDK)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads)
find_package(Boost REQUIRED COMPONENTS system thread)
add_subdirectory(ThirdParty/spdlog-1.14.1)

set(CMAKE_INCLUDE_CURRENT_DIR ON)


add_executable(untest src/untest.cpp src/Logger.cpp
        src/untest.cpp)
target_include_directories(untest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(untest Boost::system Boost::thread Threads::Threads spdlog::spdlog_header_only)
