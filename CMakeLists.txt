project(talonSRX_hardware_interface)

cmake_minimum_required(VERSION 3.1)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        hardware_interface
        )

catkin_package(
        CATKIN_DEPENDS
        roscpp
        hardware_interface
        pluginlib
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
        src/talonSRX_hardware_interface.cpp
        )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

message("Your architecture is " ${CMAKE_LIBRARY_ARCHITECTURE})
message("Your path " ${PROJECT_SOURCE_DIR}/src)
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    message("Jetson TK1 / Pi")
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCCI.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCanutils.a)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2 / Xavier
    message("aarch64")
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_Phoenix.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_PhoenixCCI.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_PhoenixCanutils.a)
else ()
    message("x86-64")
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCanutils.a)
endif ()


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})