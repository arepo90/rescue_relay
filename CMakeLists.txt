# Does this even work???????

cmake_minimum_required(VERSION 3.10)
project(rescue_relay)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(WIN32)
    message(STATUS "WIN32 DETECTED")

    set(Python3_ROOT_DIR "C:/Python38")
    set(PYTHON_EXECUTABLE "C:/Python38/python.exe")
    set(Python3_EXECUTABLE "C:/Python38/python.exe")
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
    set(CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows" CACHE STRING "")
    
    find_package(portaudio CONFIG REQUIRED)
    find_package(Opus CONFIG REQUIRED)
    list(APPEND CMAKE_MODULE_PATH "C:/vcpkg/installed/x64-windows/share/ffmpeg")
    find_package(FFmpeg REQUIRED COMPONENTS avcodec avformat avutil swscale)
elseif(UNIX)
    message(STATUS "UNIX DETECTED")

    find_package(Threads REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(AVCODEC REQUIRED libavcodec)
    pkg_check_modules(AVFORMAT REQUIRED libavformat)
    pkg_check_modules(AVUTIL REQUIRED libavutil)
    pkg_check_modules(SWSCALE REQUIRED libswscale)

    pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
    pkg_check_modules(OPUS REQUIRED opus)

    pkg_check_modules(SDL2 REQUIRED sdl2)

    #foreach(component IN ITEMS AVCODEC AVFORMAT AVUTIL SWSCALE)
    #    if(${component}_FOUND)
    #        add_library(FFmpeg::${component} INTERFACE IMPORTED)
    #        set_target_properties(FFmpeg::${component} PROPERTIES
    #            INTERFACE_INCLUDE_DIRECTORIES "${${component}_INCLUDE_DIRS}"
    #            INTERFACE_LINK_LIBRARIES "${${component}_LINK_LIBRARIES}"
    #        )
    #    endif()
    #endforeach()
    #pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
    #pkg_check_modules(OPUS REQUIRED opus)
endif()

find_package(ZLIB REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

add_executable(publisher src/ros2_publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge image_transport) 
add_executable(subscriber src/ros2_subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge image_transport) 
add_executable(controller src/ros2_controller.cpp)
ament_target_dependencies(controller rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs moveit_msgs) 
add_executable(joystick src/ros2_joystick.cpp)
ament_target_dependencies(joystick rclcpp sensor_msgs) 

if(WIN32)
    add_executable(relay src/ros2_relay.cpp)
    target_link_libraries(relay 
        Opus::opus 
        portaudio 
        ws2_32 
        Threads::Threads
        ${OpenCV_LIBS} 
        ${AVCODEC_LIB}
        ${AVUTIL_LIB}
        ${SWSCALE_LIB}
    )
elseif(UNIX)
    add_executable(relay src/ros2_relay_linux.cpp)
    target_link_libraries(relay
        ${OPUS_LIBRARIES}
        ${PORTAUDIO_LIBRARIES}
        pthread
        ${OpenCV_LIBS}
        ${AVCODEC_LIBRARIES}
        ${AVUTIL_LIBRARIES}
        ${SWSCALE_LIBRARIES}
    )
    target_include_directories(relay PRIVATE 
        ${PORTAUDIO_INCLUDE_DIRS}
        ${OPUS_INCLUDE_DIRS}
        ${AVCODEC_INCLUDE_DIRS}
        ${AVFORMAT_INCLUDE_DIRS}
        ${AVUTIL_INCLUDE_DIRS}
        ${SWSCALE_INCLUDE_DIRS}
    )
    target_include_directories(joystick PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(joystick ${SDL2_LIBRARIES})
    target_link_libraries(subscriber ${OpenCV_LIBS})
endif()
ament_target_dependencies(relay rclcpp std_msgs sensor_msgs geometry_msgs)

install(TARGETS publisher DESTINATION lib/${PROJECT_NAME})
install(TARGETS subscriber DESTINATION lib/${PROJECT_NAME})
install(TARGETS relay DESTINATION lib/${PROJECT_NAME})
install(TARGETS controller DESTINATION lib/${PROJECT_NAME})
install(TARGETS joystick DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()