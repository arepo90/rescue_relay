cmake_minimum_required(VERSION 3.10)
project(rescue_relay)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(WIN32)
    set(Python3_ROOT_DIR "C:/Python38")
    set(PYTHON_EXECUTABLE "C:/Python38/python.exe")
    set(Python3_EXECUTABLE "C:/Python38/python.exe")
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
    set(CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows" CACHE STRING "")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(portaudio CONFIG REQUIRED)
find_package(Opus CONFIG REQUIRED)
list(APPEND CMAKE_MODULE_PATH "C:/vcpkg/installed/x64-windows/share/ffmpeg")
find_package(FFmpeg REQUIRED COMPONENTS avcodec avformat avutil swscale)
find_package(ZLIB REQUIRED)
add_definitions(-DHAVE_OPENCV_CUDA=1)
find_package(OpenCV REQUIRED COMPONENTS 
    core 
    cudaarithm
    cudafilters
    cudaimgproc
    cudafeatures2d
)

add_executable(publisher src/ros2_publisher.cpp)
target_link_libraries(publisher ${OpenCV_LIBS})
ament_target_dependencies(publisher rclcpp std_msgs sensor_msgs)
add_executable(subscriber src/ros2_subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs sensor_msgs)
if(WIN32)
    add_executable(relay src/ros2_relay.cpp)
    target_link_libraries(relay 
        Opus::opus 
        portaudio 
        ws2_32 
        ${OpenCV_LIBS} 
        ${AVCODEC_LIB}
        ${AVUTIL_LIB}
        ${SWSCALE_LIB}
    )
    target_link_libraries(subscriber ZLIB::ZLIB ${OpenCV_LIBS})
elseif(UNIX)
    add_executable(relay src/ros2_relay_linux.cpp)
    target_link_libraries(relay Opus::opus portaudio pthread ${OpenCV_LIBS})
endif()
ament_target_dependencies(relay rclcpp std_msgs sensor_msgs)

install(TARGETS publisher DESTINATION lib/${PROJECT_NAME})
install(TARGETS subscriber DESTINATION lib/${PROJECT_NAME})
install(TARGETS relay DESTINATION lib/${PROJECT_NAME})
ament_package()