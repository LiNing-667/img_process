cmake_minimum_required(VERSION 3.10)

# 添加 C 语言支持，因为包含了 .c 文件
project(MonitorProject C CXX)
set(CMAKE_CXX_STANDARD 14)

find_package(Threads REQUIRED)

# 引入 OpenMP，这是 NCNN 榨干 CPU 性能必须的底层支持
find_package(OpenMP REQUIRED)

# 在全局把旧路径改成新编译的路径
set(OPENCV_INSTALL_DIR "/home/lin/opencv-4.5.4/build_la32r/install")

# ==========================================
# 【改动 1】自动搜集 monitor 文件夹下的所有 .cpp 文件
# ==========================================
file(GLOB MONITOR_SRCS "monitor/*.cpp")

# ==========================================
# 【改动 2】增加 monitor 目录到头文件搜索路径
# ==========================================
include_directories(
    "${OPENCV_INSTALL_DIR}/include/opencv4"
    "${CMAKE_SOURCE_DIR}/inc"
    "${CMAKE_SOURCE_DIR}/ncnn_new/build_la64/install/include"
    "${CMAKE_SOURCE_DIR}/monitor" # [新增] 确保能找到拆分出来的 .h 文件
)

# ==========================================
# 【改动 3】将原来写死的 src/monitor.cpp 替换为 ${MONITOR_SRCS} 变量
# ==========================================
add_executable(MonitorApp ${MONITOR_SRCS} src/ik_solver.c)

# ==========================================
# 【手机遥控页面】编译后自动拷贝到二进制同目录 (build/)
# 这样部署时把 MonitorApp 连同 index.html 一起拷走即可
# ==========================================
add_custom_command(TARGET MonitorApp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/monitor/web/index.html"
    "$<TARGET_FILE_DIR:MonitorApp>/index.html"
    COMMENT "拷贝手机遥控页面 web/index.html 到二进制同目录"
)

# 确保链接 64 位的 .so 文件
# 在末尾加上 m，链接 Linux 基础数学库，为逆解提供支持
target_link_libraries(MonitorApp PRIVATE
    "${CMAKE_SOURCE_DIR}/ncnn_new/build_la64/install/lib/libncnn.a" # 静态链接 NCNN
    "${OPENCV_INSTALL_DIR}/lib/libopencv_aruco.so" # 强力插入 ArUco 动态库
    "${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so" # 3D位姿解算与标定库
    "${OPENCV_INSTALL_DIR}/lib/libopencv_features2d.so" # 补全 calib3d 的依赖 1
    "${OPENCV_INSTALL_DIR}/lib/libopencv_flann.so" # 补全 calib3d 的依赖 2
    "${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_core.so"
    Threads::Threads
    OpenMP::OpenMP_CXX # 链接 OpenMP
    m
)

# ==========================================
# 编译 PilotApp (机械臂网络控制端)
# ==========================================
file(GLOB PILOT_SRCS "pilot/*.cpp")

add_executable(PilotApp ${PILOT_SRCS} src/ik_solver.c)

# 头文件查找路径，支持找到新拆分的 .h
target_include_directories(PilotApp PRIVATE "${CMAKE_SOURCE_DIR}/src/pilot")

target_link_libraries(PilotApp PRIVATE
    Threads::Threads
    m
)

# ==========================================
# 【EvalApp 编译规则】(考场能力测评工程)
# 自动搜集 eval_app/src 目录下的所有 C 和 C++ 源文件
# ==========================================
file(GLOB EVAL_SRCS "eval_app/src/*.cpp" "eval_app/src/*.c")

add_executable(EvalApp ${EVAL_SRCS})

# 包含测评工程的头文件目录
target_include_directories(EvalApp PRIVATE
    "${CMAKE_SOURCE_DIR}/eval_app/inc"
    "${OPENCV_INSTALL_DIR}/include/opencv4"
)

# 链接多线程库与 Linux 数学库
target_link_libraries(EvalApp PRIVATE
    Threads::Threads
    OpenMP::OpenMP_CXX
    m
)