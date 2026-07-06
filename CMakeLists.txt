cmake_minimum_required(VERSION 3.10)

# 添加 C 语言支持，因为包含了 .c 文件
project(MonitorProject C CXX)
set(CMAKE_CXX_STANDARD 14)

find_package(Threads REQUIRED)

# [新增] 引入 OpenMP，这是 NCNN 榨干 CPU 性能必须的底层支持
find_package(OpenMP REQUIRED)

set(OPENCV_INSTALL_DIR "/home/lin/loongson_2k0300_workspace/opencv_la64_install")
# 【修改这里】：在全局把旧路径改成我们刚刚新编译的路径
set(OPENCV_INSTALL_DIR "/home/lin/opencv-4.5.4/build_la32r/install")

# 增加工程的 inc 目录，以便找到 ik_solver.h，以及 NCNN 的头文件
include_directories(
    "${OPENCV_INSTALL_DIR}/include/opencv4"
    "${CMAKE_SOURCE_DIR}/inc"
    "${CMAKE_SOURCE_DIR}/ncnn_new/build_la64/install/include" # [新增] NCNN 头文件
)

# 将 ik_solver.c 一起加入编译列表
add_executable(MonitorApp src/monitor.cpp src/ik_solver.c)

# 确保链接 64 位的 .so 文件
# 在末尾加上 m，链接 Linux 基础数学库，为逆解提供支持
target_link_libraries(MonitorApp PRIVATE
    "${CMAKE_SOURCE_DIR}/ncnn_new/build_la64/install/lib/libncnn.a" # [新增] 静态链接 NCNN
    "${OPENCV_INSTALL_DIR}/lib/libopencv_aruco.so"       # 【新增】强力插入 ArUco 动态库
    "${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so" # [新增] 3D位姿解算与标定库！
    "${OPENCV_INSTALL_DIR}/lib/libopencv_features2d.so" # [新增] 补全 calib3d 的依赖 1
    "${OPENCV_INSTALL_DIR}/lib/libopencv_flann.so"      # [新增] 补全 calib3d 的依赖 2
    "${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_core.so"
    Threads::Threads
    OpenMP::OpenMP_CXX # [新增] 链接 OpenMP
    m
)

# ==========================================
# 编译 PilotApp (机械臂网络控制端)
# ==========================================
# 注意路径要带上 src/
add_executable(PilotApp src/pilot.cpp src/ik_solver.c)

# 链接 PilotApp 所需的系统库
# PilotApp 不需要 OpenCV，所以只需链接多线程(Threads)和数学库(m)
target_link_libraries(PilotApp PRIVATE
    Threads::Threads
    m
)