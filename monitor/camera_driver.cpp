/**
 * @file camera_driver.cpp
 * @brief V4L2 摄像头硬件抽象层与推流线程
 */
#include "camera_driver.h"
#include "global_state.h"
#include "config.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <thread>

using namespace cv;
using namespace std;

namespace CameraManager
{
    bool forceCameraFormat(int dev_id, bool silent = false)
    {
        char dev_name[32];
        sprintf(dev_name, "/dev/video%d", dev_id);
        int fd = open(dev_name, O_RDWR);
        if (fd < 0) return false;
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = SystemConfig::CAM_WIDTH;
        fmt.fmt.pix.height = SystemConfig::CAM_HEIGHT;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
        {
            if (!silent) std::cerr << "[底层强控] 警告：格式强制指令失败！" << std::endl;
        }
        else
        {
            if (!silent) std::cout << "[底层强控] 请求 1280x720 MJPG。硬件分配: " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height << std::endl;
        }
        struct v4l2_streamparm parm;
        memset(&parm, 0, sizeof(parm));
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = SystemConfig::CAM_FPS;
        ioctl(fd, VIDIOC_S_PARM, &parm);
        close(fd);
        return true;
    }

    void cameraThreadFunc(VideoCapture *cap, SharedFrame *shared)
    {
        Mat temp;
        int fail_count = 0;
        bool error_reported = false; 

        while (true)
        {
            if (!cap->isOpened() || !cap->read(temp) || temp.empty())
            {
                fail_count++;
                if (fail_count > 30)
                {
                    if (!error_reported)
                    {
                        std::cerr << "\n[Camera Watchdog] 摄像头连接断开！已进入后台静默重连，不打扰其他模块调试..." << std::endl;
                        error_reported = true;
                    }
                    cap->release();
                    usleep(20000000);

                    for (int dev_id = 0; dev_id < 4; ++dev_id)
                    {
                        forceCameraFormat(dev_id, true); 
                        std::vector<int> params = {
                            CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                            CAP_PROP_FRAME_WIDTH, SystemConfig::CAM_WIDTH,
                            CAP_PROP_FRAME_HEIGHT, SystemConfig::CAM_HEIGHT,
                            CAP_PROP_FPS, SystemConfig::CAM_FPS,
                            CAP_PROP_BUFFERSIZE, 1};
                        cap->open(dev_id, CAP_V4L2, params);
                        if (cap->isOpened() && cap->get(CAP_PROP_FRAME_WIDTH) >= 640)
                        {
                            fail_count = 0;
                            error_reported = false; 
                            std::cout << "\n[Camera Watchdog] 摄像头热插拔重连成功！画面恢复更新。\n" << std::endl;
                            break;
                        }
                        else
                        {
                            cap->release();
                        }
                    }
                }
                else
                {
                    usleep(30000);
                }
                continue;
            }

            fail_count = 0;
            error_reported = false;

            {
                lock_guard<mutex> lock(shared->mtx);
                temp.copyTo(shared->frame);
                shared->ready = true;
            }
            shared->cv.notify_one();
        }
    }

    VideoCapture probeAndInit()
    {
        VideoCapture cap;
        bool camera_opened = false;
        for (int retry = 0; retry < 3; ++retry)
        {
            for (int dev_id = 0; dev_id < 4; ++dev_id)
            {
                cout << "[Monitor] 正在探测 /dev/video" << dev_id << "..." << endl;
                forceCameraFormat(dev_id, false);
                usleep(100000);
                std::vector<int> params = {
                    CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                    CAP_PROP_FRAME_WIDTH, SystemConfig::CAM_WIDTH,
                    CAP_PROP_FRAME_HEIGHT, SystemConfig::CAM_HEIGHT,
                    CAP_PROP_FPS, SystemConfig::CAM_FPS,
                    CAP_PROP_BUFFERSIZE, 1};
                cap.open(dev_id, CAP_V4L2, params);
                if (cap.isOpened() && cap.get(CAP_PROP_FRAME_WIDTH) >= 640)
                {
                    cout << "[Monitor] 成功锁定视频节点 /dev/video" << dev_id << endl;
                    camera_opened = true;
                    break;
                }
                else
                {
                    cap.release();
                }
            }
            if (camera_opened) break;
            sleep(1);
        }

        if (!camera_opened)
        {
            cerr << "\n[Monitor] 警告：系统初始化时未检测到摄像头！\n>>> 已进入无摄像头调试模式，后续插上摄像头将自动热插拔重连..." << endl;
        }
        else
        {
            cout << "[Monitor] 实际分辨率: " << cap.get(CAP_PROP_FRAME_WIDTH) << "x" << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
            Mat temp;
            for (int i = 0; i < 5; i++) cap >> temp; 
        }

        return cap;
    }

    void startCaptureThread(VideoCapture &cap, SharedFrame &shared)
    {
        g_cap_ptr = &cap;
        thread cam_thread(cameraThreadFunc, &cap, &shared);
        cam_thread.detach();
    }

    bool getLatestFrame(SharedFrame &shared, Mat &frame, int timeout_ms)
    {
        unique_lock<mutex> lock(shared.mtx);
        if (!shared.cv.wait_for(lock, chrono::milliseconds(timeout_ms), [&shared] { return shared.ready; }))
            return false;
        if (shared.frame.empty()) return false;
        shared.frame.copyTo(frame);
        shared.ready = false;
        return true;
    }
}